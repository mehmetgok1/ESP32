#include <Arduino.h>
#include "config/config.h"
#include "ui/ui.h"
#include "measurement/measurement.h"
#include "ble/ble.h"
#include "timer/timer.h"
#include "memory/memory.h"
#include "ota/ota.h"
#include "communication/communication.h"

bool deviceStatus = false; // false = stopped, true = logging
bool sessionInitialized = false; // true = session folders and files ready

// Buffer for downsampled 16x16 RGB565 frame (256 pixels * 2 bytes = 512 bytes)
uint16_t downsampled16x16[256];  // Shared with BLE for transmission
// Buffer for IR thermal frame 16x12 (192 pixels * 2 bytes = 384 bytes)
uint16_t irFrame16x12[192];      // Shared with BLE for transmission

// FreeRTOS Queue for sensor data (non-blocking SD logging)
QueueHandle_t sensorDataQueue = NULL;
const int SENSOR_QUEUE_SIZE = 3;  // Buffer up to 3 packets

// ==================== COMBINED DATA PACKET ====================
// Includes both master sensor readings and slave sensor data
#pragma pack(1)
typedef struct {
  // MASTER SENSOR READINGS
  float batteryLevel;             // Battery voltage
  float batteryPercentage;        // Battery percentage (0-100%)
  float ambLight;                 // Ambient light
  uint16_t ambLight_Int;          // Ambient light (uint16)
  float PIRValue;                 // PIR motion detection
  uint16_t movingDist;            // mmWave moving target distance
  uint8_t movingEnergy;           // mmWave moving target energy
  uint16_t staticDist;            // mmWave static target distance
  uint8_t staticEnergy;           // mmWave static target energy
  uint16_t detectionDist;         // mmWave detection distance
  
  // SLAVE SENSOR PACKET
  SensorDataPacket slaveData;     // All slave sensor data
} CombinedDataPacket;
#pragma pack()

// SD Card logging task (runs independently from main loop)
void sdCardLoggingTask(void *parameter) {
  static CombinedDataPacket packet;  // Static allocation - not on stack
  uint32_t packetsLogged = 0;
  
  Serial.println("[SD-TASK] SD logging task started (independent, ready to queue packets)");
  
  while(1) {
    // Wait for packet from main loop (blocks if queue empty)
    if (xQueueReceive(sensorDataQueue, &packet, portMAX_DELAY)) {
      // Validate packet before logging
      if (packet.slaveData.sequence == 0 || packet.slaveData.temperature == 0.0) {
        Serial.printf("[SD-TASK] Dropped invalid packet (seq=%u, temp=%.1f)\n", 
                     packet.slaveData.sequence, packet.slaveData.temperature);
        continue;  // Skip this packet
      }
      
      // LOGGING OPERATIONS (non-blocking from main loop perspective)
      // Now includes both master and slave data:
      // - logSensorData(&packet) with battery, light, PIR, mmWave
      // - logMicAccelSamples(&packet.slaveData)
      // - saveRGBImage(&packet.slaveData)
      // - saveIRImage(&packet.slaveData)
      
      packetsLogged++;
      if (packetsLogged % 10 == 0) {
        Serial.printf("[SD-TASK] Logged %u packets to SD\n", packetsLogged);
      }
    }
  }
}

void setup() {
  initPins();
  initPeripherals();
  initLed();
  initSD();
  // Session folder will be created when BLE connection starts logging
  uiInit();
  initmmWave();
  initBLE();
  initTimer();
  disableTimer();
  initSPIComm();
  
  // Create FreeRTOS queue for sensor data (queue item size = CombinedDataPacket)
  sensorDataQueue = xQueueCreate(SENSOR_QUEUE_SIZE, sizeof(CombinedDataPacket));
  if (sensorDataQueue == NULL) {
    Serial.println("[ERROR] Failed to create sensor queue!");
  } else {
    Serial.printf("[QUEUE] Created combined data queue (max %d packets)\n", SENSOR_QUEUE_SIZE);
  }
  
  // Create SD logging task (lower priority, gets 4KB stack)
  xTaskCreatePinnedToCore(
    sdCardLoggingTask,      // Task function
    "SDLoggingTask",        // Task name
    4096,                   // Stack size (bytes)
    NULL,                   // Parameter
    1,                      // Priority (lower than main)
    NULL,                   // Task handle
    1                       // Core 1 (leave core 0 for main)
  );
  

  Serial.println("\n=== HYDROCARE MASTER - SPI SENSOR ACQUISITION ===");
  Serial.println("Reading sensor data from slave via SPI protocol");
  Serial.println("SPI Clock: 10 MHz, Buffer: 20k bytes");
  Serial.println("SD logging: Queue-based (non-blocking)\n");
}

void loop() {
  //checkUSB();
  
  if (otaUpdateAvailable) {
    uiOTAStarted();
    connectToWiFi();
    performOTAUpdate();
  }
  if (deviceConnected && timerStream == 1 && deviceStatus == 1 && sessionInitialized) {
    uint32_t loopStart = millis();
    
    // ==================== FETCH SLAVE DATA FIRST ====================
    SensorDataPacket* slaveData = readSlaveData(); 
    
    // ==================== MEASURE MASTER SENSORS ====================
    measureBatteryLevel();
    measureAmbLight();
    measurePIR();
    measuremmWave();
    
    // ==================== COMBINE AND PUSH TO QUEUE ====================
    if (slaveData != nullptr && sensorDataQueue != NULL) {
      // Create combined packet with master + slave data
      static CombinedDataPacket combinedPacket = {};  // Static allocation - not on stack
      
      // Copy master sensor readings
      combinedPacket.batteryLevel = batteryLevel;
      combinedPacket.batteryPercentage = batteryPercentage;
      combinedPacket.ambLight = ambLight;
      combinedPacket.ambLight_Int = ambLight_Int;
      combinedPacket.PIRValue = PIRValue;
      combinedPacket.movingDist = movingDist;
      combinedPacket.movingEnergy = movingEnergy;
      combinedPacket.staticDist = staticDist;
      combinedPacket.staticEnergy = staticEnergy;
      combinedPacket.detectionDist = detectionDist;
      
      // Copy slave sensor data
      memcpy(&combinedPacket.slaveData, slaveData, sizeof(SensorDataPacket));
      
      // Queue the combined packet
      if (xQueueSend(sensorDataQueue, (void *)&combinedPacket, 0) == pdTRUE) {
        Serial.printf("[QUEUE] Combined packet queued\n");
      } else {
        Serial.println("[QUEUE] WARNING: Queue full, overwriting");
        xQueueOverwrite(sensorDataQueue, (void *)&combinedPacket);
      }
    }
    
    // ==================== BLE NOTIFICATION PHASE ====================
    if (slaveData != nullptr) {
      downsampleRGBFrame(slaveData->rgbFrame, downsampled16x16);
      memcpy(irFrame16x12, slaveData->irFrame, sizeof(irFrame16x12));
    }
    notifyAll();
    
    // Total loop execution time
    Serial.printf("[LOOP] Cycle: %u ms (< 1000 ms timer)\n", millis() - loopStart);
    
    timerStream = 0;
  }
}
