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
  
  Serial.println("\n=== HYDROCARE MASTER - SPI SENSOR ACQUISITION ===");
  Serial.println("Reading sensor data from slave via SPI protocol");
  Serial.println("SPI Clock: 10 MHz, Buffer: 20k bytes");
  Serial.println("Waiting for BLE connection to start logging...\n");
}

void loop() {
  //checkUSB();
  
  if (otaUpdateAvailable) {
    uiOTAStarted();
    connectToWiFi();
    performOTAUpdate();
  }
  if (deviceConnected && timerStream == 1 && deviceStatus == 1 && sessionInitialized) {
    uint32_t loopCycleStart = millis();
    uint32_t startTime, duration;
    
    Serial.println("\n========== MAIN LOOP CYCLE START ==========");
    
    // ==================== SENSOR ACQUISITION PHASE ====================
    // Measure all local sensors
    startTime = millis();
    measureBatteryLevel();
    duration = millis() - startTime;
    Serial.printf("[TIMER] measureBatteryLevel: %u ms\n", duration);
    
    startTime = millis();
    measureAmbLight();
    duration = millis() - startTime;
    Serial.printf("[TIMER] measureAmbLight: %u ms\n", duration);
    
    startTime = millis();
    measurePIR();
    duration = millis() - startTime;
    Serial.printf("[TIMER] measurePIR: %u ms\n", duration);
    
    startTime = millis();
    measuremmWave();
    duration = millis() - startTime;
    Serial.printf("[TIMER] measuremmWave: %u ms\n", duration);
    
    // Fetch fresh data from slave
    startTime = millis();
    SensorDataPacket* slaveData = readSlaveData(); 
    duration = millis() - startTime;
    Serial.printf("[TIMER] readSlaveData: %u ms\n", duration);
    
    uint32_t currentTimestamp = millis();
    
    // Update internal ambient light from slave data
    if (slaveData != nullptr) {
      ambLight_Int = slaveData->ambientLight;
    }
    
    // ==================== SD CARD LOGGING PHASE ====================
    // All SD operations grouped together: open once, write all, close once
    startTime = millis();
    
    // LOG SENSOR DATA
    logSensorData(currentTimestamp, 
                  batteryPercentage,
                  ambLight,
                  PIRValue,
                  movingDist,
                  movingEnergy,
                  staticDist,
                  staticEnergy,
                  detectionDist,
                  ambLight_Int,
                  slaveData != nullptr ? slaveData->humidity : 0.0f,
                  slaveData != nullptr ? slaveData->temperature : 0.0f);
    Serial.printf("[SD] Sensor data logged: ts=%u ms\n", currentTimestamp);
    
    // LOG MIC & ACCEL SAMPLES
    if (slaveData != nullptr) {
      logMicAccelSamples(slaveData->accelX_samples, slaveData->accelY_samples, 
                         slaveData->accelZ_samples, slaveData->microphoneSamples, 
                         slaveData->accelSampleCount, currentTimestamp);
    }
    
    // SAVE COLOR CAMERA (RGB565)
    if (slaveData != nullptr) {
      saveRGBImage(slaveData->rgbFrame, currentTimestamp);
    }
    
    // SAVE THERMAL CAMERA (IR)
    if (slaveData != nullptr) {
      saveIRImage(slaveData->irFrame, currentTimestamp);
    }
    
    duration = millis() - startTime;
    Serial.printf("[TIMER] SD LOGGING BATCH: %u ms\n", duration);
    
    // ==================== BLE NOTIFICATION PHASE ====================
    startTime = millis();
    
    // Downsample 64x64 RGB frame to 16x16 for BLE transmission
    if (slaveData != nullptr) {
      downsampleRGBFrame(slaveData->rgbFrame, downsampled16x16);
      memcpy(irFrame16x12, slaveData->irFrame, sizeof(irFrame16x12));
    }
    
    // Send BLE notifications
    notifyAll();
    
    duration = millis() - startTime;
    Serial.printf("[TIMER] BLE PHASE: %u ms\n", duration);
    
    // Print total loop cycle time
    uint32_t totalCycleDuration = millis() - loopCycleStart;
    Serial.printf("[TIMER] TOTAL LOOP CYCLE: %u ms\n", totalCycleDuration);
    Serial.println("========== MAIN LOOP CYCLE END ==========\n");
    
    timerStream = 0;
  }
}
