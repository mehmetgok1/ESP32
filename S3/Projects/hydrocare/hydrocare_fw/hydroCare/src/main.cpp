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

// Buffer for downsampled 16x16 RGB565 frame (256 pixels * 2 bytes = 512 bytes)
uint16_t downsampled16x16[256];  // Shared with BLE for transmission

void setup() {
  initPins();
  initPeripherals();
  initLed();
  initSD();
  initSessionFolder();  // Create timestamped folder structure
  uiInit();
  initmmWave();
  initBLE();
  initTimer();
  disableTimer();
  initSPIComm();
  
  Serial.println("\n=== HYDROCARE MASTER - SPI SENSOR ACQUISITION ===");
  Serial.println("Reading sensor data from slave via SPI protocol");
  Serial.println("SPI Clock: 10 MHz, Buffer: 20k bytes\n");
}

void loop() {
  //checkUSB();
  
  if (otaUpdateAvailable) {
    uiOTAStarted();
    connectToWiFi();
    performOTAUpdate();
  }
  
  // TODO: Process BLE tasks with down-sampled 16x16 image (4x4 pixel averaging from 64x64)
  // processBLETasks();  // Disabled for now - will update with averaged RGB565 image
  
  if (deviceConnected && timerStream == 1 && deviceStatus == 1) {

    String dataRow;
    
    // Measure all local sensors
    measureBatteryLevel();
    measureAmbLight();
    measurePIR();
    measuremmWave();
    
    uint32_t loopStartTime = millis();
    
    // Fetch fresh data and assign it to our pointer
    SensorDataPacket* slaveData = readSlaveData(); 
    
    uint32_t loopDuration = millis() - loopStartTime;
    Serial.printf("\n[Master] SLAVE sensor READ completed in %u ms\n", loopDuration);
    
    // Update internal ambient light from slave data
    if (slaveData != nullptr) {
      ambLight_Int = slaveData->ambientLight;
    }
    
    // Build data row with local and slave measurements
    dataRow = String(millis()) + "," +
              String(batteryPercentage) + "," +
              String(ambLight) + "," +
              String(PIRValue) + "," +
              String(movingDist) + "," +
              String(movingEnergy) + "," +
              String(staticDist) + "," +
              String(staticEnergy) + "," +
              String(detectionDist) + "," +
              String(ambLight_Int);
    
    logData(dataRow);
    
    // Log microphone and accelerometer samples if available
    if (slaveData != nullptr) {
      logMicAccelSamples(slaveData->accelX_samples, slaveData->accelY_samples, 
                         slaveData->accelZ_samples, slaveData->microphoneSamples, 
                         slaveData->accelSampleCount);
    }
    
    // Downsample 64x64 RGB frame to 16x16 for BLE transmission (4x4 pixel averaging)
    uint16_t* rgbFrame = getLastRGBFrame();
    if (rgbFrame != nullptr) {
      downsampleRGBFrame(rgbFrame, downsampled16x16);
      Serial.println("[BLE] Downsampled RGB 64x64 -> 16x16 (ready for transmission)");
    }
    
    // Send all notifications (includes downsampled images)
    notifyAll();
    
    timerStream = 0;

    
  }
}
