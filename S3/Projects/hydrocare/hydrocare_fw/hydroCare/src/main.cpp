#include <Arduino.h>
#include "config/config.h"
#include "ui/ui.h"
#include "measurement/measurement.h"
#include "ble/ble.h"
#include "timer/timer.h"
#include "memory/memory.h"
#include "ota/ota.h"
#include "communication/communication.h"

void setup() {

  initPins();
  initPeripherals();
  initLed();
  initSD();
  uiInit();
  initmmWave();
  initBLE();
  initTimer();
  setTimer();  // TEST: Enable timer immediately for sensor testing (normally enabled by BLE)
  initSPIComm();
  deviceStatus = 1;  // TEST: Enable device status for sensor testing (normally set by BLE)
}

void loop() {
  static uint32_t lastLoopDebug = 0;

  checkUSB();
  
  if(otaUpdateAvailable){
    
    uiOTAStarted();
    connectToWiFi();
    performOTAUpdate();
  }
  
  // Debug: Print loop status every 5 seconds
  if (millis() - lastLoopDebug > 5000) {
    Serial.printf("[Loop] Status - deviceConnected:%d, timerStream:%d, deviceStatus:%d\n", 
                  deviceConnected, timerStream, deviceStatus);
    lastLoopDebug = millis();
  }
  
  // TEST: Allow sensor code to run without BLE for debugging
  if (timerStream == 1 && deviceStatus == 1) {

    String dataRow;
    
    // Local measurements (master-side)
    measureBatteryLevel();
    measureAmbLight();
    measurePIR();
    measuremmWave();
    
    // NEW: Collect data from sensor slave via SPI
    // ============================================
    Serial.println("[Master] === SENSOR SYNC CYCLE ===");
    
    // Step 1: Tell slave to collect all sensor data
    if (triggerMeasurementCycle()) {
      
      // Step 2: Wait for slave to finish (typically ~1000ms for all sensors)
      if (waitForSlaveReady(2000)) {  // 2 second timeout
        
        // Step 3: Burst-read all sensor data from slave
        SlaveData sensorData = {};
        if (readSensorDataBurst(&sensorData)) {
          
          // Extract slave sensor readings
          ambLight_Int = sensorData.ambLight;
          
          // TODO: Also use camera frames, IR data, etc.
          Serial.printf("[Master] ✓ Slave IMU: X=%.3f Y=%.3f Z=%.3f\n",
                        sensorData.ax, sensorData.ay, sensorData.az);
          Serial.printf("[Master] ✓ Slave Ambient: %d lux\n", sensorData.ambLight);
          Serial.printf("[Master] ✓ Data valid mask: 0x%02X\n", sensorData.dataValidMask);
          
        } else {
          Serial.println("[Master] ✗ Failed to read sensor data");
        }
      } else {
        Serial.println("[Master] ✗ Slave did not become ready");
      }
    } else {
      Serial.println("[Master] ✗ 0Failed to trigger measurement");
    }

    // Build data log row with both master and slave sensor data
    dataRow = String(millis()) + "," +
              String(batteryPercentage) + "," +
              String(ambLight) + "," +
              String(PIRValue) + "," +
              String(movingDist) + "," +
              String(movingEnergy) + "," +
              String(staticDist) + "," +
              String(staticEnergy) + "," +
              String(detectionDist) + "," +
              String(ambLight_Int);  // Now from slave instead of polling

    logData(dataRow); // This appends to the file and closes it
    notifyAll();

    timerStream = 0;
  }
}