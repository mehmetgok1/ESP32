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

void setup() {
  initPins();
  initPeripherals();
  initLed();
  initSD();
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
  
  // Process any pending BLE operations (like sending RGB frame)
  processBLETasks();
  
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
    notifyAll();
    
    timerStream = 0;

    
  }
}
