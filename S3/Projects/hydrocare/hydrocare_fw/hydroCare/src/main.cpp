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
    static uint32_t lastSlaveRead = 0;
    
    String dataRow;
    
    // Measure all local sensors
    measureBatteryLevel();
    measureAmbLight();
    measurePIR();
    measuremmWave();
    
    // Read sensor data from slave every 1 second
    if (millis() - lastSlaveRead >= 1000) {
      Serial.println("\n[Master] Requesting sensor data from slave...");
      readSlaveData();
      lastSlaveRead = millis();
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
              String(detectionDist);
    
    logData(dataRow);
    notifyAll();
    
    timerStream = 0;
  }
}
