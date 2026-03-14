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
  
  Serial.println("\n=== HYDROCARE MASTER - 256 BYTE DATA PACKETS ===");
  Serial.println("Reading sensor data from slave every 5 seconds");
  Serial.println("SPI Clock: 10 MHz\n");
}

void loop() {
  static uint32_t lastRead = 0;
  
  if (millis() - lastRead > 5000) {
    Serial.printf("\n[%u ms] Reading from slave...\n", millis());
    readSlaveData();
    lastRead = millis();
  }
  
  delay(100);
}

