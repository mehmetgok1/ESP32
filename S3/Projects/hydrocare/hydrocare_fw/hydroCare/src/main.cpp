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
}

void loop() {

  checkUSB();
  /*
  for (int b = 0; b <= 100; b += 5) {
      sendBrightness((uint8_t)b);
      delay(5);
    }
    for (int b = 100; b >= 0; b -= 5) {
      sendBrightness((uint8_t)b);
      delay(5);
    }  
  */
  if(otaUpdateAvailable){
    
    uiOTAStarted();
    connectToWiFi();
    performOTAUpdate();
  }
  if (deviceConnected && timerStream == 1 && deviceStatus == 1) {

    String dataRow;
    
    measureBatteryLevel();
    measureAmbLight();
    measurePIR();
    measuremmWave();
    readAmbientLight_Int();

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

    logData(dataRow); // This appends to the file and closes it
    notifyAll();

    timerStream = 0;
  }
}