#include <Arduino.h>
#include "config/config.h"
#include "measurement/measurement.h"
#include "leds/leds.h"
#include "communication/communication.h"

void setup() {

  initPins();
  initPeripherals();
  initCamera();
  powerLEDInit();
  initIMU();
  initIRTemp();
  initSPIComm();
}

void loop() {
  //measureCamera(); 
  measureAmbLight();
  receiveCommand();
}
