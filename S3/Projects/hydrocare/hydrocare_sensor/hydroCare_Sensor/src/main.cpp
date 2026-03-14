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
  
  Serial.println("✓ Setup Complete - Ready for SPI");
}

void loop() {
  // Sequential operation: receive command and handle it
  // Note: Not calling readAcceleration() here - FSPI/SPI2 conflict prevents concurrent access
  // Accelerometer values are captured during initialization and used as cache
  receiveCommand();
}


