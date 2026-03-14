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
  
  // Start the background measurement task (runs on Core 0)
  startMeasurementTask();
  
  Serial.println("✓ Setup Complete - Ready for SPI with concurrent measurement task");
}

void loop() {
  // SPI Handler: Process commands from master
  // Runs continuously on Core 1, fast and non-blocking
  // When TRIGGER arrives, it signals the measurement task and returns immediately
  // Measurement task runs on Core 0 in parallel
  receiveCommand();
}


