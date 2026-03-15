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
  
  // Start background tasks
  startHighSpeedSamplerTask();  // 1kHz accel + mic sampling on Core 0 (Priority 3)
  startMeasurementTask();       // Measurement collector on Core 0 (Priority 2)
  
  Serial.println("✓ Setup Complete - Ready for SPI with 1kHz sampler + concurrent measurements");
}

void loop() {
  // SPI Handler: Process commands from master
  // Runs continuously on Core 1, fast and non-blocking
  // When TRIGGER arrives, it signals the measurement task and returns immediately
  // High-speed sampler runs independently on Core 0 every 1ms
  // Measurement task runs on Core 0 when triggered
  receiveCommand();
}


