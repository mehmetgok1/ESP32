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
  initBME688(); // Ensure this is after initIMU so the SPI bus is ready
  initSPIComm();
  
  // Start background tasks
  startHighSpeedSamplerTask();  // 1kHz accel + mic sampling on Core 0 (Priority 3)
  startMeasurementTask();       // Measurement collector on Core 0 (Priority 2)
  
  Serial.println("✓ Setup Complete - Ready for SPI with 1kHz sampler + concurrent measurements");
}

void loop() {
  receiveCommand();
}
