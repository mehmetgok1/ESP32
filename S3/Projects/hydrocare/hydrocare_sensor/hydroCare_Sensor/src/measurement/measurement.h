#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <Arduino.h>
#include "config/config.h"
#include <SPI.h>

extern uint16_t ambLight;
extern float ax,ay,az;

// Individual measurement functions (called by measurement task)
void measureAmbLight();
void measureIRTemp();
void readAcceleration();

// Initialization
void initCamera();
void initIMU();
void initIRTemp();

// I2C/SPI primitives for sensors
void writeRegister(uint8_t reg, uint8_t value);
uint8_t readRegister(uint8_t reg);
void readMultiple(uint8_t startReg, uint8_t *buffer, uint8_t len);

// New: Measurement collection task (runs in FreeRTOS task)
void measurementCollectorTask(void *pvParameters);

// Task handle for sending notifications
extern TaskHandle_t measurementTaskHandle;

#endif