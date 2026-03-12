#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <Arduino.h>
#include "config/config.h"
#include <SPI.h>

extern uint16_t ambLight;
extern float ax,ay,az;

void measureCamera();
void measureAmbLight();
void initCamera();
void writeRegister(uint8_t reg, uint8_t value);
uint8_t readRegister(uint8_t reg);
void readMultiple(uint8_t startReg, uint8_t *buffer, uint8_t len);
void readAcceleration();
void initIMU();
void measureIRTemp();
void initIRTemp();

#endif