#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include <SPI.h>

void initSPIComm();
void sendBrightness(uint8_t brightness);
void sendIRLED(bool state);
void readAmbientLight_Int();

#endif // COMMUNICATION_H