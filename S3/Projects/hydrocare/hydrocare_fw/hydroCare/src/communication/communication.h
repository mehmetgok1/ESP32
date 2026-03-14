#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include <SPI.h>

void initSPIComm();
void readSlaveData();
void sendIRLED(bool state);
void sendBrightness(uint8_t brightness);

#endif
