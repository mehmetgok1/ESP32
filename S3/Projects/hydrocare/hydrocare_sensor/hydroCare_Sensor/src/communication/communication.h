#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include "config/config.h"
#include <freertos/FreeRTOS.h>

// Initialization and communication
void initSPIComm();
void receiveCommand();

// Get slave status byte
uint8_t getStatusByte();

#endif