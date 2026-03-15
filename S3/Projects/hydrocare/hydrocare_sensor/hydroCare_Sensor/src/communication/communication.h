#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include "config/config.h"
#include <freertos/FreeRTOS.h>

// Initialization and communication
void initSPIComm();
void receiveCommand();
void startMeasurementTask();          // Start background measurement collector task
void startHighSpeedSamplerTask();     // Start 1kHz accel+mic sampler task

// Get slave status byte
uint8_t getStatusByte();

#endif