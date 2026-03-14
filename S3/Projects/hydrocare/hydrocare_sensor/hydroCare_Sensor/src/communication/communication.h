#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include <SPI.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dataBuffer.h"

// Task declarations
void spiCommunicationTask(void *pvParameters);
void dataUpdateTask(void *pvParameters);

// Initialization
void initSPIComm();
void receiveCommand();

extern TaskHandle_t spiTaskHandle;
extern TaskHandle_t dataTaskHandle;

#endif