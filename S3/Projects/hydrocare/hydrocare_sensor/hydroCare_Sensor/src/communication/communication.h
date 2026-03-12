#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include <SPI.h>

#define LED_CONTROL_CMD  0x01
#define IR_CONTROL_CMD   0x02
#define ACK_OK           0xAA
#define ACK_UNKNOWN      0xFF

void initSPIComm();
void receiveCommand();

#endif