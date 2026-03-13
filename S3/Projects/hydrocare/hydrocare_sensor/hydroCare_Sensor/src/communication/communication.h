#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include <SPI.h>
#include "dataBuffer.h"
#include "protocolState.h"

// Protocol command opcodes
#define CMD_POLL_STATUS          0x00  // Master: Poll status only (no state change)
#define CMD_TRIGGER_MEASUREMENT  0x01  // Master: start collecting data
#define CMD_LOCK_BUFFERS         0x02  // Master: freeze buffers for reading
#define CMD_TRANSFER_COMPLETE    0x03  // Master: Done reading, return to IDLE
#define CMD_READ_DATA            0x04  // Master: Read next data chunk (keep slave in TRANSFERRING)
#define CMD_ACK                  0xAA
#define CMD_ERROR                0xFF

// Legacy commands (still supported for compatibility)
#define LED_CONTROL_CMD  0x10
#define IR_CONTROL_CMD   0x11

// SPI Protocol structure
typedef struct {
  uint8_t opcode;    // Command from master
  uint8_t status;    // Status from slave (MEASURING, READY, etc.)
  uint8_t dataLen;   // How many bytes of payload follow
} SPIProtocolHeader;

void initSPIComm();
void receiveCommand();

// New protocol handlers
void handleMasterCommand(uint8_t cmd);
void sendStatusAndData(uint8_t *txBuffer, uint16_t bufferSize);

// Get global instances
extern DataBuffer *g_dataBuffer;
extern ProtocolStateMachine *g_protocolState;
extern TaskHandle_t measurementTaskHandle;

#endif