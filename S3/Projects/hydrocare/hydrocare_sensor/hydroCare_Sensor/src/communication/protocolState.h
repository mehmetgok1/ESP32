#ifndef PROTOCOL_STATE_H
#define PROTOCOL_STATE_H

#include <Arduino.h>

typedef enum {
  STATE_IDLE = 0,         // Waiting for master command
  STATE_MEASURING = 1,    // Actively collecting sensor data
  STATE_READY = 2,        // Data collected and locked, ready for burst read
  STATE_TRANSFERRING = 3  // Master is reading data
} ProtocolState;

typedef enum {
  CMD_NONE = 0x00,
  CMD_TRIGGER_MEASUREMENT = 0x01,  // Master: start collecting data
  CMD_LOCK_BUFFERS = 0x02,         // Master: freeze buffers for reading
  CMD_ACK = 0xAA,                  // Slave: acknowledge command
  CMD_ERROR = 0xFF                 // Slave: error occurred
} ProtocolCommand;

class ProtocolStateMachine {
private:
  ProtocolState currentState;
  uint32_t measurementStartTime;
  uint32_t measurementTimeout_ms;  // How long to wait for measurement (default 1000ms)
  
  portMUX_TYPE stateLock = portMUX_INITIALIZER_UNLOCKED;
  
public:
  ProtocolStateMachine();
  
  // State transitions
  bool triggerMeasurement();      // IDLE → MEASURING (returns true if state changed)
  void completeMeasurement();    // MEASURING → READY (called by SW timer or task)
  void beginTransfer();          // READY → TRANSFERRING
  void completeTransfer();       // TRANSFERRING → IDLE
  void setError();               // Any → Error state (optional)
  
  // State queries
  ProtocolState getState() const;
  bool isMeasuring() const;
  bool isReady() const;
  bool isTransferring() const;
  bool isIdle() const;
  
  // Timeout check: returns true if measurement has taken too long
  bool isMeasurementTimeout() const;
  
  // Status byte to send to master
  uint8_t getStatusByte() const;
};

#endif
