#include "protocolState.h"

ProtocolStateMachine::ProtocolStateMachine() 
  : currentState(STATE_IDLE), measurementStartTime(0), measurementTimeout_ms(1200) {
}

bool ProtocolStateMachine::triggerMeasurement() {
  taskENTER_CRITICAL(&stateLock);
  bool triggered = false;
  if (currentState == STATE_IDLE) {
    currentState = STATE_MEASURING;
    measurementStartTime = millis();
    triggered = true;
  }
  taskEXIT_CRITICAL(&stateLock);
  if (triggered) {
    Serial.println("[Slave] State: IDLE → MEASURING");
  }
  return triggered;
}

void ProtocolStateMachine::completeMeasurement() {
  uint32_t elapsed = 0;
  taskENTER_CRITICAL(&stateLock);
  if (currentState == STATE_MEASURING) {
    currentState = STATE_READY;
    elapsed = millis() - measurementStartTime;
  }
  taskEXIT_CRITICAL(&stateLock);
  // Print OUTSIDE critical section to avoid blocking
  if (elapsed > 0) {
    Serial.printf("[Slave] State: MEASURING → READY (took %lu ms)\n", elapsed);
  }
}

void ProtocolStateMachine::beginTransfer() {
  taskENTER_CRITICAL(&stateLock);
  bool transferred = false;
  if (currentState == STATE_READY) {
    currentState = STATE_TRANSFERRING;
    transferred = true;
  }
  taskEXIT_CRITICAL(&stateLock);
  if (transferred) {
    Serial.println("[Slave] State: READY → TRANSFERRING");
  }
}

void ProtocolStateMachine::completeTransfer() {
  taskENTER_CRITICAL(&stateLock);
  bool completed = false;
  uint8_t oldState = currentState;
  
  // Transition to IDLE from EITHER TRANSFERRING or READY state
  // (handles case where master sends TRANSFER_COMPLETE without explicit LOCK_BUFFERS)
  if (currentState == STATE_TRANSFERRING || currentState == STATE_READY) {
    currentState = STATE_IDLE;
    completed = true;
  }
  taskEXIT_CRITICAL(&stateLock);
  
  if (completed) {
    Serial.printf("[Slave] State: %d → IDLE\n", oldState);
  }
}

void ProtocolStateMachine::setError() {
  taskENTER_CRITICAL(&stateLock);
  currentState = STATE_IDLE;  // Reset on error
  taskEXIT_CRITICAL(&stateLock);
  Serial.println("[Slave] ERROR: Reset to IDLE");
}

ProtocolState ProtocolStateMachine::getState() const {
  return currentState;
}

bool ProtocolStateMachine::isMeasuring() const {
  return currentState == STATE_MEASURING;
}

bool ProtocolStateMachine::isReady() const {
  return currentState == STATE_READY;
}

bool ProtocolStateMachine::isTransferring() const {
  return currentState == STATE_TRANSFERRING;
}

bool ProtocolStateMachine::isIdle() const {
  return currentState == STATE_IDLE;
}

bool ProtocolStateMachine::isMeasurementTimeout() const {
  if (!isMeasuring()) return false;
  return (millis() - measurementStartTime) > measurementTimeout_ms;
}

uint8_t ProtocolStateMachine::getStatusByte() const {
  uint8_t status = 0x00;
  if (isMeasuring()) status |= 0x01;    // Bit 0: MEASURING
  if (isReady()) status |= 0x02;        // Bit 1: READY
  if (isTransferring()) status |= 0x04; // Bit 2: TRANSFERRING
  return status;
}
