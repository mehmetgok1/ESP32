#include "dataBuffer.h"

DataBuffer::DataBuffer() {
  memset(&liveBuffer, 0, sizeof(SensorDataFrame));
  memset(&stagingBuffer, 0, sizeof(SensorDataFrame));
  memset(&readyBuffer, 0, sizeof(SensorDataFrame));
  // Create mutex for buffer protection
  bufferMutex = xSemaphoreCreateMutex();
}

void DataBuffer::updateIMU(float x, float y, float z) {
  if (bufferMutex && xSemaphoreTake(bufferMutex, portMAX_DELAY) == pdTRUE) {
    liveBuffer.ax = x;
    liveBuffer.ay = y;
    liveBuffer.az = z;
    liveBuffer.dataValid |= 0x01;  // Mark IMU as valid
    xSemaphoreGive(bufferMutex);
  }
}

void DataBuffer::updateAmbLight(uint16_t value) {
  if (bufferMutex && xSemaphoreTake(bufferMutex, portMAX_DELAY) == pdTRUE) {
    liveBuffer.ambLight = value;
    liveBuffer.dataValid |= 0x02;  // Mark ambient light as valid
    xSemaphoreGive(bufferMutex);
  }
}

void DataBuffer::updateIRFrame(uint16_t *frame) {
  if (bufferMutex && xSemaphoreTake(bufferMutex, portMAX_DELAY) == pdTRUE) {
    if (frame) {
      memcpy(liveBuffer.irFrame, frame, sizeof(liveBuffer.irFrame));
      liveBuffer.dataValid |= 0x04;  // Mark IR as valid
    }
    xSemaphoreGive(bufferMutex);
  }
}

void DataBuffer::updateRGBFrame(uint16_t *frame) {
  if (bufferMutex && xSemaphoreTake(bufferMutex, portMAX_DELAY) == pdTRUE) {
    if (frame) {
      memcpy(liveBuffer.rgbFrame, frame, sizeof(liveBuffer.rgbFrame));
      liveBuffer.dataValid |= 0x08;  // Mark RGB as valid
    }
    xSemaphoreGive(bufferMutex);
  }
}

void DataBuffer::commitFrame() {
  if (bufferMutex && xSemaphoreTake(bufferMutex, portMAX_DELAY) == pdTRUE) {
    liveBuffer.timestamp_ms = millis();
    // Atomic swap: live → staging
    memcpy(&stagingBuffer, &liveBuffer, sizeof(SensorDataFrame));
    // Clear live buffer for next cycle
    memset(&liveBuffer, 0, sizeof(SensorDataFrame));
    xSemaphoreGive(bufferMutex);
  }
}

void DataBuffer::lockForTransfer() {
  if (bufferMutex && xSemaphoreTake(bufferMutex, portMAX_DELAY) == pdTRUE) {
    // Swap staging → ready (freezes data for transfer)
    memcpy(&readyBuffer, &stagingBuffer, sizeof(SensorDataFrame));
    xSemaphoreGive(bufferMutex);
  }
}

const SensorDataFrame* DataBuffer::getReadableBuffer() const {
  return &readyBuffer;
}

uint8_t DataBuffer::getDataValidMask() const {
  return readyBuffer.dataValid;
}
