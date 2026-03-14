#include "dataBuffer.h"
#include <Arduino.h>
#include <cstring>
#include <cstddef>

DataBuffer g_dataBuffer;

DataBuffer::DataBuffer() : sequenceNumber(0) {
  memset(&currentData, 0, sizeof(SensorDataPacket));
  memset(&txData, 0, sizeof(SensorDataPacket));
  dataMutex = xSemaphoreCreateMutex();
}

void DataBuffer::init() {
  if (dataMutex == NULL) {
    dataMutex = xSemaphoreCreateMutex();
  }
  sequenceNumber = 0;
  Serial.println("[DataBuffer] Initialized");
}

void DataBuffer::updateAmbLight(uint16_t value) {
  if (dataMutex && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    currentData.ambientLight = value;
    xSemaphoreGive(dataMutex);
  }
}

void DataBuffer::updateTemperature(float value) {
  if (dataMutex && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    currentData.temperature = value;
    xSemaphoreGive(dataMutex);
  }
}

void DataBuffer::updateHumidity(float value) {
  if (dataMutex && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    currentData.humidity = value;
    xSemaphoreGive(dataMutex);
  }
}

void DataBuffer::updateIMU(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz) {
  if (dataMutex && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    currentData.accelX = ax;
    currentData.accelY = ay;
    currentData.accelZ = az;
    currentData.gyroX = gx;
    currentData.gyroY = gy;
    currentData.gyroZ = gz;
    xSemaphoreGive(dataMutex);
  }
}

void DataBuffer::updateTimestamp() {
  if (dataMutex && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    currentData.timestamp_ms = millis();
    xSemaphoreGive(dataMutex);
  }
}

void DataBuffer::prepareTxBuffer(uint8_t *buffer, uint16_t bufferSize) {
  if (bufferSize < sizeof(SensorDataPacket)) {
    return;
  }

  if (dataMutex && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    // Copy current data to tx buffer
    currentData.sequence = sequenceNumber++;
    currentData.status = 0x01;  // Mark as valid
    
    // Zero out padding before CRC calculation
    memset(currentData.padding, 0, sizeof(currentData.padding));
    
    // Calculate CRC only on data fields (first 32 bytes, excluding CRC and padding)
    uint16_t crcSize = offsetof(SensorDataPacket, crc);
    currentData.crc = calculateCRC((uint8_t*)&currentData, crcSize);
    
    // Copy to buffer
    memcpy(buffer, &currentData, sizeof(SensorDataPacket));
    
    xSemaphoreGive(dataMutex);
  }
}

SensorDataPacket* DataBuffer::getCurrentData() {
  return &currentData;
}

uint8_t DataBuffer::calculateCRC(const uint8_t *data, uint16_t len) {
  uint8_t crc = 0;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= data[i];
  }
  return crc;
}
