#ifndef DATA_BUFFER_H
#define DATA_BUFFER_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Extended sensor data packet with camera frames
typedef struct {
  uint16_t sequence;              // Packet sequence number (2 bytes)
  uint16_t ambientLight;          // Ambient light value (2 bytes)
  float temperature;              // Temperature in °C (4 bytes)
  float humidity;                 // Humidity % (4 bytes)
  int16_t accelX, accelY, accelZ; // IMU accel (6 bytes)
  int16_t gyroX, gyroY, gyroZ;    // IMU gyro (6 bytes)
  uint32_t timestamp_ms;          // System uptime (4 bytes)
  uint8_t status;                 // Status flags (1 byte)
  uint8_t crc;                    // Checksum (1 byte)
  uint8_t reserved[2];            // Alignment (2 bytes)
  
  // Camera data frames
  uint16_t rgbFrame[4096];        // RGB565 64x64 (8192 bytes)
  uint16_t irFrame[192];          // IR thermal 16x12 (384 bytes)
} SensorDataPacket;

// Total size: 32 + 8192 + 384 = 8608 bytes (round to 8704)

class DataBuffer {
private:
  SensorDataPacket currentData;    // Current sensor readings
  SensorDataPacket txData;         // Buffer to send via SPI
  uint16_t sequenceNumber;         // Increments with each packet
  SemaphoreHandle_t dataMutex;    // Protect concurrent access
  
public:
  DataBuffer();
  void init();
  
  // Called by measurement tasks: update sensor values
  void updateAmbLight(uint16_t value);
  void updateTemperature(float value);
  void updateHumidity(float value);
  void updateIMU(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);
  void updateRGBFrame(uint16_t *frame);    // 64x64 RGB565
  void updateIRFrame(uint16_t *frame);     // 16x12 thermal
  void updateTimestamp();
  
  // Called by SPI task: prepare packet for transmission
  void prepareTxBuffer(uint8_t *buffer, uint16_t bufferSize);
  
  // Get current sensor data
  SensorDataPacket* getCurrentData();
  
private:
  uint8_t calculateCRC(const uint8_t *data, uint16_t len);
};

extern DataBuffer g_dataBuffer;

#endif
