#ifndef DATA_BUFFER_H
#define DATA_BUFFER_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Simple sensor data packet (fits perfectly in 256-byte SPI frame)
typedef struct {
  uint16_t sequence;              // Packet sequence number (2 bytes)
  uint16_t ambientLight;          // Ambient light value (2 bytes)
  float temperature;              // Temperature in °C (4 bytes)
  float humidity;                 // Humidity % (4 bytes)
  int16_t accelX, accelY, accelZ; // IMU accel (6 bytes)
  int16_t gyroX, gyroY, gyroZ;    // IMU gyro (6 bytes)
  uint32_t timestamp_ms;          // System uptime (4 bytes)
  uint16_t reserved;              // Future use (2 bytes)
  uint8_t status;                 // Status flags (1 byte)
  uint8_t crc;                    // Checksum (1 byte)
  uint8_t padding[216];           // Padding to 256 bytes
} SensorDataPacket;

// Total size: exactly 256 bytes

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
