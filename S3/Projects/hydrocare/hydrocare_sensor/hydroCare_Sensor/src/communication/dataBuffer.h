#ifndef DATA_BUFFER_H
#define DATA_BUFFER_H

#include <Arduino.h>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Data structure containing all sensor readings
typedef struct {
  // IMU Data
  float ax, ay, az;
  
  // Ambient Light
  uint16_t ambLight;
  
  // IR Camera (MLX90641 - 16x12 = 192 pixels, 2 bytes each)
  uint16_t irFrame[192];  // 384 bytes
  
  // RGB Camera (64x64 = 4096 pixels, 2 bytes each - RGB565)
  uint16_t rgbFrame[4096];  // 8192 bytes
  
  // Timestamp
  uint32_t timestamp_ms;
  
  // Status flags
  uint8_t dataValid;  // Bitmask: bit 0=IMU, bit 1=AmbLight, bit 2=IR, bit 3=RGB
} SensorDataFrame;

// Total size: ~9KB per frame

class DataBuffer {
private:
  SensorDataFrame liveBuffer;      // Being written by measurement tasks
  SensorDataFrame stagingBuffer;   // Locked for SPI reads
  SensorDataFrame readyBuffer;     // Safe for master to read
  
  // Use a FreeRTOS mutex to protect buffer access without disabling interrupts
  SemaphoreHandle_t bufferMutex;
  
public:
  DataBuffer();
  
  // Called by measurement tasks: write to live buffer
  void updateIMU(float x, float y, float z);
  void updateAmbLight(uint16_t value);
  void updateIRFrame(uint16_t *frame);
  void updateRGBFrame(uint16_t *frame);
  
  // Called by measurement task: mark frame ready and swap to staging
  void commitFrame();
  
  // Called by SPI handler: lock staging buffer for reading
  void lockForTransfer();
  
  // Called by SPI handler: read data
  const SensorDataFrame* getReadableBuffer() const;
  
  // Get current data validity mask
  uint8_t getDataValidMask() const;
};

#endif
