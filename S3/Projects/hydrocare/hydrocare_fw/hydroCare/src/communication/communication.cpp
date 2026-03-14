#include "communication/communication.h"
#include <Arduino.h>
#include <SPI.h>
#include "config/config.h"
#include <cstring>
#include <cstddef>

// Sensor data packet structure (matches slave)
typedef struct {
  uint16_t sequence;              // Packet sequence number
  uint16_t ambientLight;          // Ambient light value
  float temperature;              // Temperature in °C
  float humidity;                 // Humidity %
  int16_t accelX, accelY, accelZ; // IMU accel
  int16_t gyroX, gyroY, gyroZ;    // IMU gyro
  uint32_t timestamp_ms;          // System uptime
  uint8_t status;                 // Status flags
  uint8_t crc;                    // Checksum
  uint8_t reserved[2];            // Alignment
  
  // Camera data frames
  uint16_t rgbFrame[4096];        // RGB565 64x64 (8192 bytes)
  uint16_t irFrame[192];          // IR thermal 16x12 (384 bytes)
} SensorDataPacket;

#define SPI_CLOCK_HZ     10000000  // 10 MHz
#define SPI_BUFFER_SIZE  8704       // 8704 bytes per transaction

SPIClass spi(HSPI);
static uint8_t *spiTxBuffer = NULL;
static uint8_t *spiRxBuffer = NULL;

void initSPIComm() {
  spi.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);
  
  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);
  
  spiTxBuffer = (uint8_t*) heap_caps_malloc(SPI_BUFFER_SIZE, MALLOC_CAP_DMA);
  spiRxBuffer = (uint8_t*) heap_caps_malloc(SPI_BUFFER_SIZE, MALLOC_CAP_DMA);
  
  if (!spiTxBuffer || !spiRxBuffer) {
    Serial.println("[Master] DMA allocation failed!");
    while(1) delay(1000);
  }
  
  memset(spiTxBuffer, 0, SPI_BUFFER_SIZE);
  memset(spiRxBuffer, 0, SPI_BUFFER_SIZE);
  
  Serial.println("[Master] SPI Init OK - 10 MHz, 8704-byte data packets");
}

void readSlaveData() {
  // Pull CS low and read 8704-byte packet
  spi.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(SPI_CS, LOW);
  delayMicroseconds(50);
  
  // Read 8704 bytes
  for (uint16_t i = 0; i < SPI_BUFFER_SIZE; i++) {
    spiRxBuffer[i] = spi.transfer(0x00);
  }
  
  delayMicroseconds(20);
  digitalWrite(SPI_CS, HIGH);
  spi.endTransaction();
  
  // Parse packet
  SensorDataPacket *packet = (SensorDataPacket*)spiRxBuffer;
  
  // Verify CRC (calculate only on data fields BEFORE crc byte)
  uint16_t crcSize = offsetof(SensorDataPacket, crc);
  uint8_t crc = 0;
  for (uint16_t i = 0; i < crcSize; i++) {
    crc ^= spiRxBuffer[i];
  }
  
  if (crc != packet->crc) {
    Serial.printf("[Master RX] CRC Error: expected 0x%02X, got 0x%02X\n", packet->crc, crc);
    return;
  }
  
  // Calculate frame data statistics
  uint16_t rgbMin = 65535, rgbMax = 0;
  for (int i = 0; i < 4096; i++) {
    if (packet->rgbFrame[i] < rgbMin) rgbMin = packet->rgbFrame[i];
    if (packet->rgbFrame[i] > rgbMax) rgbMax = packet->rgbFrame[i];
  }
  
  uint16_t irMin = 65535, irMax = 0;
  for (int i = 0; i < 192; i++) {
    if (packet->irFrame[i] < irMin) irMin = packet->irFrame[i];
    if (packet->irFrame[i] > irMax) irMax = packet->irFrame[i];
  }
  
  // Display packet data
  Serial.printf("[Master RX] #%u [T:%.1f°C H:%.1f%% L:%u] AX:%d AY:%d AZ:%d RGB[%u-%u] IR[%u-%u] [OK]\n", 
    packet->sequence,
    packet->temperature,
    packet->humidity,
    packet->ambientLight,
    packet->accelX, packet->accelY, packet->accelZ,
    rgbMin, rgbMax, irMin, irMax
  );
}

// Stub functions for compatibility with BLE module
void sendIRLED(bool state) {
  (void)state;  // Unused in test mode
}

void sendBrightness(uint8_t brightness) {
  (void)brightness;  // Unused in test mode
}

