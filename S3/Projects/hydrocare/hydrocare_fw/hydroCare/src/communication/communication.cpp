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
  uint16_t reserved;              // Future use
  uint8_t status;                 // Status flags
  uint8_t crc;                    // Checksum
  uint8_t padding[216];           // Padding to 256 bytes
} SensorDataPacket;

#define SPI_CLOCK_HZ     10000000  // 10 MHz
#define SPI_BUFFER_SIZE  256        // 256 bytes per transaction

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
  
  Serial.println("[Master] SPI Init OK - 10 MHz, 256-byte data packets");
}

void readSlaveData() {
  // Pull CS low and read 256-byte sensor packet
  spi.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(SPI_CS, LOW);
  delayMicroseconds(50);
  
  // Read 256 bytes
  for (uint16_t i = 0; i < SPI_BUFFER_SIZE; i++) {
    spiRxBuffer[i] = spi.transfer(0x00);
  }
  
  delayMicroseconds(20);
  digitalWrite(SPI_CS, HIGH);
  spi.endTransaction();
  
  // Parse packet
  SensorDataPacket *packet = (SensorDataPacket*)spiRxBuffer;
  
  // Verify CRC (calculate only on data fields, not padding)
  uint16_t crcSize = offsetof(SensorDataPacket, crc);
  uint8_t crc = 0;
  for (uint16_t i = 0; i < crcSize; i++) {
    crc ^= spiRxBuffer[i];
  }
  
  if (crc != packet->crc) {
    Serial.printf("[Master RX] CRC Error: expected 0x%02X, got 0x%02X\n", packet->crc, crc);
    return;
  }
  
  // Display packet data
  Serial.printf("[Master RX] #%u [T:%.1f°C H:%.1f%% L:%u] AX:%d AY:%d AZ:%d GX:%d GY:%d GZ:%d [OK]\n",
    packet->sequence,
    packet->temperature,
    packet->humidity,
    packet->ambientLight,
    packet->accelX, packet->accelY, packet->accelZ,
    packet->gyroX, packet->gyroY, packet->gyroZ
  );
}

// Stub functions for compatibility with BLE module
void sendIRLED(bool state) {
  (void)state;  // Unused in test mode
}

void sendBrightness(uint8_t brightness) {
  (void)brightness;  // Unused in test mode
}

