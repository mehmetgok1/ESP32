#include "communication/communication.h"
#include <Arduino.h>
#include <SPI.h>
#include "config/config.h"
#include <cstring>
#include <cstddef>

// Protocol commands (match slave side)
#define CMD_NONE 0x00
#define CMD_TRIGGER_MEASUREMENT 0x01
#define CMD_LOCK_BUFFERS 0x02
#define CMD_TRANSFER_DATA 0x03
#define CMD_ACK 0xAA
#define CMD_ERROR 0xFF

// Slave status bits
#define STATUS_MEASURING 0x01
#define STATUS_MEASURED 0x02
#define STATUS_LOCKED 0x04
#define STATUS_READY_TRANSFER 0x08

// Sensor data packet structure (matches slave)
#pragma pack(1)
typedef struct {
  uint16_t sequence;              // Packet sequence number
  uint16_t ambientLight;          // Ambient light value
  float temperature;              // Temperature in °C
  float humidity;                 // Humidity %
  int16_t accelX, accelY, accelZ; // IMU accel
  int16_t gyroX, gyroY, gyroZ;    // IMU gyro
  uint32_t timestamp_ms;          // System uptime
  uint8_t status;                 // Status flags
  
  // Camera data frames
  uint16_t rgbFrame[4096];        // RGB565 64x64 (8192 bytes)
  uint16_t irFrame[192];          // IR thermal 16x12 (384 bytes)
} SensorDataPacket;
#pragma pack()

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

// Send a command to slave and get immediate response
uint8_t sendCommand(uint8_t cmd) {
  memset(spiTxBuffer, 0, 10);
  spiTxBuffer[0] = cmd;
  
  spi.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(SPI_CS, LOW);
  delayMicroseconds(50);
  
  uint8_t response = spi.transfer(cmd);
  
  // Complete 10-byte handshake
  for (int i = 1; i < 10; i++) {
    spi.transfer(0x00);
  }
  
  delayMicroseconds(20);
  digitalWrite(SPI_CS, HIGH);
  spi.endTransaction();
  
  return response;
}

// Poll slave status
uint8_t getSlaveStatus() {
  return sendCommand(CMD_NONE);
}

// Poll with timeout - returns true when status flag is set
bool pollStatus(uint8_t statusFlag, uint32_t timeoutMs) {
  uint32_t startTime = millis();
  uint8_t status = 0;
  
  while (millis() - startTime < timeoutMs) {
    status = getSlaveStatus();
    if (status & statusFlag) {
      return true;
    }
    delay(10);
  }
  
  return false;
}

// Read sensor data from slave (full 8704-byte transfer)
void readSensorData() {
  memset(spiRxBuffer, 0, SPI_BUFFER_SIZE);
  
  spi.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(SPI_CS, LOW);
  delayMicroseconds(50);
  
  // Send TRANSFER_DATA command in first byte and immediately continue reading
  // Slave sees CMD_TRANSFER_DATA and starts sending data on-the-go
  for (uint16_t i = 0; i < SPI_BUFFER_SIZE; i++) {
    uint8_t txByte = (i == 0) ? CMD_TRANSFER_DATA : 0x00;  // Send command only at byte 0
    spiRxBuffer[i] = spi.transfer(txByte);
  }
  
  delayMicroseconds(20);
  digitalWrite(SPI_CS, HIGH);
  spi.endTransaction();
  
  // Parse: Byte 0 is status, sensor data starts at byte 1
  uint8_t slaveStatus = spiRxBuffer[0];
  Serial.printf("[Master RX] Slave status: 0x%02X\n", slaveStatus);
  
  // Parse packet from byte 1 onwards
  SensorDataPacket *packet = (SensorDataPacket*)(&spiRxBuffer[1]);
  
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

// Main measurement cycle with state machine
void readSlaveData() {
  Serial.println("[Master] === Starting measurement cycle ===");
  
  // Step 1: Send TRIGGER command to slave
  Serial.println("[Master] Step 1: Sending TRIGGER command...");
  uint8_t response = sendCommand(CMD_TRIGGER_MEASUREMENT);
  Serial.printf("[Master] Slave response: 0x%02X\n", response);
  delay(100);
  
  // Step 2: Poll until STATUS_MEASURED
  Serial.println("[Master] Step 2: Waiting for measurement to complete...");
  if (!pollStatus(STATUS_MEASURED, 10000)) {
    Serial.println("[Master] ERROR: Timeout waiting for STATUS_MEASURED");
    return;
  }
  Serial.println("[Master] Slave measurement complete!");
  
  // Step 3: Send LOCK_BUFFERS command
  Serial.println("[Master] Step 3: Sending LOCK_BUFFERS command...");
  response = sendCommand(CMD_LOCK_BUFFERS);
  Serial.printf("[Master] Slave response: 0x%02X\n", response);
  delay(50);
  
  // Step 4: Poll until STATUS_LOCKED
  Serial.println("[Master] Step 4: Waiting for buffers to be locked...");
  if (!pollStatus(STATUS_LOCKED, 10000)) {
    Serial.println("[Master] ERROR: Timeout waiting for STATUS_LOCKED");
    return;
  }
  Serial.println("[Master] Buffers locked!");
  
  // Step 5: Send TRANSFER_DATA command and receive data continuously
  Serial.println("[Master] Step 5: Sending TRANSFER_DATA command and streaming data...");
  
  // readSensorData() sends CMD_TRANSFER_DATA in byte 0 and keeps CS LOW
  // for entire 8704-byte transfer. Slave sees command and responds on-the-go
  readSensorData();
  
  Serial.println("[Master] === Measurement cycle complete ===");
}

// Stub functions for compatibility with BLE module
void sendIRLED(bool state) {
  (void)state;  // Unused in test mode
}

void sendBrightness(uint8_t brightness) {
  (void)brightness;  // Unused in test mode
}

