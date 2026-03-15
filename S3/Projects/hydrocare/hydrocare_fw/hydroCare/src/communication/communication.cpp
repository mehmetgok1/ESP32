#include "communication/communication.h"
#include <Arduino.h>
#include <SPI.h>
#include "config/config.h"
#include <cstring>
#include <cstddef>

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
  
  Serial.println("[Master] SPI Init OK - Protocol-based 10 MHz, 20KB DMA buffers");
}

// ==================== PROTOCOL IMPLEMENTATION ====================
// Protocol: Master-Slave SPI with address-based R/W
// 
// *** CRITICAL: SPI Protocol Sequence ***
// Master MUST send first. Slave ONLY responds to what master sends.
// 
// STEP 1: Master sends Command Byte [R/W bit (MSB)] [Address (7 bits)]
//         => Slave receives command, prepares response
//         => Master receives garbage/echo (first byte is MEANINGLESS - discard it)
//
// STEP 2: Master sends dummy bytes (0x00) to clock in slave response
//         => Slave sends actual response/data
//         => Master receives real data
//
// Command Byte format:
//   Bit 7: R/W (1 = Read, 0 = Write)
//   Bits 6-0: Address (0-127)
//
// For WRITE operations:
//   Byte 0: [R/W=0 | Address]
//   Byte 1: Data byte
//
// For READ operations:
//   Byte 0: [R/W=1 | Address]  (master sends, slave receives)
//   Byte 1+: Master sends 0x00, slave responds with data

// Single byte READ from slave register/memory address
// 
// Protocol sequence:
// 1. Master sends: [R/W=1 | Address]  <- Slave receives and prepares response
//    Master receives: GARBAGE (discard)
// 2. Master sends: 0x00               <- Just clocking to read slave's data
//    Master receives: actual data from slave
//
// Returns: Data byte from slave
uint8_t spiRead(uint8_t address) {
  if (address > 0x7F) {
    Serial.printf("[SPI Error] Read address 0x%02X exceeds 7-bit space\n", address);
    return 0xFF;
  }
  
  uint8_t cmdByte = PROTO_CMD_READ | (address & PROTO_ADDR_MASK);
  
  spi.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(SPI_CS, LOW);
  delayMicroseconds(50);
  
  // Master sends command, discard garbage response
  spi.transfer(cmdByte);
  
  // Master sends dummy byte (0x00), slave sends actual data
  uint8_t readData = spi.transfer(0x00);
  
  delayMicroseconds(20);
  digitalWrite(SPI_CS, HIGH);
  spi.endTransaction();
  
  return readData;
}

// Single byte WRITE to slave register/memory address
//
// Protocol sequence:
// 1. Master sends: [R/W=0 | Address]  <- Slave receives command
//    Master receives: GARBAGE (discard)
// 2. Master sends: Data byte          <- Slave receives and writes data
//    Master receives: GARBAGE (discard)
//
void spiWrite(uint8_t address, uint8_t data) {
  if (address > 0x7F) {
    Serial.printf("[SPI Error] Write address 0x%02X exceeds 7-bit space\n", address);
    return;
  }
  
  uint8_t cmdByte = PROTO_CMD_WRITE | (address & PROTO_ADDR_MASK);
  
  spi.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(SPI_CS, LOW);
  delayMicroseconds(50);
  
  // Master sends write command+address, discard garbage
  spi.transfer(cmdByte);
  
  // Master sends data, slave receives and stores it
  spi.transfer(data);
  
  delayMicroseconds(20);
  digitalWrite(SPI_CS, HIGH);
  spi.endTransaction();
}

// Bulk READ from slave (for large transfers like sensor data)
//
// Protocol sequence:
// 1. Master sends: [R/W=1 | Address]  <- Slave receives command, prepares data
//    Master receives: GARBAGE (discard)
// 2. Master sends: dummy bytes (0x00) <- Slave sends data continuously
//    Master receives: actual data payload
//
// Special case: address=0 (ADDR_SENSOR_DATA) triggers large sensor packet read (>20KB)
// Slave handles fragmented transfers - can call multiple times for mega-transfers
void spiReadBulk(uint8_t address, uint8_t *buffer, uint16_t numBytes) {
  if (address > 0x7F) {
    Serial.printf("[SPI Error] Read address 0x%02X exceeds 7-bit space\n", address);
    return;
  }
  
  if (numBytes > SPI_BUFFER_SIZE) {
    Serial.printf("[SPI Error] Requested bytes %u exceeds buffer %u\n", numBytes, SPI_BUFFER_SIZE);
    return;
  }
  
  if (!buffer) {
    Serial.println("[SPI Error] NULL buffer pointer");
    return;
  }
  
  uint8_t cmdByte = PROTO_CMD_READ | (address & PROTO_ADDR_MASK);
  
  spi.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(SPI_CS, LOW);
  delayMicroseconds(50);
  
  // Master sends command, discard garbage response
  spi.transfer(cmdByte);
  
  // Master sends dummy bytes, slave sends actual data continuously
  for (uint16_t i = 0; i < numBytes; i++) {
    buffer[i] = spi.transfer(0x00);
  }
  
  delayMicroseconds(20);
  digitalWrite(SPI_CS, HIGH);
  spi.endTransaction();
}

// ==================== CONVENIENCE FUNCTIONS ====================

// Read sensor data from slave using protocol-based state machine:
// 1. Set trigger in CTRL register
// 2. Poll STATUS for measurement complete
// 3. Set lock in CTRL register
// 4. Poll STATUS for lock complete
// 5. Bulk read sensor data
// Called from main.cpp every 1 second
void readSlaveData() {
  Serial.println("[Master] === Starting sensor data acquisition ===");
  
  // ========== STEP 1: Set Trigger ==========
  Serial.println("[Master] Step 1: Writing TRIGGER to control register...");
  spiWrite(ADDR_CTRL, CTRL_TRIGGER_MEASUREMENT);
  delay(50);
  
  // ========== STEP 2: Poll for MEASURED status ==========
  Serial.println("[Master] Step 2: Polling for measurement complete...");
  uint32_t startTime = millis();
  uint8_t status = 0;
  bool measured = false;
  
  while (millis() - startTime < 2000) {  // 2 second timeout
    status = spiRead(ADDR_STATUS);
    if (status & STATUS_MEASURED) {
      Serial.printf("[Master] Measurement complete! Status: 0x%02X\n", status);
      measured = true;
      break;
    }
    Serial.printf("  Polling... status: 0x%02X\n", status);
    delay(100);
  }
  
  if (!measured) {
    Serial.println("[Master] ERROR: Timeout waiting for STATUS_MEASURED");
    return;
  }
  
  // ========== STEP 3: Set Lock ==========
  Serial.println("[Master] Step 3: Writing LOCK to control register...");
  spiWrite(ADDR_CTRL, CTRL_LOCK_BUFFERS);
  delay(50);
  
  // ========== STEP 4: Poll for LOCKED status ==========
  Serial.println("[Master] Step 4: Polling for buffers locked...");
  startTime = millis();
  bool locked = false;
  
  while (millis() - startTime < 2000) {  // 2 second timeout
    status = spiRead(ADDR_STATUS);
    if (status & STATUS_LOCKED) {
      Serial.printf("[Master] Buffers locked! Status: 0x%02X\n", status);
      locked = true;
      break;
    }
    Serial.printf("  Polling... status: 0x%02X\n", status);
    delay(100);
  }
  
  if (!locked) {
    Serial.println("[Master] ERROR: Timeout waiting for STATUS_LOCKED");
    return;
  }
  
  // ========== STEP 5: Bulk Read Sensor Data ==========
  Serial.println("[Master] Step 5: Reading bulk sensor data from address 0...");
  
  uint8_t dataBuffer[SPI_BUFFER_SIZE];
  memset(dataBuffer, 0, SPI_BUFFER_SIZE);
  
  spiReadBulk(ADDR_SENSOR_DATA, dataBuffer, SPI_BUFFER_SIZE);
  
  Serial.printf("[Master] Received %u bytes of sensor data\n", SPI_BUFFER_SIZE);
  
  // Debug header
  Serial.printf("[Debug] Raw buffer [0-15]: ");
  for (int i = 0; i < 16; i++) {
    Serial.printf("%02X ", dataBuffer[i]);
  }
  Serial.println();
  
  // Cast packet directly (data starts at byte 0)
  SensorDataPacket *packet = (SensorDataPacket*)(dataBuffer);
  
  // Verify packet integrity
  Serial.printf("[Packet] Sequence: %u | Temp: %.1f°C | Humidity: %.1f%% | Light: %u\n",
    packet->sequence, packet->temperature, packet->humidity, packet->ambientLight);
  
  Serial.printf("[Samples] Accel count: %u | X:%+d Y:%+d Z:%+d mG\n",
    packet->accelSampleCount, packet->accelX, packet->accelY, packet->accelZ);
  
  // Calculate statistics for accel samples
  if (packet->accelSampleCount > 0) {
    int32_t sumX = 0, sumY = 0, sumZ = 0;
    int16_t minX = 32767, maxX = -32768;
    int16_t minY = 32767, maxY = -32768;
    int16_t minZ = 32767, maxZ = -32768;
    
    for (int i = 0; i < packet->accelSampleCount; i++) {
      sumX += packet->accelX_samples[i];
      sumY += packet->accelY_samples[i];
      sumZ += packet->accelZ_samples[i];
      
      if (packet->accelX_samples[i] < minX) minX = packet->accelX_samples[i];
      if (packet->accelX_samples[i] > maxX) maxX = packet->accelX_samples[i];
      if (packet->accelY_samples[i] < minY) minY = packet->accelY_samples[i];
      if (packet->accelY_samples[i] > maxY) maxY = packet->accelY_samples[i];
      if (packet->accelZ_samples[i] < minZ) minZ = packet->accelZ_samples[i];
      if (packet->accelZ_samples[i] > maxZ) maxZ = packet->accelZ_samples[i];
    }
    
    float avgX = (float)sumX / packet->accelSampleCount;
    float avgY = (float)sumY / packet->accelSampleCount;
    float avgZ = (float)sumZ / packet->accelSampleCount;
    
    Serial.printf("[1kHz Accel Stats] X: avg=%+.1f min=%+d max=%+d\n", avgX, minX, maxX);
    Serial.printf("[1kHz Accel Stats] Y: avg=%+.1f min=%+d max=%+d\n", avgY, minY, maxY);
    Serial.printf("[1kHz Accel Stats] Z: avg=%+.1f min=%+d max=%+d\n", avgZ, minZ, maxZ);
  }
  
  // Frame statistics (RGB & IR)
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
  
  Serial.printf("[Frames] RGB: [%u - %u] | IR: [%u - %u]\n", rgbMin, rgbMax, irMin, irMax);
  Serial.println("[Master] === Sensor data acquisition complete! ===\n");
}

// LED control using protocol address writes
void sendIRLED(bool state) {
  Serial.printf("[Master] Setting IR LED: %s\n", state ? "ON" : "OFF");
  spiWrite(ADDR_IR_LED, state ? 0x01 : 0x00);
}

void sendBrightness(uint8_t brightness) {
  if (brightness > 100) brightness = 100;
  Serial.printf("[Master] Setting LED brightness: %u%%\n", brightness);
  spiWrite(ADDR_BRIGHTNESS, brightness);
}

