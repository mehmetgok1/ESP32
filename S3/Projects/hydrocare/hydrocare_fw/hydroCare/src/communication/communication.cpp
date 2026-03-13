#include "communication/communication.h"
#include <Arduino.h>
#include <SPI.h>
#include "config/config.h"
#include "measurement/measurement.h"

#define SPI_CLOCK_HZ     16000000
#define SPI_BUFFER_SIZE  256

// Global storage for latest sensor data from slave
SlaveData latestSlaveData = {};

// SPI instance using HSPI (SPI Bus 2)
SPIClass spi(HSPI);

// DMA-capable SPI buffers
static uint8_t *spiTxBuffer = NULL;
static uint8_t *spiRxBuffer = NULL;

void initSPIComm() {
  // Initialize SPI bus
  spi.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);
  
  // Allocate DMA-capable buffers
  spiTxBuffer = (uint8_t*) heap_caps_malloc(SPI_BUFFER_SIZE, MALLOC_CAP_DMA);
  spiRxBuffer = (uint8_t*) heap_caps_malloc(SPI_BUFFER_SIZE, MALLOC_CAP_DMA);
  
  if (!spiTxBuffer || !spiRxBuffer) {
    Serial.println("[Master] ERROR: DMA buffer allocation failed!");
    while(1) delay(1000);
  }
  
  Serial.println("[Master] SPI Initialized (New Protocol)");
}

// Low-level SPI transfer (handles physical SPI transaction)
uint8_t spiTransfer(uint8_t cmd, uint8_t *rxBuffer, uint16_t bufferSize) {
  if (bufferSize > SPI_BUFFER_SIZE) {
    Serial.printf("[Master] ERROR: Buffer size %u exceeds max %u\n", bufferSize, SPI_BUFFER_SIZE);
    return 0xFF;
  }
  
  // Prepare TX buffer
  memset(spiTxBuffer, 0, SPI_BUFFER_SIZE);
  spiTxBuffer[0] = cmd;
  
  // Prepare RX buffer
  memset(spiRxBuffer, 0, SPI_BUFFER_SIZE);
  
  // Perform SPI transaction
  spi.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(SPI_CS, LOW);
  delayMicroseconds(10);
  
  // Transfer data (master clocks data, slave responds)
  for (uint16_t i = 0; i < bufferSize; i++) {
    spiRxBuffer[i] = spi.transfer(spiTxBuffer[i]);
  }
  
  delayMicroseconds(10);
  digitalWrite(SPI_CS, HIGH);
  spi.endTransaction();
  
  // Copy results to caller's buffer
  if (rxBuffer) {
    memcpy(rxBuffer, spiRxBuffer, bufferSize);
  }
  
  return spiRxBuffer[0];  // Return status byte
}

// ===== NEW PROTOCOL: Multi-step measurement collection =====

// STEP 1: Tell slave to start collecting data
bool triggerMeasurementCycle() {
  Serial.println("[Master] ► Step 1: TRIGGER_MEASUREMENT");
  
  uint8_t status = spiTransfer(CMD_TRIGGER_MEASUREMENT, spiRxBuffer, SPI_BUFFER_SIZE);
  
  Serial.printf("[Master]  Response: Status=0x%02X\n", status);
  
  if (status == 0xFF) {
    Serial.println("[Master] ✗ No response from slave!");
    return false;
  }
  
  return true;
}

// STEP 2: Wait for slave to finish collecting and signal READY
bool waitForSlaveReady(uint32_t timeoutMs) {
  Serial.println("[Master] ► Step 2: WAIT_FOR_READY");
  
  uint32_t startTime = millis();
  uint8_t lastStatus = 0xFF;
  
  while ((millis() - startTime) < timeoutMs) {
    // Poll with CMD_POLL_STATUS (doesn't change state, just reads status)
    uint8_t status = spiTransfer(CMD_POLL_STATUS, spiRxBuffer, SPI_BUFFER_SIZE);
    
    // Print status only on change to reduce spam
    if (status != lastStatus) {
      Serial.printf("[Master]  Status: 0x%02X (MEASURING=%d, READY=%d, TRANSFERRING=%d)\n",
                    status, 
                    (status & 0x01) ? 1 : 0,
                    (status & 0x02) ? 1 : 0,
                    (status & 0x04) ? 1 : 0);
      lastStatus = status;
    }
    
    if (status & STATUS_READY) {
      Serial.printf("[Master]  ✓ Slave READY after %lu ms\n", millis() - startTime);
      return true;
    }
    
    delay(50);  // Poll every 50ms
  }
  
  Serial.printf("[Master] ✗ TIMEOUT waiting for READY after %lu ms\n", timeoutMs);
  Serial.printf("[Master]  Last status: 0x%02X\n", lastStatus);
  return false;
}

// STEP 3: Burst-read sensor data from slave (256 bytes at a time)
bool readSensorDataBurst(SlaveData *outData) {
  if (!outData) return false;
  
  Serial.println("[Master] ► Step 3: BURST_READ");
  
  // First, send CMD_LOCK_BUFFERS to lock slave state for transfer
  Serial.println("[Master]  Sending CMD_LOCK_BUFFERS...");
  uint8_t status = spiTransfer(CMD_LOCK_BUFFERS, spiRxBuffer, SPI_BUFFER_SIZE);
  Serial.printf("[Master]  Response: Status=0x%02X\n", status);
  delay(10);
  
  uint16_t totalBytes = 0;
  uint8_t *dataPtr = (uint8_t*)outData;
  uint16_t dataStructSize = sizeof(SlaveData);
  
  // Read data in chunks until we have everything
  while (totalBytes < dataStructSize) {
    uint32_t bytesToRead = (dataStructSize - totalBytes) > SPI_BUFFER_SIZE 
                           ? SPI_BUFFER_SIZE 
                           : (dataStructSize - totalBytes);
    
    // Perform SPI read (use global DMA buffer)
    uint8_t status = spiTransfer(CMD_READ_DATA, spiRxBuffer, SPI_BUFFER_SIZE);  // Keep sending data chunks
    
    // First byte is status, skip it (it was just status from previous state)
    // Bytes 1+ are actual data
    uint16_t copyBytes = (bytesToRead < (SPI_BUFFER_SIZE - 1)) ? bytesToRead : (SPI_BUFFER_SIZE - 1);
    memcpy(&dataPtr[totalBytes], &spiRxBuffer[1], copyBytes);
    
    totalBytes += copyBytes;
    
    Serial.printf("[Master]  Read %u bytes (total: %u/%u)\n", copyBytes, totalBytes, dataStructSize);
    
    if (copyBytes < (SPI_BUFFER_SIZE - 1)) break;  // Last chunk
    delay(10);  // Small delay between chunks
  }
  
  if (totalBytes >= sizeof(SlaveData) - 10) {  // Allow some tolerance
    Serial.printf("[Master]  ✓ Data transfer complete (%u bytes)\n", totalBytes);
    latestSlaveData = *outData;  // Update global
    
    // Tell slave we're done reading and it can return to IDLE
    Serial.println("[Master] ► Sending TRANSFER_COMPLETE");
    spiTransfer(CMD_TRANSFER_COMPLETE, spiRxBuffer, SPI_BUFFER_SIZE);
    delay(10);
    
    return true;
  }
  
  Serial.printf("[Master] ✗ Incomplete data: got %u/%u bytes\n", totalBytes, dataStructSize);
  return false;
}

// ===== LEGACY FUNCTIONS (backwards compatible) =====

void readAmbientLight_Int() {
  // Old 3-byte protocol (kept for backwards compatibility)
  // Now just reads from the latest synced data
  
  if (latestSlaveData.dataValidMask & DATA_VALID_LIGHT) {
    ambLight_Int = latestSlaveData.ambLight;
    Serial.printf("[Master] Using cached ambient light: %d\n", ambLight_Int);
  } else {
    // Fallback: perform a minimal SPI read (for old protocol)
    Serial.println("[Master] Cache empty, performing legacy SPI read...");
    
    spiTransfer(0x03, spiRxBuffer, 3);  // Old command - use global buffer
    
    ambLight_Int = (spiRxBuffer[1] << 8) | spiRxBuffer[2];
    Serial.printf("[Master] Legacy ambient light: %d\n", ambLight_Int);
  }
}

void sendBrightness(uint8_t brightness) {
  Serial.printf("[Master] ► Sending LED brightness: %d\n", brightness);
  
  memset(spiTxBuffer, 0, SPI_BUFFER_SIZE);
  
  spiTxBuffer[0] = LED_CONTROL_CMD;
  spiTxBuffer[1] = brightness;
  
  spi.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(SPI_CS, LOW);
  delayMicroseconds(10);
  
  for (int i = 0; i < 10; i++) {
    spi.transfer(spiTxBuffer[i]);
  }
  
  delayMicroseconds(10);
  digitalWrite(SPI_CS, HIGH);
  spi.endTransaction();
  
  Serial.printf("[Master]  ✓ LED command sent\n");
}

void sendIRLED(bool state) {
  Serial.printf("[Master] ► Sending IR LED state: %d\n", state);
  
  memset(spiTxBuffer, 0, SPI_BUFFER_SIZE);
  
  spiTxBuffer[0] = IR_CONTROL_CMD;
  spiTxBuffer[1] = state ? 1 : 0;
  
  spi.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(SPI_CS, LOW);
  delayMicroseconds(10);
  
  for (int i = 0; i < 10; i++) {
    spi.transfer(spiTxBuffer[i]);
  }
  
  delayMicroseconds(10);
  digitalWrite(SPI_CS, HIGH);
  spi.endTransaction();
  
  Serial.printf("[Master]  ✓ IR command sent\n");
}