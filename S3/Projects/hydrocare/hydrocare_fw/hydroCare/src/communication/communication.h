#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include <SPI.h>

// ===== Protocol Commands =====
#define CMD_POLL_STATUS          0x00  // Master: Poll slave status (no state change)
#define CMD_TRIGGER_MEASUREMENT  0x01  // Master: Tell slave to collect all sensor data
#define CMD_LOCK_BUFFERS         0x02  // Master: Freeze slave buffers, start transfer
#define CMD_READ_DATA            0x04  // Master: Read next data chunk (keep slave in TRANSFERRING)
#define CMD_TRANSFER_COMPLETE    0x03  // Master: Done reading, slave return to IDLE
#define CMD_ACK                  0xAA  // Slave: Acknowledge
#define CMD_ERROR                0xFF  // Slave: Error occurred

// Legacy commands (still supported)
#define LED_CONTROL_CMD  0x10
#define IR_CONTROL_CMD   0x11

// ===== Status Byte Bits (from Slave) =====
#define STATUS_MEASURING   0x01  // Bit 0: Slave is currently measuring
#define STATUS_READY       0x02  // Bit 1: Slave has data ready
#define STATUS_TRANSFERRING 0x04 // Bit 2: Slave is in transfer mode

// ===== Data Validity Masks (from Slave) =====
#define DATA_VALID_IMU      0x01  // Bit 0: IMU data valid
#define DATA_VALID_LIGHT    0x02  // Bit 1: Ambient light valid
#define DATA_VALID_IR       0x04  // Bit 2: IR frame valid
#define DATA_VALID_RGB      0x08  // Bit 3: RGB frame valid

// ===== Sensor Data Structure (received from slave) =====
typedef struct {
  float ax, ay, az;           // IMU acceleration (12 bytes)
  uint16_t ambLight;          // Ambient light sensor (2 bytes)
  uint16_t irFrame[192];      // IR thermal (16x12 = 192 pixels, 384 bytes)
  uint16_t rgbFrame[4096];    // RGB camera (64x64, 8192 bytes)
  uint32_t timestamp_ms;      // Timestamp (4 bytes)
  uint8_t dataValidMask;      // Which sensors have valid data
} SlaveData;

// Initialization
void initSPIComm();

// New protocol: Master initiates data collection cycle
bool triggerMeasurementCycle();      // Step 1: Start slave measurement
bool waitForSlaveReady(uint32_t timeoutMs);  // Step 2: Poll until ready
bool readSensorDataBurst(SlaveData *outData); // Step 3: Burst-read all data

// Legacy functions (still work with backwards-compatible protocol)
void sendBrightness(uint8_t brightness);
void sendIRLED(bool state);

// Old polling function (deprecated, kept for compatibility)
void readAmbientLight_Int();

// Helpers
uint8_t spiTransfer(uint8_t cmd, uint8_t *rxBuffer, uint16_t bufferSize);

// Global storage for latest slave sensor data
extern SlaveData latestSlaveData;

#endif // COMMUNICATION_H