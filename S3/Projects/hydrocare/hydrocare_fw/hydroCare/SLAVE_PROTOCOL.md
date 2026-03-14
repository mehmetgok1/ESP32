# HydroCare Slave SPI Protocol Specification

## Overview
Master-Slave SPI protocol for sensor data transfer. Master initiates all communication. Slave responds based on state machine.

## SPI Configuration
- **Clock Speed:** 10 MHz
- **Bit Order:** MSBFIRST
- **Mode:** SPI_MODE0
- **Chip Select:** Active LOW
- **Buffer Size:** 8704 bytes per transaction

## Protocol Commands

```c
#define CMD_NONE 0x00              // Poll/status request
#define CMD_TRIGGER_MEASUREMENT 0x01   // Start measurement
#define CMD_LOCK_BUFFERS 0x02      // Lock measurement buffers
#define CMD_TRANSFER_DATA 0x03     // Start data transfer
#define CMD_ACK 0xAA               // Acknowledgment
#define CMD_ERROR 0xFF             // Error response
```

## Status Bits (Slave Response Byte 0)

```c
#define STATUS_MEASURING 0x01      // Currently measuring
#define STATUS_MEASURED 0x02       // Measurement complete
#define STATUS_LOCKED 0x04         // Buffers locked
#define STATUS_READY_TRANSFER 0x08 // Data ready to transfer
```

## Packet Structure (sends in bytes 1-8703)

```c
#pragma pack(1)
typedef struct {
  uint16_t sequence;              // Packet sequence number (auto-increment)
  uint16_t ambientLight;          // Ambient light value
  float temperature;              // Temperature in °C
  float humidity;                 // Humidity %
  int16_t accelX, accelY, accelZ; // IMU accel
  int16_t gyroX, gyroY, gyroZ;    // IMU gyro
  uint32_t timestamp_ms;          // System uptime
  uint8_t status;                 // Status flags (copy of status byte)
  
  // Camera data frames
  uint16_t rgbFrame[4096];        // RGB565 64x64 (8192 bytes)
  uint16_t irFrame[192];          // IR thermal 16x12 (384 bytes)
} SensorDataPacket;
#pragma pack()
```

**Total size:** 8605 bytes (fits in 8703 available bytes)

---

## State Machine

### State 1: IDLE
- Wait for master command
- Always respond with current status on SPI

### State 2: MEASURING (after CMD_TRIGGER_MEASUREMENT)
1. Set status = `STATUS_MEASURING`
2. Collect sensor data (temperature, humidity, IMU, light, RGB, IR)
3. When complete: Set status = `STATUS_MEASURED`

### State 3: LOCKED (after CMD_LOCK_BUFFERS)
1. Set status = `STATUS_LOCKED`
2. Prepare buffer for transfer (copy measurement data to transfer buffer)
3. When ready: Set status = `STATUS_READY_TRANSFER`

### State 4: TRANSFERRING (after CMD_TRANSFER_DATA)
1. Master sends `CMD_TRANSFER_DATA` in byte 0, keeps CS LOW for 8704 bytes
2. Slave recognizes byte 0 = `0x03` → start sending data
3. Send status byte (byte 0 of response)
4. Send complete `SensorDataPacket` (bytes 1-8703)
5. Return to IDLE

---

## Communication Flow

### Step 1: TRIGGER (10-byte handshake)
```
Master:  [CS=LOW] 0x01 0x00 0x00 ... 0x00 [CS=HIGH]  (10 bytes)
Slave:   Response with current status in byte 0
```
- Slave enters MEASURING state
- Continue collecting sensor data

### Step 2: POLL UNTIL MEASURED (repeated 10-byte handshakes)
```
Master:  [CS=LOW] 0x00 0x00 0x00 ... 0x00 [CS=HIGH]  (10 bytes, repeated)
Slave:   Response: status byte with STATUS_MEASURED bit set when done
```
- Master polls every 10ms up to 2 seconds timeout

### Step 3: LOCK BUFFERS (10-byte handshake)
```
Master:  [CS=LOW] 0x02 0x00 0x00 ... 0x00 [CS=HIGH]  (10 bytes)
Slave:   Response with status
```
- Slave enters LOCKED state
- Copies/prepares measurement data for transfer

### Step 4: POLL UNTIL LOCKED (repeated 10-byte handshakes)
```
Master:  [CS=LOW] 0x00 0x00 0x00 ... 0x00 [CS=HIGH]  (10 bytes, repeated)
Slave:   Response: status byte with STATUS_LOCKED bit set when done
```

### Step 5: TRANSFER DATA (8704-byte continuous transfer)
```
Master:  [CS=LOW] 0x03 0x00 0x00 ... 0x00 [CS=HIGH]  (8704 bytes, CONTINUOUS)
         Byte 0: CMD_TRANSFER_DATA
         Bytes 1-8703: padding zeros

Slave:   Byte 0: STATUS_READY_TRANSFER (immediate response to 0x03)
         Bytes 1-8703: Complete SensorDataPacket
```

**IMPORTANT:** CS stays LOW for entire 8704-byte transfer. No gaps.

---

## Byte-Level Details

### All Transactions Start with CS Assertion
```
1. Master sets CS = LOW
2. Wait 50 microseconds (slave setup time)
3. Begin SPI transfer
4. Wait 20 microseconds (cleanup)
5. Master sets CS = HIGH
```

### What Slave Should Do

1. **Listen on CS pin** - When LOW, prepare for SPI
2. **Read Byte 0** - Contains command code
3. **Respond appropriately:**
   - If byte 0 = `0x01` → TRIGGER received, set MEASURING status
   - If byte 0 = `0x02` → LOCK received, set LOCKED status
   - If byte 0 = `0x03` → TRANSFER received, send status + full packet
   - If byte 0 = `0x00` → POLL received, just send current status

---

## Key Implementation Notes

### Status Response Timing
- Always send status byte first (byte 0 of any response)
- Slave can prepare this byte **while** receiving command in byte 0
- Full-duplex: while master sends byte 0, simultaneously receive status

### Data Transfer Strategy
- After `CMD_TRANSFER_DATA` received, slave has limited time to respond
- Must be ready to send full packet immediately (0 delay if possible)
- Use double-buffering: one buffer for measurement, one for transfer

### Timing Constraints
- Measurement (triggered by 0x01): typically 100-1000ms
- Master polls every 10ms
- Timeouts: 2000ms per poll phase

---

## Example Slave FSM Pseudocode

```cpp
enum SlaveState {
  IDLE,
  MEASURING,
  READY_FOR_LOCK,
  LOCKED,
  READY_FOR_TRANSFER,
  ERROR
};

SlaveState state = IDLE;

void spiISR() {
  byte cmd = receivedByte[0];
  
  switch(cmd) {
    case CMD_TRIGGER_MEASUREMENT:
      state = MEASURING;
      startMeasurement();
      sendStatusByte(STATUS_MEASURING);
      break;
      
    case CMD_LOCK_BUFFERS:
      if(state == READY_FOR_LOCK) {
        state = LOCKED;
        prepareDataForTransfer();
        sendStatusByte(STATUS_LOCKED);
      }
      break;
      
    case CMD_TRANSFER_DATA:
      if(state == READY_FOR_TRANSFER) {
        sendStatusByte(STATUS_READY_TRANSFER);
        sendCompletePacket(8703 bytes);
        state = IDLE;  // Return to idle after transfer
      }
      break;
      
    case CMD_NONE:
      sendStatusByte(getCurrentStatus());
      break;
  }
}

// When measurement complete:
if(measurementDone) {
  state = READY_FOR_LOCK;  // Wait for LOCK command
}

// When data prepared for transfer:
if(dataReady) {
  state = READY_FOR_TRANSFER;  // Wait for TRANSFER command
}
```

---

## Testing Checklist

- [ ] Slave responds to `CMD_TRIGGER_MEASUREMENT` with `STATUS_MEASURING`
- [ ] Slave sets `STATUS_MEASURED` after measurement completes
- [ ] Slave responds to `CMD_LOCK_BUFFERS` with `STATUS_LOCKED`
- [ ] Slave sets `STATUS_READY_TRANSFER` when ready
- [ ] Slave correctly formats packet with status byte first
- [ ] Full 8704-byte transfer completes without errors
- [ ] Sequence number increments on each transfer
- [ ] Camera frames contain valid data (RGB min/max, IR min/max)
- [ ] Timestamps reasonable (system uptime)
- [ ] Returns to IDLE after transfer completed
