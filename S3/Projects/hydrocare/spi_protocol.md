# HydroCare SPI Protocol

## Hardware Configuration
```
Clock:        10 MHz
Mode:         SPI_MODE0 (CPOL=0, CPHA=0)  
Bit Order:    MSBFIRST
CS:           Active LOW
DMA Buffer:   20 KB
Address Space: 0x00-0x7F (128 locations, 7-bit)
```

## Command Byte Format
```
Bit 7:     R/W flag (1=READ, 0=WRITE)
Bits 6-0:  Address (0x00-0x7F)
```

Code constants:
```c
#define PROTO_CMD_READ   0x80
#define PROTO_CMD_WRITE  0x00
#define PROTO_ADDR_MASK  0x7F
```

## Address Map

| Addr | Name | Type | Size | Purpose |
|------|------|------|------|---------|
| 0x00 | ADDR_SENSOR_DATA | Read | 20480 B | Bulk sensor packet (pre-prepared) |
| 0x01 | ADDR_STATUS | Read | 1 B | Status register (constantly updated) |
| 0x02 | ADDR_CTRL | Write | 1 B | Control: 0x01=trigger, 0x02=lock |
| 0x03 | ADDR_IR_LED | Write | 1 B | IR LED: 0x01=on, 0x00=off |
| 0x04 | ADDR_BRIGHTNESS | Write | 1 B | LED brightness 0-100% |
| 0x05 | ADDR_AMB_LIGHT | Read | 1 B | Ambient light level |

Status bits:
```c
#define STATUS_MEASURING 0x01      // Measurement in progress
#define STATUS_MEASURED 0x02       // Measurement complete  
#define STATUS_LOCKED 0x04         // Buffers locked & data prefilled
```

## Transaction Formats

### Single Byte Read (2 bytes total)
```
Byte 0: Master sends [R/W=1 | ADDR]  ← Slave sends DUMMY (discard)
Byte 1: Master sends 0x00            ← Slave sends DATA/STATUS
```
Master receives the actual data at byte 1.

### Single Byte Write (3 bytes total)  
```
Byte 0: Master sends [R/W=0 | ADDR]  ← Slave sends DUMMY (discard)
Byte 1: Master sends 0x00            ← Slave sends STATUS
Byte 2: Master sends DATA            ← Slave sends ACK (discard)
```
Slave processes write after CS goes HIGH.

### Bulk Read - 20KB (20482 bytes total)
```
Byte 0: Master sends [R/W=1 | ADDR]  ← Slave sends DUMMY (discard)
Byte 1: Master sends 0x00            ← Slave sends STATUS
Bytes 2-20481: Master sends 0x00 stream  ← Slave sends pre-prepared data
```
All sensor data pre-filled in slave buffer before transaction.

## Sensor Data Packet

Returned by bulk read (address 0x00):
```c
#pragma pack(1)
typedef struct {
  // Metadata (31 bytes)
  uint16_t sequence;
  uint16_t ambientLight;
  float temperature;
  float humidity;
  int16_t accelX, accelY, accelZ;
  int16_t gyroX, gyroY, gyroZ;
  uint32_t timestamp_ms;
  uint8_t status;
  uint16_t accelSampleCount;
  
  // Samples (8000 bytes) - 1kHz data
  int16_t accelX_samples[1000];
  int16_t accelY_samples[1000];
  int16_t accelZ_samples[1000];
  uint16_t microphoneSamples[1000];
  
  // Frames (8576 bytes)
  uint16_t rgbFrame[4096];     // 64×64 RGB565
  uint16_t irFrame[192];       // 16×12 thermal
} SensorDataPacket;  // Total: 16,607 bytes
#pragma pack()
```

## Master Sequence (Sensor Acquisition)

1. **TRIGGER**: `spiWrite(ADDR_CTRL, 0x01)` → Slave sets STATUS_MEASURING
2. **POLL** (2 sec timeout): `status = spiRead(ADDR_STATUS)` → Wait for STATUS_MEASURED
3. **LOCK**: `spiWrite(ADDR_CTRL, 0x02)` → Slave locks buffers and pre-prepares data
4. **POLL** (2 sec timeout): `status = spiRead(ADDR_STATUS)` → Wait for STATUS_LOCKED
5. **READ**: `spiReadBulk(ADDR_SENSOR_DATA, buffer, 20480)` → Get pre-prepared packet

## Slave Sequence (SYNCHRONOUS)

```
IDLE
  ↓ Trigger command received
MEASURING ← Set STATUS_MEASURING, start measurement hardware
  ↓ Measurement complete (hardware done)
MEASURED ← Set STATUS_MEASURED, master will poll this
  ↓ Lock command received
LOCKED ← Set STATUS_LOCKED, PRE-PREPARE all sensor data to buffer
  ↓ Master bulk read starts
TRANSFERRING ← Stream pre-prepared data bytes (no computation)
  ↓ CS HIGH
  → Ready for next cycle
```

**Key: All data pre-filled AFTER lock command, just stream during transfer.**

## Implementation Rules

**Master:**
- Always set CS HIGH between transactions
- Discard first byte in all transactions (it's garbage)
- Use 2 second timeouts when polling STATUS
- Maximum address: 0x7F

**Slave:**
- Master sends FIRST (never initiate)
- Always respond with DUMMY at byte 0
- Keep STATUS register constantly updated (background task)
- Pre-prepare sensor data AFTER lock command, then stream it
- No interrupts or async during transactions (fully synchronous)

## Timing

- Single read: ~100 µs (including CS delays)
- Single write: ~150 µs (including CS delays)
- Bulk read 20KB: ~2 ms (at 10 MHz)
- Full measurement: ~2 sec (hardware) + 200 ms (polling) + 2 ms (transfer)

