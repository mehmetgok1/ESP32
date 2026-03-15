# HydroCare Master-Slave SPI Protocol Specification

## Overview
Master-Slave SPI protocol using address-based read/write operations for register and bulk data transfer.
Master initiates ALL communication. Slave ONLY responds to commands received.

## SPI Configuration
- **Clock Speed:** 10 MHz
- **Bit Order:** MSBFIRST
- **Mode:** SPI_MODE0 (CPOL=0, CPHA=0)
- **Chip Select:** Active LOW
- **DMA Buffer Size:** 20KB
- **Address Space:** 7-bit (0x00 - 0x7F = 128 addressable locations)

---

## Protocol: Command Byte Format

### Byte 0: Command + Address
```
Bit 7: R/W flag
  1 = READ operation (master requests data from slave)
  0 = WRITE operation (master sends data to slave)

Bits 6-0: Address (0-127)
  Determines which register/memory slave returns/writes
```

**In Code:**
```c
#define PROTO_CMD_READ  0x80    // R/W=1
#define PROTO_CMD_WRITE 0x00    // R/W=0
#define PROTO_ADDR_MASK 0x7F    // Lower 7 bits

cmdByte = PROTO_CMD_READ | (address & PROTO_ADDR_MASK);  // Build command
```

---

## Address Map

### Read-Only Addresses (Slave responses)
| Address | Name | Size | Purpose |
|---------|------|------|---------|
| 0x00 | ADDR_SENSOR_DATA | 20480 bytes | Bulk sensor data packet (>20KB) |
| 0x01 | ADDR_STATUS | 1 byte | Status register (measurement/lock state) |
| 0x05 | ADDR_AMB_LIGHT | 1 byte | Ambient light level |

### Write-Only Addresses (Master commands)
| Address | Name | Size | Value | Purpose |
|---------|------|------|-------|---------|
| 0x02 | ADDR_CTRL | 1 byte | 0x01 | Trigger measurement |
| 0x02 | ADDR_CTRL | 1 byte | 0x02 | Lock buffers |
| 0x03 | ADDR_IR_LED | 1 byte | 0x01/0x00 | Turn IR LED on/off |
| 0x04 | ADDR_BRIGHTNESS | 1 byte | 0-100 | LED brightness (%) |

### Control Register Values (ADDR_CTRL = 0x02)
```c
#define CTRL_TRIGGER_MEASUREMENT 0x01
#define CTRL_LOCK_BUFFERS 0x02
```

### Status Register Values (ADDR_STATUS = 0x01)
```c
#define STATUS_MEASURING 0x01        // Measurement in progress
#define STATUS_MEASURED 0x02         // Measurement complete
#define STATUS_LOCKED 0x04           // Buffers locked for transfer
#define STATUS_READY_TRANSFER 0x08   // Data ready to transfer
```

---

## Single Byte Read (e.g., status register)

```
Master Action              | Slave Action                | Notes
--------------------------|---------------------------|-------------------
1. Pull CS LOW             | Prepare response            |
2. Send [R/W=1|ADDR]       | Receive command             | Slave stores command
   (Master receives GARBAGE)| (not ready to respond yet)  |
3. Send dummy 0x00 byte    | Send data byte              | Master clocks data out
   (Master receives DATA)   | Device sends actual response|
4. Pull CS HIGH            | Processing complete         |
```

**Example: Read status register (ADDR_STATUS = 0x01)**
```
Byte 0: Master sends 0x81 (R/W=1, ADDR=0x01)
        Master receives: GARBAGE (discard)
        
Byte 1: Master sends 0x00 (dummy clock)
        Master receives: STATUS value from slave
```

---

## Single Byte Write (e.g., LED brightness)

```
Master Action              | Slave Action                | Notes
--------------------------|---------------------------|-------------------
1. Pull CS LOW             | Prepare to receive          |
2. Send [R/W=0|ADDR]       | Receive command             | Slave stores address
   (Master receives GARBAGE)| Prepare for data            |
3. Send DATA byte          | Receive and store data      | Slave processes write
   (Master receives GARBAGE)| Execute write operation     |
4. Pull CS HIGH            | Writing complete            |
```

**Example: Set LED brightness to 50% (ADDR_BRIGHTNESS = 0x04)**
```
Byte 0: Master sends 0x04 (R/W=0, ADDR=0x04)
        Master receives: GARBAGE (discard)
        
Byte 1: Master sends 0x32 (50 decimal)
        Master receives: GARBAGE (discard)
        Slave stores: brightness = 50
```

---

## Bulk Read (for large sensor data transfer)

Used for reading large data packets (>20KB) like compressed sensor readings.

```
Master Action              | Slave Action                | Notes
--------------------------|---------------------------|-------------------
1. Pull CS LOW             | Prepare large data buffer   |
2. Send [R/W=1|ADDR_0]     | Receive command             | Slave prepares data
   (Master receives GARBAGE)|Prepare streaming           |
3. Loop: Send 0x00 bytes   | Loop: Send data bytes       | Continuous stream
   (Master receives DATA)   |while bytes_remaining       |until N bytes sent
4. Pull CS HIGH            | Transfer complete           |
```

**Example: Read sensor data (ADDR_SENSOR_DATA = 0x00, 20480 bytes)**
```
Byte 0: Master sends 0x80 (R/W=1, ADDR=0x00)
        Master receives: GARBAGE (discard)
        Slave recognizes ADDR_SENSOR_DATA read request
        Slave prepares to stream sensor packet

Bytes 1-20480: Master sends dummy 0x00 bytes (one per byte)
               Master receives actual sensor data continuously
               
Example byte stream:
  Master sends:  0x00 0x00 0x00 0x00 0x00 ...
  Slave sends:   0x01 0x42 0x03 0x18 0xFF ...  (sensor packet data)
  Master gets:   0x01 0x42 0x03 0x18 0xFF ...  (all 20480 bytes)
```

---

## Sensor Data Packet Structure

Sent when master reads from ADDR_SENSOR_DATA (address 0x00).

```c
#pragma pack(1)
typedef struct {
  // Metadata (31 bytes)
  uint16_t sequence;              // Packet sequence number
  uint16_t ambientLight;          // Ambient light value
  float temperature;              // Temperature in °C
  float humidity;                 // Humidity %
  int16_t accelX, accelY, accelZ; // IMU accel
  int16_t gyroX, gyroY, gyroZ;    // IMU gyro
  uint32_t timestamp_ms;          // System uptime
  uint8_t status;                 // Status flags
  uint16_t accelSampleCount;      // Sample count (1000)
  
  // High-speed samples (8000 bytes)
  int16_t accelX_samples[1000];   // 1kHz accel X
  int16_t accelY_samples[1000];   // 1kHz accel Y
  int16_t accelZ_samples[1000];   // 1kHz accel Z
  uint16_t microphoneSamples[1000]; // 1kHz microphone
  
  // Camera frames (8576 bytes)
  uint16_t rgbFrame[4096];        // RGB565 64x64 image
  uint16_t irFrame[192];          // IR thermal 16x12 image
} SensorDataPacket;  // Total: ~8.7KB
#pragma pack()
```

---

## State Machine Flow (Sensor Acquisition)

### Master-Side Flow
```
1. TRIGGER MEASUREMENT
   └─ Write 0x01 to ADDR_CTRL (0x02)
   
2. POLL STATUS
   └─ Read ADDR_STATUS (0x01) repeatedly
   └─ Wait for STATUS_MEASURED bit set (timeout: 2 sec)
   
3. LOCK BUFFERS
   └─ Write 0x02 to ADDR_CTRL (0x02)
   
4. POLL STATUS AGAIN
   └─ Read ADDR_STATUS (0x01) repeatedly
   └─ Wait for STATUS_LOCKED bit set (timeout: 2 sec)
   
5. READ SENSOR DATA
   └─ Bulk read from ADDR_SENSOR_DATA (0x00)
   └─ Read 20480 bytes continuously
   └─ Parse SensorDataPacket from received data
```

### Slave-Side Flow (IMPORTANT for implementation)
```
IDLE STATE:
  └─ Listen for incoming SPI command byte [R/W | ADDRESS]
  
ON READ REQUEST (R/W=1):
  ├─ If ADDRESS == 0x00 (SENSOR_DATA):
  │  └─ Send prepared sensor packet (20480 bytes)
  │
  ├─ If ADDRESS == 0x01 (STATUS):
  │  └─ Send 1-byte status register
  │
  └─ If ADDRESS == 0x05 (AMB_LIGHT):
     └─ Send 1-byte ambient light value

ON WRITE REQUEST (R/W=0):
  ├─ If ADDRESS == 0x02 (CTRL):
  │  ├─ If DATA == 0x01: Start measurement (set STATUS_MEASURING)
  │  └─ If DATA == 0x02: Lock buffers (set STATUS_LOCKED)
  │
  ├─ If ADDRESS == 0x03 (IR_LED):
  │  └─ Control IR LED on/off
  │
  └─ If ADDRESS == 0x04 (BRIGHTNESS):
     └─ Set LED brightness value
```

---

## Critical Implementation Notes

### For Slave Developer
1. **Master sends FIRST** - Slave never initiates SPI communication
2. **First byte is always garbage** - Discard the response to the command byte
3. **Slave responds on next clock** - When master sends dummy bytes (0x00), that's when slave sends real data
4. **Keep it stateless for now** - Just read command byte, prepare response, send on next byte clock
5. **No handshaking** - Don't send status bytes; just send data

### For Master Developer
1. Always pull CS HIGH between transactions
2. Discard first byte in bulk reads (command echo)
3. Use proper timeouts when polling status (2 second timeout)
4. Validate address space (max 0x7F)
5. Don't send more bytes than buffer size (20KB max)

---

## Example C Code

### Master: Read Status Register
```cpp
uint8_t status = spiRead(ADDR_STATUS);  // Reads 1 byte from address 0x01
// Returns STATUS bits: MEASURING, MEASURED, LOCKED, etc.
```

### Master: Trigger Measurement
```cpp
spiWrite(ADDR_CTRL, CTRL_TRIGGER_MEASUREMENT);  // Write 0x01 to address 0x02
// Slave enters measurement state
```

### Master: Read Sensor Data
```cpp
uint8_t buffer[SPI_BUFFER_SIZE];
spiReadBulk(ADDR_SENSOR_DATA, buffer, SPI_BUFFER_SIZE);  // Read 20KB from address 0x00
SensorDataPacket* packet = (SensorDataPacket*)(buffer);
// Parse packet data
```

---

## Timing

- **Single read/write:** ~2µs per byte + delays
- **Bulk read (20KB):** ~2ms at 10MHz + overhead
- **Status polling:** 100ms interval × 2 sec timeout = up to 20 polls
- **Full measurement cycle:** ~2 sec (measurement) + 200ms (polling) + 2ms (read)

---

## Differences from Old Protocol

| Feature | Old | New |
|---------|-----|-----|
| Command style | Fixed opcodes (0x01, 0x02, etc.) | Address-based R/W |
| Registers | N/A | 7-bit address space (128 locations) |
| Data handling | Status byte always sent | Clean separation: cmd then data |
| Bulk transfer | Hardcoded CMD_TRANSFER_DATA | Generic address 0x00 |
| Flexibility | Limited | Easy to add new registers |

