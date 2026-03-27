# HydroCare Master-Slave System Architecture

## System Overview

HydroCare is a **distributed IoT sensor system** with two ESP32-S3 processors:

1. **Slave (Sensor Board)** - Data acquisition and preprocessing
   - Location: `/hydrocare_sensor/hydroCare_Sensor/`
   - Role: Continuous sensor sampling, buffering, caching
   - Output: Binary sensor packets via SPI to master

2. **Master (Main Board)** - Data logging and BLE connectivity
   - Location: `/hydrocare_fw/hydroCare/`
   - Role: Orchestrates measurements, logs to SD card, sends BLE notifications
   - Output: Binary files on SD card (`part_0.bin`, `part_50.bin`, etc.)

---

## Physical Connection (SPI Protocol)

### Hardware Configuration
```
Clock Speed:    10 MHz
SPI Mode:       SPI_MODE0 (CPOL=0, CPHA=0)
Bit Order:      MSBFIRST
CS Active:      LOW
DMA Buffers:    25.6 KB (sufficient for ~24.6 KB packet + overhead)
Max Address:    0x7F (128 locations)
```

### Pinout (Typical ESP32-S3)
```
Master                Slave
─────────────────────────────
GPIO_MOSI_M   ←→   GPIO_MOSI_S    (Command/Data TO slave)
GPIO_MISO_M   ←→   GPIO_MISO_S    (Response FROM slave)
GPIO_SCK_M    ←→   GPIO_SCK_S     (Clock)
GPIO_CS_M     ←→   GPIO_CS_S      (Chip Select - Active LOW)
GND           ←→   GND            (Common ground)
```

---

## Communication Protocol

### Packet Structure (Byte Level)

**Master sends 3-byte command, Slave responds simultaneously.**

#### Single-Byte Read (2 bytes total)
```
Master TX:  [R/W=1 | ADDR]  [0x00]
Slave TX:   [DUMMY      ]   [DATA]
            ↑                ↑
            Discarded       Received by master
```
Master reads data at byte 1.

#### Single-Byte Write (3 bytes total)
```
Master TX:  [R/W=0 | ADDR]  [0x00]  [DATA]
Slave TX:   [DUMMY      ]   [STATUS][ACK]
            ↑                ↑
            Discarded       Discarded
Slave processes write after CS goes HIGH.
```

#### Bulk Read - 25.6 KB Sensor Packet
```
Master TX:  [R/W=1 | ADDR]  [0x00]  [0x00] × 25598 bytes (DMA stream)
Slave TX:   [DUMMY      ]   [STATUS][SENSOR DATA (25596 bytes)]
            ↑                ↑
            Discarded       Received
```
Master reads full packet as DMA transfer in one blocking call.

---

## Control Registers (Master-to-Slave Commands)

| Addr | Name | Type | Purpose |
|------|------|------|---------|
| 0x02 | CTRL | Write | Measurement control (trigger/lock/unlock) |
| 0x03 | IR_LED | Write | IR LED control (0=off, 1=on) |
| 0x04 | BRIGHTNESS | Write | LED brightness (0-100%) |

### CTRL Register Values
```c
#define CTRL_TRIGGER_MEASUREMENT   0x01   // Start measurement
#define CTRL_LOCK_BUFFERS          0x02   // Freeze & prepare data
#define CTRL_UNLOCK_BUFFERS        0x03   // Release for next cycle
```

### Status Register (Slave-to-Master Responses)

**Byte 1 of every SPI transaction (status byte):**
```c
#define STATUS_IDLE         0x00   // Ready for next trigger
#define STATUS_MEASURING    0x01   // Currently measuring
#define STATUS_MEASURED     0x02   // Measurement complete, data ready
#define STATUS_LOCKED       0x04   // Data locked, can be read
```

---

## Measurement State Machine

The slave operates in a strict finite state machine:

```
IDLE
  ↓
  [Master sends TRIGGER_MEASUREMENT]
  ↓
MEASURING (background task collects packet)
  ↓
  [Measurement completes automatically (~11ms)]
  ↓
MEASURED (packet in currentData buffer, ready)
  ↓
  [Master sends LOCK_BUFFERS]
  ↓
READY_TRANSFER (data copied to txBuf, can be read)
  ↓
  [Master reads 25.6 KB packet via SPI bulk read]
  ↓
  [Master sends UNLOCK_BUFFERS]
  ↓
IDLE (ready for next cycle)
```

**Critical detail:** Data is LOCKED before master reads it, preventing race conditions with background sampling tasks.

---

## What Data is in Each Packet (24.6 KB Total)

### SensorDataPacket Structure (from slave)

```c
#pragma pack(1)
typedef struct {
  // === METADATA (31 bytes) ===
  uint16_t sequence;           // Packet counter (0, 1, 2, ...)
  uint16_t ambientLight;       // Ambient light sensor
  float temperature;           // Average IR thermal temp (°C)
  float humidity;              // BME688 humidity (%)
  int16_t accelX, accelY, accelZ;   // IMU latest accel
  int16_t gyroX, gyroY, gyroZ;      // IMU latest gyro
  uint32_t timestamp_ms;       // System uptime (ms)
  uint8_t status;              // Status flags
  uint16_t accelSampleCount;   // Always 2000
  
  // === HIGH-SPEED SAMPLES (8000 bytes) ===
  // Continuous 2kHz sampling for 1 second
  int16_t accelX_samples[2000];      // X acceleration @ 2kHz
  int16_t accelY_samples[2000];      // Y acceleration @ 2kHz
  int16_t accelZ_samples[2000];      // Z acceleration @ 2kHz
  uint16_t microphoneSamples[2000];  // Audio @ 2kHz
  
  // === CAMERA FRAMES (8576 bytes) ===
  uint16_t rgbFrame[4096];    // RGB565 64×64 pixels (center crop of 320×240)
  uint16_t irFrame[192];      // Thermal 16×12 pixels, fixed-point:
                               // (raw / 100.0) - 40 = temp_°C
} SensorDataPacket;  // Total: ~24.6 KB (padded to 25.6 KB for SPI buffer)
#pragma pack()
```

### CombinedDataPacket Structure (master layer on top)

The **master** wraps slave data with its own sensor readings:

```c
typedef struct {
  // === MASTER SENSORS (26 bytes) ===
  float batteryLevel;          // Battery voltage
  float batteryPercentage;     // Battery 0-100%
  float ambLight;              // Ambient light
  uint16_t ambLight_Int;       // Ambient light (uint16)
  float PIRValue;              // PIR motion detection
  uint16_t movingDist;         // mmWave moving target distance
  uint8_t movingEnergy;        // mmWave moving target energy
  uint16_t staticDist;         // mmWave static target distance
  uint8_t staticEnergy;        // mmWave static target energy
  uint16_t detectionDist;      // mmWave detection distance
  
  // === SLAVE DATA (24.6 KB) ===
  SensorDataPacket slaveData;  // Complete slave packet
} CombinedDataPacket;  // Total: ~24.6 KB
#pragma pack()
```

---

## How Master Gets Data (Per Loop Cycle)

**Timing: Every 1000ms (1Hz timer, controlled by BLE callback)**

### Step 1: Trigger Measurement on Slave
```cpp
Master sends: WRITE [ADDR_CTRL] = CTRL_TRIGGER_MEASUREMENT
Slave enters: MEASURING state
Slave task starts: ambient light, IMU samples, IR read, RGB capture (11ms total)
```

### Step 2: Wait for Measurement Complete
```cpp
Master polls: READ [ADDR_STATUS] → wait for STATUS_MEASURED
Slave task does: Lock buffers, copy finalData → txBuf
Polling continues until slave returns STATUS_LOCKED
```

### Step 3: Bulk Read Sensor Packet
```cpp
Master issues: SPI bulk read from ADDR_SENSOR_DATA
Master receives: 25.6 KB of SensorDataPacket
Slave: Data streamed via DMA
Duration: ~25 ms @ 10 MHz clock
```

### Step 4: Lock Buffers (Prepare Data)
```cpp
Master sends: WRITE [ADDR_CTRL] = CTRL_LOCK_BUFFERS
Slave: Copies currentData (with new timestamp/sequence) into txBuf
Slave state: READY_TRANSFER
```

### Step 5: Unlock and Next Cycle
```cpp
Master sends: WRITE [ADDR_CTRL] = CTRL_UNLOCK_BUFFERS
Slave state: IDLE (ready for next TRIGGER)
Master: Returns to main loop
```

**One complete cycle duration:** ~100ms for measurement + 25ms for SPI read = ~125ms net

---

## Master-Side Data Processing

### Step A: Combine Master + Slave Data
```cpp
CombinedDataPacket packet;
packet.batteryLevel = measureBatteryLevel();
packet.ambLight = measureAmbLight();
packet.PIRValue = measurePIR();
packet.movingDist = measuremmWave();
// ... etc (all master sensors)

memcpy(&packet.slaveData, slaveSensorPacket, sizeof(SensorDataPacket));
```

### Step B: Queue for SD Logging (Non-Blocking)
```cpp
xQueueSend(sensorDataQueue, &packet, 0);  // Enqueue, don't block
```
SD logging task (running on Core 1, priority 1) processes queue independently.

### Step C: Process for BLE Notifications
```cpp
downsampleRGBFrame(slaveData->rgbFrame, downsampled16x16);
memcpy(irFrame16x12, slaveData->irFrame, sizeof(irFrame16x12));
notifyAll();  // Send BLE notifications
```

---

## RGB Downsampling for BLE

### Why Downsampling?
- **Slave captures:** 64×64 RGB pixels = 4,096 pixels × 2 bytes = 8,192 bytes per frame
- **BLE MTU:** Limited to ~200 bytes per notification (with overhead)
- **Solution:** Downsampling to 16×16 = 256 pixels × 2 bytes = 512 bytes

### Downsampling Algorithm

```cpp
void downsampleRGBFrame(uint16_t* src64x64, uint16_t* dst16x16) {
  // Input: 64×64 RGB565 frame (4,096 pixels)
  // Output: 16×16 RGB565 frame (256 pixels)
  // Ratio: 4×4 pixels → 1 output pixel (every 4th pixel, every 4th row)
  
  int dstIdx = 0;
  for (int row = 0; row < 64; row += 4) {
    for (int col = 0; col < 64; col += 4) {
      int srcIdx = (row * 64) + col;
      dst16x16[dstIdx++] = src64x64[srcIdx];
    }
  }
  // Result: Simple nearest-neighbor sampling (every 4th pixel)
}
```

### BLE Transmission Flow
```
Downsampled 16×16 frame (512 bytes)
         ↓
    Split into chunks (200 bytes each)
         ↓
    Notify client with 3 BLE notifications:
    - Chunk 1: 200 bytes
    - Chunk 2: 200 bytes  
    - Chunk 3: 112 bytes
         ↓
    Client reassembles into full frame
```

---

## SD Card Data Storage

### File Organization
```
/YYYYMMDD_HHMMSS/                    ← Session folder (created per BLE start)
├── YYYYMMDD_HHMMSS_part_0.bin       ← Packets 0-49
├── YYYYMMDD_HHMMSS_part_50.bin      ← Packets 50-99
├── YYYYMMDD_HHMMSS_part_100.bin     ← Packets 100-149
└── ...
```

### Per-Packet Logging
```cpp
while (xQueueReceive(sensorDataQueue, &packet, portMAX_DELAY)) {
  // Validate packet (sequence != 0, temperature != 0.0)
  // Calculate file path: part_(packets / 50) * 50
  // Write: df.write(&packet, sizeof(CombinedDataPacket))  [~50µs write]
  // Close: df.close()                                      [~25ms close]
  // Total: ~100ms per packet
}
```

### Data Durability
- File is **closed after every packet** (not kept open)
- Ensures data is flushed to SD card immediately
- Trade-off: Close overhead (~25ms) vs. data safety (acceptable)

---

## FreeRTOS Background Tasks

### Slave-Side (hydrocare_sensor/)

| Task | Core | Priority | Function | Frequency |
|------|------|----------|----------|-----------|
| highSpeedSamplerTask | 1 | 1 | 2kHz accel+mic ring buffer | Continuous |
| irSensorBackgroundTask | 1 | 1 | Read IR thermal sensor | 200ms |
| bmeSensorBackgroundTask | 1 | 1 | Read BME688 environment | 200ms |
| measurementCollectorTask | 0 | 2 | Assemble final packet on TRIGGER | On-demand |
| receiveCommand (main loop) | 0 | - | SPI handler (blocking) | Continuous |

### Master-Side (hydrocare_fw/)

| Task | Core | Priority | Function | Stack |
|------|------|----------|----------|-------|
| sdCardLoggingTask | 1 | 1 | Queue consumer, writes binary to SD | 16 KB |
| (main loop) | 0 | - | Sensor acquisition, BLE notify | - |

---

## Performance Metrics

| Operation | Time | Notes |
|-----------|------|-------|
| Slave measurement | ~11 ms | IMU + camera + IR + BME |
| SPI transfer (25.6 KB) | ~25 ms | @ 10 MHz clock |
| SD write (24.6 KB) | ~50 µs | DMA optimized |
| SD close | ~25 ms | Flush to media |
| Master loop cycle | 100-140 ms | < 1000 ms timer requirement ✓ |

---

## Data Quality & Integrity

### Ring Buffer (Slave-Side)
- Size: 5,000 samples @ 2kHz = 2.5 seconds of history
- Prevents race conditions: Measurement reader always far behind sampler
- Thread-safe: Single writer (no mutex needed)

### Double Buffering (Slave Caches)
```cpp
// IR and BME sensors update async (200ms)
// Use double buffers to prevent mid-read updates
irCache[2];    // Master reads from [1 - irWriteIdx]
               // Background task writes to [irWriteIdx]
```

### CombinedDataPacket Validation (Master-Side)
```cpp
if (packet.slaveData.sequence == 0 || packet.slaveData.temperature == 0.0) {
  // Drop invalid packet
}
```

---

## Temperature Conversion (Post-Processing)

### IR Thermal Data
Stored as **fixed-point uint16_t** for space efficiency:
```
Raw in file:    6341
Conversion:     (6341 / 100.0) - 40 = 23.41°C
```

### Average Temperature
Stored directly as `float` in packet:
```
Value in file:  19.5
Already in °C:  19.5°C (no conversion needed)
```

---

## System Workflow (Complete Cycle)

```
[IDLE]
  ↓
[BLE "Start" command arrives]
  ↓
  Master: sessionFolder = timestamp, initSessionFolder()
          sessionInitialized = true
  ↓
[1000ms timer fires (1 Hz)]
  ↓
  Master: readSlaveData() → TRIGGER_MEASUREMENT
             ↓
             Slave: measurementCollectorTask wakes up
                    Collects all sensor data
                    Locks buffers
             ↓
             Master: reads 25.6 KB packet via SPI
  ↓
  Master: measureBatteryLevel(), measureAmbLight(), measurePIR(), measuremmWave()
  ↓
  Master: Combine master + slave → CombinedDataPacket
  ↓
  Master: xQueueSend() → packet to queue
             ↓
             SD Task: Dequeues packet
                      Open/create part file
                      Write 24.6 KB to SD
                      Close file
                      [~100ms total]
  ↓
  Master: downsampleRGBFrame(), copy IR frame
  ↓
  Master: notifyAll() → BLE RGB + IR notifications
  ↓
  [Wait 1000ms until next timer]
  ↓
[Repeat]
  ↓
[BLE "Stop" command arrives]
  ↓
  Master: sessionInitialized = false
  ↓
[IDLE - ready for next session]
```

---

## Summary

**Master-Slave architecture enables:**
- ✅ **Parallel processing:** Slave samples @ 2kHz, master logs @ 1Hz
- ✅ **Data efficiency:** Binary format, 50-packet files, uint16 compression
- ✅ **Reliability:** Data locked before read, double buffering, validation
- ✅ **Scalability:** Queue-based logging, non-blocking operations
- ✅ **Real-time:** BLE notifications with downsampled frames

All sensor data converges into **CombinedDataPacket (~24.6 KB)** every second, logged to SD card and streamed via BLE.
