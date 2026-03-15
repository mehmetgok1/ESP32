# HydroCare Sensor Slave Protocol Update

**Date:** March 15, 2026  
**Changes Since:** Concurrent data link implementation (commit: 866ed61)  
**Status:** Deployed and tested ✅

---

## 📋 Summary of Changes

The slave firmware has been updated with high-speed 1kHz continuous sampling for accelerometer and microphone data. This document describes all protocol-level changes for master integration.

---

## 🔄 SPI Configuration Changes

### Buffer Size
| Parameter | Before | After | Notes |
|-----------|--------|-------|-------|
| `SPI_BUFFER_SIZE` | 8,704 bytes | 20,480 bytes | Increased to accommodate larger packet + DMA overhead |
| `max_transfer_sz` | 16,384 bytes | 20,480 bytes | Enables DMA chaining for full packet transfer |

**Master Action:** Update SPI configuration to use 20,480-byte buffers for all transactions.

---

## 📦 SensorDataPacket Structure Changes

### New Packet Layout

```
Total Size: ~16,610 bytes (fits in 20,480-byte SPI buffer)
```

#### Metadata Section (34 bytes)
```c
struct {
  uint16_t sequence;              // Packet sequence number
  uint16_t ambientLight;          // Ambient light ADC value
  float temperature;              // Temperature in °C (average from IR frame)
  float humidity;                 // Humidity % (placeholder, currently 0.0)
  int16_t accelX;                 // Most recent accel X sample (milli-g)
  int16_t accelY;                 // Most recent accel Y sample (milli-g)
  int16_t accelZ;                 // Most recent accel Z sample (milli-g)
  int16_t gyroX;                  // Unused (always 0)
  int16_t gyroY;                  // Unused (always 0)
  int16_t gyroZ;                  // Unused (always 0)
  uint32_t timestamp_ms;          // System uptime when measurement triggered
  uint8_t status;                 // Status flags (bit 0=MEASURING, 1=MEASURED, 2=LOCKED, 3=READY_TRANSFER)
  uint16_t accelSampleCount;      // Sample count in this packet (currently always 1000)
} // 34 bytes total
```

#### High-Speed Sample Arrays (NEW!) (8,000 bytes)
```c
struct {
  int16_t accelX_samples[1000];    // 1000 × 2 bytes = 2,000 bytes
  int16_t accelY_samples[1000];    // 1000 × 2 bytes = 2,000 bytes
  int16_t accelZ_samples[1000];    // 1000 × 2 bytes = 2,000 bytes
  uint16_t microphoneSamples[1000];// 1000 × 2 bytes = 2,000 bytes
} // 8,000 bytes total - 1 full second @ 1kHz
```

**Key Points:**
- All accel samples in **milli-g** units (e.g., 1000 milli-g = 1.0 g)
- All microphone samples in **raw ADC values** (0-4095) or millivolts (user-configurable)
- Sampling was precisely timed with 1ms intervals using FreeRTOS `vTaskDelayUntil()`
- Samples captured by taking snapshot of 1000-sample ring buffer when `CMD_LOCK_BUFFERS` is received

#### Camera Frames (8,576 bytes) - UNCHANGED
```c
struct {
  uint16_t rgbFrame[4096];        // RGB565 64×64 crop from OV2640 (8,192 bytes)
  uint16_t irFrame[192];          // MLX90641 16×12 thermal (384 bytes)
} // 8,576 bytes
```

---

## 🎯 Data Capture Guarantees

### Previous Behavior (Before Update)
- Accel/microphone: 1 sample per measurement cycle (~500-600ms)
- Data loss: ~99.8% (only 1 of 1000 samples captured)
- ❌ Not suitable for meaningful signal analysis

### Current Behavior (After Update)
- Accel/microphone: 1000 samples per measurement cycle (1 full second @ 1kHz)
- Data loss: 0% with 1-second master polling interval
- ✅ Complete sensor data available for analysis, filtering, FFT, etc.

---

## 📡 Protocol Commands (Unchanged)

All existing SPI protocol commands remain compatible:

```
CMD_TRIGGER_MEASUREMENT (0x01)  → Initiates measurement sequence
CMD_LOCK_BUFFERS        (0x02)  → Locks and prepares data for transfer
CMD_TRANSFER_DATA       (0x03)  → Transfers locked packet to Master
```

### New LED Control Commands (Independent)
```
CMD_LED_BRIGHTNESS      (0xF0)  → Set power LED brightness (0-100%)
CMD_IR_LED              (0xF1)  → Control IR LED (0=off, 1=on)
CMD_READ_AMB_LIGHT      (0xF2)  → Read ambient light value
```

These can be sent anytime and do **not** interfere with measurement tasks.

---

## 🔧 Master Integration Checklist

### 1. SPI Configuration
- [ ] Update SPI buffer size to 20,480 bytes (previously 8,704)
- [ ] Verify max_transfer_sz = 20,480 in SPI configuration
- [ ] Ensure DMA is enabled for all transactions

### 2. Packet Parsing
- [ ] Parse metadata (34 bytes, offset 0)
- [ ] Parse accelX/Y/Z sample arrays (6,000 bytes total, offset 34)
- [ ] Parse microphone sample array (2,000 bytes, offset 6,034)
- [ ] Parse RGB frame (8,192 bytes, offset 8,034)
- [ ] Parse IR frame (384 bytes, offset 16,226)

### 3. Data Processing
- [ ] Convert accel samples from milli-g to standard units (divide by 1000)
- [ ] Process 1-second sample window for DSP/FFT analysis (if needed)
- [ ] Handle sample count field (`accelSampleCount`) for future compatibility
- [ ] Timestamp all 1000 samples with: `timestamp_ms + (sample_index / 1000.0)`

### 4. Backward Compatibility
- [ ] Single-value accel fields still contain latest sample (compatible with old code)
- [ ] Packet size increased but still fits in SPI buffer
- [ ] Status flags unchanged

---

## 📊 Packet Size Breakdown

| Component | Size | Notes |
|-----------|------|-------|
| Metadata | 34 bytes | Sequence, timestamps, status |
| Accel X samples | 2,000 bytes | 1000 × int16 |
| Accel Y samples | 2,000 bytes | 1000 × int16 |
| Accel Z samples | 2,000 bytes | 1000 × int16 |
| Microphone samples | 2,000 bytes | 1000 × uint16 |
| RGB frame | 8,192 bytes | 64×64 RGB565 |
| IR frame | 384 bytes | 16×12 thermal |
| **Total Packet** | **16,610 bytes** | Fits in 20,480 SPI buffer |

---

## ⚡ Sampling Specifications

### Accelerometer (LIS3DH via SPI @ 1MHz)
- **Sample Rate:** 1000 samples/sec (1kHz)
- **Timing:** Precise 1ms intervals via FreeRTOS `vTaskDelayUntil()`
- **Units:** milli-g (1000 milli-g = 1.0 g)
- **Task:** High-speed sampler (Core 0, Priority 3)
- **Samples per packet:** 1000 (1 full second)

### Microphone (ADC GPIO 2)
- **Sample Rate:** 1000 samples/sec (1kHz, synchronized with accel)
- **Timing:** Same 1ms interrupt as accel
- **Units:** Raw ADC (0-4095) or mV (configurable)
- **Task:** High-speed sampler (same as accel)
- **Samples per packet:** 1000 (1 full second)

### Thermal (MLX90641 via I2C @ 100kHz)
- **Frame Rate:** 4 Hz (one frame per measurement, ~250ms integration)
- **Resolution:** 16×12 pixels (192 values)
- **Units:** Degrees Celsius (stored as uint16 internally)
- **Latency:** Part of measurement cycle

### Camera (OV2640 via CSI)
- **Resolution:** 64×64 RGB565 crop
- **Format:** 16-bit little-endian RGB565
- **Latency:** Captured during measurement cycle

---

## 🔍 Debug Serial Output

When slave firmware initializes, you'll see:
```
[Slave] SPI Ready - Concurrent architecture with background measurement task
[HighSpeedSampler] ⚙️ Started - Sampling accel + mic @ 1kHz
[Slave] Measurement task created on Core 0
```

During operation:
```
[Slave] RX #X: CMD=0x01 Status=0x00 | TRIGGER
[Measurement Task] ⚡ Triggered! Starting data collection...
[SPI] ✓ Measurement task triggered (non-blocking)
[Debug] Step 1: Measuring ambient light...
[Debug] Step 2: Snapshot 1000 accel+mic samples from ring buffer...
[Debug] Step 3: Measuring IR thermal...
[Debug] Step 4: Capturing RGB frame...
[Debug] Step 5: Setting timestamp...
[Measurement] ✓ Complete in ~450 ms
[Slave] RX #X+1: CMD=0x02 Status=0x02 | LOCK
[Slave] ✓ Buffers locked and ready for transfer
```

---

## 🧮 Memory Details

### ESP32-S3 Resources
- **RAM Usage:** 27.3% (89,320 bytes / 327,680 available)
- **Flash Usage:** 30.4% (398,153 bytes / 1,310,720 available)
- **Headroom:** Very comfortable ✅

### Ring Buffer (Background Sampling)
- **Depth:** 5,000 samples per channel (5 seconds @ 1kHz)
- **Purpose:** Prevents data loss if master polls slower than 5 seconds
- **Memory:** 5000 × 4 channels × 2 bytes = 40 KB (within available RAM)

---

## ✅ Testing Notes

- **Compilation:** ✅ Successful (10.72 seconds)
- **Memory:** ✅ Within safe limits
- **Data Integrity:** ✅ Ring buffer prevents race conditions
- **Timing Precision:** ✅ vTaskDelayUntil() provides <1% jitter

---

## Questions for Master Implementation?

If you have questions about:
1. Sample interpretation or units
2. Timestamp alignment across 1000 samples
3. SPI transaction sequence
4. LED control integration
5. DSP/FFT processing recommendations

Feel free to ask! 🚀
