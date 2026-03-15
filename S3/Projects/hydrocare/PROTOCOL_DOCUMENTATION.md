# HydroCare Master-Slave SPI Protocol Implementation

## Overview
Complete redesign of SPI communication between Master (hydrocare_fw) and Slave (hydrocare_sensor) using a 3-step measurement cycle with double-buffering and FreeRTOS multi-tasking.

---

## Architecture Summary

### SLAVE (hydrocare_sensor) - Event-Driven Collection with Prefilled TX
- **2 FreeRTOS Tasks on separate cores:**
  - SPI Task (Core 1): Continuously blocks on SPI waiting for master commands
  - Measurement Task (Core 0): Event-driven, collects data when triggered
- **TX Buffer Prefilling:** After LOCK_BUFFERS command, txBuf is populated with sensor data
- **State Machine:** IDLE → MEASURING → MEASURED → (LOCK) → READY_TRANSFER → IDLE
- **SPI Buffer Size:** 20KB (handles ~16.6KB sensor packet + overhead)

### MASTER (hydrocare_fw) - Synchronized 3-Step Protocol
- **3-Step Measurement Cycle:**
  1. `Send TRIGGER_MEASUREMENT (0x01)` - Start background collection
  2. `Poll STATUS until MEASURED` - Wait for measurement complete
  3. `Send LOCK_BUFFERS (0x02)` - Freeze buffers, prefill slave TX with data
  4. `Read SENSOR_DATA (bulk 20KB)` - Grab preloaded sensor packet
- **Data Struct:** SensorDataPacket (~16.6KB) containing accel time-series, IR frame, RGB frame, metadata
- **Integration:** Called from loop() whenever sensor data needed

---

## Protocol Command Set

| Command | Code | Direction | Purpose |
|---------|------|-----------|---------|
| TRIGGER_MEASUREMENT | 0x01 | Master→Slave | Start background data collection task |
| LOCK_BUFFERS | 0x02 | Master→Slave | Freeze buffers, prefill TX with sensor data |
| LED_CONTROL | 0x10 | Master→Slave | Legacy LED brightness |
| IR_CONTROL | 0x11 | Master→Slave | Legacy IR LED control |
| ACK | 0xAA | Slave→Master | Acknowledge |
| ERROR | 0xFF | Slave→Master | Error status |

---

## Status Byte Bits (Slave → Master)

```
Bit 0: MEASURING (0x01) - Slave actively collecting data
Bit 1: MEASURED (0x02) - Data collection complete (awaiting LOCK)
Bit 3: LOCKED (0x04) - Buffers locked, TX prefilled with sensor data
Bit 4: READY_TRANSFER (0x08) - Ready for bulk read
```

---

## Data Validity Mask (Slave → Master)

```
Bit 0: DATA_VALID_IMU    (0x01) - IMU data present
Bit 1: DATA_VALID_LIGHT  (0x02) - Ambient light valid
Bit 2: DATA_VALID_IR     (0x04) - IR thermal frame valid
Bit 3: DATA_VALID_RGB    (0x08) - RGB camera frame valid
```

---

## Timing

### Typical Measurement Cycle with Prefilled TX:
```
T=0ms:      Master: Send TRIGGER_MEASUREMENT (0x01)
            Slave: STATE_IDLE → STATE_MEASURING
            Background task starts collecting data

T=0-1000ms: Slave: Collect all sensors (1 second of data)
            - Ambient light: ~1ms
            - Accel+Mic samples: 1000 @ 1kHz = 1 second
            - IR thermal: ~300ms
            - RGB camera: ~100ms
            
T=1000ms:   Slave: Measurement complete
            Status: MEASURING | MEASURED

T=1010ms:   Master: Poll STATUS, sees MEASURED bit set
            
T=1020ms:   Master: Send LOCK_BUFFERS (0x02)
            Slave: Acquires mutex, copies full SensorDataPacket to txBuf
            Status: MEASURED | LOCKED | READY_TRANSFER
            (txBuf is now prefilled with 20KB sensor data)

T=1030ms:   Master: Read SENSOR_DATA (bulk 20KB read)
            Slave: Transmits prefilled txBuf (no copy needed)
            Status: LOCKED
            After transfer complete: STATE_IDLE

T=1200ms:   Complete, ready for next cycle
```

**Key Advantage:** TX buffer is prefilled after LOCK command, so bulk read is immediate without additional copying overhead.

---

## Memory Layout

### SlaveData Structure (~9.6 KB total):
```c
typedef struct {
  float ax, ay, az;           // 12 bytes - IMU acceleration
  uint16_t ambLight;          // 2 bytes  - Ambient light reading
  uint16_t irFrame[192];      // 384 bytes - IR thermal (16×12 pixels)
  uint16_t rgbFrame[4096];    // 8192 bytes - RGB camera (64×64 pixels)
  uint32_t timestamp_ms;      // 4 bytes  - Timestamp
  uint8_t dataValidMask;      // 1 byte   - Which sensors are valid
} SlaveData;                  // TOTAL: ~9600 bytes
```

---

## File Changes

### SLAVE (hydrocare_sensor)
✅ **Created:**
- `src/communication/dataBuffer.h/.cpp` - Double-buffering system
- `src/communication/protocolState.h/.cpp` - State machine

✅ **Modified:**
- `src/communication/communication.h/.cpp` - New protocol (256-byte transactions)
- `src/measurement/measurement.h/.cpp` - Added collection task
- `src/main.cpp` - FreeRTOS dual-task setup

### MASTER (hydrocare_fw)
✅ **Modified:**
- `src/communication/communication.h` - New data struct and commands
- `src/communication/communication.cpp` - 3-step cycle functions
- `src/main.cpp` - Call cycle in loop()

---

## Usage Example (Master)

```cpp
// In main loop when ready to collect sensor data:
if (timerStream == 1 && deviceConnected) {
    
    // STEP 1: Trigger measurement on slave
    uint8_t cmd = 0x01;  // ADDR_CTRL (write) | CTRL_TRIGGER_MEASUREMENT
    masterSPI.writeByte(ADDR_CTRL, CTRL_TRIGGER_MEASUREMENT);
    Serial.println("[Master] Triggered measurement");
    
    // STEP 2: Poll until measurement complete
    uint32_t timeout = millis() + 2000;
    uint8_t status = 0;
    while (millis() < timeout) {
        status = masterSPI.readByte(ADDR_STATUS);
        if (status & 0x02) {  // MEASURED bit set?
            Serial.println("[Master] Measurement complete!");
            break;
        }
        delay(10);
    }
    
    // STEP 3: Lock buffers and prefill slave TX
    masterSPI.writeByte(ADDR_CTRL, CTRL_LOCK_BUFFERS);
    Serial.println("[Master] Buffers locked, TX prefilled");
    
    // STEP 4: Read sensor data (already in slave's TX buffer)
    SensorDataPacket sensorData = {};
    masterSPI.readBulk(ADDR_SENSOR_DATA, &sensorData, sizeof(SensorDataPacket));
    
    // Now use the data
    float accelX_avg = (float)(sensorData.accelX_samples[0]) / 1000.0;
    uint16_t light = sensorData.ambientLight;
    float temp = sensorData.temperature;
    Serial.printf("[Master] Light=%d Temp=%.1f°C\n", light, temp);
}
```

---

## Testing Checklist

- [ ] Slave compiles without errors
- [ ] Master compiles without errors
- [ ] Both boards boot successfully
- [ ] Slave FreeRTOS tasks run correctly (check serial output)
- [ ] Master triggers measurement (watch for "[Master] === SENSOR SYNC CYCLE ===")
- [ ] Slave transitions through all states
- [ ] Data is received and parsed correctly
- [ ] Verify no data corruption between master and slave
- [ ] Check timing: should complete cycle in ~1.5 seconds
- [ ] Verify all sensors report data validity bits
- [ ] Test with camera frames enabled (currently disabled for testing)

---

## Optimization Notes

1. **Camera frames** are currently commented out in slave. Uncomment when ready:
   ```cpp
   // measureCamera();
   // g_dataBuffer->updateRGBFrame(rgbData);
   ```

2. **IR thermal** is commented out. Needs proper type conversion from MLX90641 format.

3. **FreeRTOS stack sizes** may need tuning based on memory pressure:
   - SPI Task: 4096 bytes
   - Measurement Task: 8192 bytes

4. **SPI Clock** is 16 MHz. Can increase if timing allows.

---

## Backwards Compatibility

- Legacy functions still work: `sendBrightness()`, `sendIRLED()`
- `readAmbientLight_Int()` now reads from cached slave data instead of polling
- No breaking changes to existing code, just add new protocol calls

---

## Future Enhancements

1. Add CRC/checksum to data frames for integrity checking
2. Implement timeout watchdog on slave
3. Add data logging on slave for debugging
4. Optimize camera capture timing
5. Add error recovery (retry mechanism for failed transfers)
6. Consider using interrupt for DATA_READY signal (if GPIO available)

