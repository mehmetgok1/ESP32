# HydroCare Master-Slave SPI Protocol Implementation

## Overview
Complete redesign of SPI communication between Master (hydrocare_fw) and Slave (hydrocare_sensor) using a 3-step measurement cycle with double-buffering and FreeRTOS multi-tasking.

---

## Architecture Summary

### SLAVE (hydrocare_sensor) - Event-Driven Collection
- **2 FreeRTOS Tasks on separate cores:**
  - SPI Task (Core 1): Continuously blocks on SPI waiting for master commands
  - Measurement Task (Core 0): Event-driven, collects data when triggered
- **3-Level Buffering:** Live → Staging → Ready (prevents corruption)
- **State Machine:** IDLE → MEASURING → READY → TRANSFERRING → IDLE
- **SPI Buffer Size:** 256 bytes (was 3 bytes)

### MASTER (hydrocare_fw) - Synchronized Polling
- **3-Step Measurement Cycle:**
  1. `triggerMeasurementCycle()` - Send TRIGGER_MEASUREMENT (0x01)
  2. `waitForSlaveReady()` - Poll until slave reports READY (0x02 status)
  3. `readSensorDataBurst()` - Burst-read all sensor data in 256-byte chunks
- **Data Struct:** SlaveData containing IMU, ambient light, IR frame, RGB camera frame
- **Integration:** Called from loop() whenever `timerStream == 1`

---

## Protocol Command Set

| Command | Code | Direction | Purpose |
|---------|------|-----------|---------|
| TRIGGER_MEASUREMENT | 0x01 | Master→Slave | Start data collection |
| LOCK_BUFFERS | 0x02 | Master→Slave | Freeze buffers for reading |
| LED_CONTROL | 0x10 | Master→Slave | Legacy LED brightness |
| IR_CONTROL | 0x11 | Master→Slave | Legacy IR LED control |
| ACK | 0xAA | Slave→Master | Acknowledge |
| ERROR | 0xFF | Slave→Master | Error status |

---

## Status Byte Bits (Slave → Master)

```
Bit 0: MEASURING (0x01) - Slave actively collecting data
Bit 1: READY    (0x02) - Data ready for transfer
Bit 2: TRANSFERRING (0x04) - Transfer in progress
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

### Typical Measurement Cycle:
```
T=0ms:      Master: triggerMeasurementCycle()
            Slave: IDLE → MEASURING

T=0-1000ms: Slave: Collect all sensors
            - Ambient light: ~1ms
            - IMU: ~5ms
            - IR thermal: ~300ms
            - RGB camera: ~100ms
            
T=1000ms:   Slave: Data commit
            MEASURING → READY (buffers locked)

T=1050ms:   Master: waitForSlaveReady() detects READY
            
T=1060ms:   Master: readSensorDataBurst()
            - Chunk 1 (256 bytes)
            - Chunk 2 (256 bytes)
            - Etc. until complete

T=1200ms:   Complete, return to IDLE
```

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
// In main loop when time to collect:
if (timerStream == 1 && deviceConnected) {
    
    // Step 1: Tell slave to collect
    triggerMeasurementCycle();
    
    // Step 2: Wait for slave to finish (up to 2 seconds)
    if (waitForSlaveReady(2000)) {
        
        // Step 3: Read all data
        SlaveData sensorData = {};
        if (readSensorDataBurst(&sensorData)) {
            
            // Use the data
            float tem_x = sensorData.ax;
            uint16_t light = sensorData.ambLight;
            // ... process frames ...
            
            // Log with master's own sensors
            String dataRow = String(millis()) + "," + 
                           String(light) + "," + 
                           String(tem_x) + " ...";
            logData(dataRow);
        }
    }
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

