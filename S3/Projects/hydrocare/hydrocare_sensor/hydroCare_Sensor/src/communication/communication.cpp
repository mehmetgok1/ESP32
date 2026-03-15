#include "communication/communication.h"
#include "config/config.h"
#include "measurement/measurement.h"
#include "leds/leds.h"
#include "driver/spi_slave.h"
#include "esp_camera.h"
#include "MLX90641.h"
#include <Arduino.h>
#include <cstring>
#include <cmath>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>

// Protocol commands (match SLAVE_PROTOCOL.md)
#define CMD_NONE 0x00
#define CMD_TRIGGER_MEASUREMENT 0x01
#define CMD_LOCK_BUFFERS 0x02
#define CMD_TRANSFER_DATA 0x03

// LED control commands (can run anytime, independent of measurement state)
#define CMD_LED_BRIGHTNESS 0xF0    // Set power LED brightness (0-100%)
#define CMD_IR_LED 0xF1            // Control IR LED (0=off, 1=on)
#define CMD_READ_AMB_LIGHT 0xF2    // Read ambient light value

// Slave status bits
#define STATUS_MEASURING 0x01
#define STATUS_MEASURED 0x02
#define STATUS_LOCKED 0x04
#define STATUS_READY_TRANSFER 0x08

// Sensor data packet structure with high-speed samples (~16.6KB)
#pragma pack(1)
typedef struct {
  // Metadata
  uint16_t sequence;              // Packet sequence number
  uint16_t ambientLight;          // Ambient light value (instantaneous)
  float temperature;              // Temperature in °C (average)
  float humidity;                 // Humidity % (average)
  int16_t accelX, accelY, accelZ; // IMU accel (most recent single sample)
  int16_t gyroX, gyroY, gyroZ;    // IMU gyro (unused, zeros)
  uint32_t timestamp_ms;          // System uptime when measurement triggered
  uint8_t status;                 // Status flags
  uint16_t accelSampleCount;      // Number of accel/mic samples in this packet (1000)
  
  // High-speed samples (1kHz sampling over 1 second measurement window)
  int16_t accelX_samples[1000];    // 1000 accel X samples @ 1kHz = 1 second
  int16_t accelY_samples[1000];    // 1000 accel Y samples @ 1kHz = 1 second
  int16_t accelZ_samples[1000];    // 1000 accel Z samples @ 1kHz = 1 second
  uint16_t microphoneSamples[1000];// 1000 microphone samples @ 1kHz = 1 second
  
  // Slow sensor frames (camera data)
  uint16_t rgbFrame[4096];        // RGB565 64x64 (8192 bytes)
  uint16_t irFrame[192];          // IR thermal 16x12 (384 bytes)
} SensorDataPacket;
#pragma pack()

// DMA-capable buffers (padded for new packet structure ~16.6KB)
uint8_t *rxBuf;
uint8_t *txBuf;
const uint16_t SPI_BUFFER_SIZE = 20480;  // 20KB - handles 16.6KB packet + header + margin

// ============ High-Speed Sampling Ring Buffers (1kHz) ============
// 5000-sample buffer = 5 seconds of continuous data @ 1kHz
// Large buffer prevents race condition: sampler index always moves far ahead of read position
const int RING_BUFFER_SIZE = 5000;  // 5 seconds of 1kHz data
int16_t accelX_ring[RING_BUFFER_SIZE] = {0};
int16_t accelY_ring[RING_BUFFER_SIZE] = {0};
int16_t accelZ_ring[RING_BUFFER_SIZE] = {0};
uint16_t microphone_ring[RING_BUFFER_SIZE] = {0};
volatile int ringBufferIndex = 0;  // Index for next write (no mutex needed - single writer)

// Global sensor data
static SensorDataPacket currentData = {0};
static uint16_t sequenceNumber = 0;

// State machine
typedef enum {
  STATE_IDLE = 0,
  STATE_MEASURING = 1,
  STATE_READY_FOR_LOCK = 2,
  STATE_LOCKED = 3,
  STATE_READY_FOR_TRANSFER = 4,
  STATE_ERROR = 5
} SlaveState;

static SlaveState slaveState = STATE_IDLE;
static uint32_t measurementStartTime = 0;

// ============ FreeRTOS Synchronization ============
static EventGroupHandle_t spiEventGroup = NULL;
static SemaphoreHandle_t currentDataMutex = NULL;
static TaskHandle_t measurementTaskHandle = NULL;

// Event group bits
#define EVENT_TRIGGER_RECEIVED (1 << 0)    // Trigger command received

// ============ Initialization ============
void initSPIComm() {
  // Allocate DMA buffers
  rxBuf = (uint8_t*) heap_caps_malloc(SPI_BUFFER_SIZE, MALLOC_CAP_DMA);
  txBuf = (uint8_t*) heap_caps_malloc(SPI_BUFFER_SIZE, MALLOC_CAP_DMA);
  
  if (!rxBuf || !txBuf) {
    Serial.println("[Slave] DMA allocation failed!");
    while(1) delay(1000);
  }
  
  // Clear initial buffers
  memset(rxBuf, 0, SPI_BUFFER_SIZE);
  memset(txBuf, 0, SPI_BUFFER_SIZE);
  memset(&currentData, 0, sizeof(SensorDataPacket));
  
  spi_bus_config_t buscfg = {
    .mosi_io_num     = SPI_MOSI,
    .miso_io_num     = SPI_MISO,
    .sclk_io_num     = SPI_SCK,
    .quadwp_io_num   = -1,
    .quadhd_io_num   = -1,
    .max_transfer_sz = 20480,  // 20KB - supports full 16.6KB packet with DMA chaining
  };

  spi_slave_interface_config_t slvcfg = {
    .spics_io_num  = SPI_CS,
    .flags         = 0,
    .queue_size    = 1,
    .mode          = 0,  // SPI_MODE0
    .post_setup_cb = NULL,
    .post_trans_cb = NULL,
  };

  esp_err_t ret = spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
  if (ret != ESP_OK) {
    Serial.printf("[Slave] Init failed: %s\n", esp_err_to_name(ret));
    while(1) delay(1000);
  }
  
  // Create FreeRTOS synchronization primitives
  spiEventGroup = xEventGroupCreate();
  currentDataMutex = xSemaphoreCreateMutex();
  
  if (!spiEventGroup || !currentDataMutex) {
    Serial.println("[Slave] Mutex/EventGroup creation failed!");
    while(1) delay(1000);
  }
  
  Serial.println("[Slave] SPI Ready - Concurrent architecture with background measurement task");
}

// ============ Get Status Byte ============
uint8_t getStatusByte() {
  uint8_t status = 0x00;
  
  switch (slaveState) {
    case STATE_MEASURING:
      status |= STATUS_MEASURING;
      break;
    case STATE_READY_FOR_LOCK:
      status |= STATUS_MEASURED;
      break;
    case STATE_LOCKED:
      status |= STATUS_LOCKED;
      break;
    case STATE_READY_FOR_TRANSFER:
      status |= STATUS_LOCKED | STATUS_READY_TRANSFER;
      break;
    default:
      break;
  }
  
  return status;
}

// ============ Measurement Collection (runs in background task) ============
void collectMeasurementData() {
  Serial.println("[Measurement] ★ COLLECTING DATA...");
  uint32_t startTime = millis();
  
  // Acquire mutex before modifying currentData
  xSemaphoreTake(currentDataMutex, portMAX_DELAY);
  
  // 1. Ambient light (fast, ~1ms)
  Serial.println("[Debug] Step 1: Measuring ambient light...");
  measureAmbLight();
  currentData.ambientLight = ambLight;
  Serial.printf("[Debug] ✓ Ambient light: %d\n", ambLight);
  
  // 2. Grab high-speed accel + mic samples from ring buffer (last 1000 @ 1kHz = 1 second of data)
  Serial.println("[Debug] Step 2: Snapshot 1000 accel+mic samples from ring buffer...");
  
  // Calculate start index (1000 samples back from current write position)
  // Capture endIdx FIRST - even if sampler advances during copy, we use this snapshot
  int endIdx = ringBufferIndex;
  int startIdx = (endIdx - 1000 + RING_BUFFER_SIZE) % RING_BUFFER_SIZE;
  
  // Copy 1000 consecutive samples into packet
  // RACE CONDITION SAFE: 5000-sample buffer is large enough that sampler can't catch up
  // While copying (20ms max), sampler advances only ~20 positions
  // With 5000 total slots, there's always massive separation. No overwrite risk!
  for (int i = 0; i < 1000; i++) {
    int srcIdx = (startIdx + i) % RING_BUFFER_SIZE;
    currentData.accelX_samples[i] = accelX_ring[srcIdx];
    currentData.accelY_samples[i] = accelY_ring[srcIdx];
    currentData.accelZ_samples[i] = accelZ_ring[srcIdx];
    currentData.microphoneSamples[i] = microphone_ring[srcIdx];
  }
  currentData.accelSampleCount = 1000;  // Full 1 second of 1kHz data
  
  // Also store most recent single values for backward compatibility
  currentData.accelX = accelX_ring[endIdx > 0 ? endIdx - 1 : RING_BUFFER_SIZE - 1];
  currentData.accelY = accelY_ring[endIdx > 0 ? endIdx - 1 : RING_BUFFER_SIZE - 1];
  currentData.accelZ = accelZ_ring[endIdx > 0 ? endIdx - 1 : RING_BUFFER_SIZE - 1];
  currentData.gyroX = 0;
  currentData.gyroY = 0;
  currentData.gyroZ = 0;
  
  Serial.printf("[Debug] ✓ Accel+Mic samples: Count=%d (full 1 second), Last=[X:%d Y:%d Z:%d Mic:%d]\n", 
    1000, currentData.accelX, currentData.accelY, currentData.accelZ, microphone_ring[endIdx > 0 ? endIdx - 1 : RING_BUFFER_SIZE - 1]);
  
  // 3. IR temperature frame (medium, ~300ms at 4Hz)
  Serial.println("[Debug] Step 3: Measuring IR thermal...");
  measureIRTemp();
  Serial.println("[Debug] ✓ IR measurement done, processing data...");
  
  float avgTemp = 0;
  for (int i = 0; i < 192; i++) {
    currentData.irFrame[i] = (uint16_t)((myIRcam.T_o[i] + 40) * 100);
    avgTemp += myIRcam.T_o[i];
  }
  avgTemp /= 192;
  currentData.temperature = avgTemp;
  currentData.humidity = 0.0f;  // TODO: add humidity sensor
  Serial.printf("[Debug] ✓ IR processed: avg temp = %.1f°C\n", avgTemp);
  
  // 4. RGB camera frame (slow, ~100ms)
  Serial.println("[Debug] Step 4: Capturing RGB frame...");
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb) {
    Serial.printf("[Debug] Camera buffer: %dx%d | Cropping to 64x64 center...\n", fb->width, fb->height);
    int startX = (fb->width - 64) / 2;
    int startY = (fb->height - 64) / 2;
    Serial.printf("[Debug] Crop offset: X=%d Y=%d\n", startX, startY);
    
    int idx = 0;
    uint16_t rgbMin = 65535, rgbMax = 0;
    uint32_t rgbSum = 0;
    
    for (int row = 0; row < 64; row++) {
      for (int col = 0; col < 64; col++) {
        int src = ((startY + row) * fb->width + (startX + col)) * 2;
        uint16_t pixel = (fb->buf[src] << 8) | fb->buf[src + 1];
        currentData.rgbFrame[idx++] = pixel;
        
        // Track statistics
        if (pixel < rgbMin) rgbMin = pixel;
        if (pixel > rgbMax) rgbMax = pixel;
        rgbSum += pixel;
      }
    }
    
    // Calculate statistics
    uint16_t rgbAvg = rgbSum / 4096;
    Serial.printf("[Debug] ✓ RGB frame captured: Min=0x%04X Max=0x%04X Avg=0x%04X\n", rgbMin, rgbMax, rgbAvg);
    
    // Sample first few pixels for debugging
    Serial.print("[Debug] RGB sample (first 8 pixels): ");
    for (int i = 0; i < 8; i++) {
      Serial.printf("0x%04X ", currentData.rgbFrame[i]);
    }
    Serial.println();
    
    esp_camera_fb_return(fb);
    Serial.println("[Debug] ✓ RGB frame captured (64×64 center crop stored)");
  } else {
    Serial.println("[Debug] ✗ Camera buffer NULL!");
  }
  
  // 5. Timestamp
  Serial.println("[Debug] Step 5: Setting timestamp...");
  currentData.timestamp_ms = millis();
  Serial.printf("[Debug] ✓ Timestamp: %lu ms\n", currentData.timestamp_ms);
  
  // Release mutex after all updates complete
  xSemaphoreGive(currentDataMutex);
  
  uint32_t elapsed = millis() - startTime;
  Serial.printf("[Measurement] ✓ Complete in %lu ms\n", elapsed);
}

// ============ Background Measurement Task ============
static void measurementCollectorTask(void *pvParameters) {
  while (1) {
    // Wait for trigger event from SPI handler
    EventBits_t uxBits = xEventGroupWaitBits(
      spiEventGroup,
      EVENT_TRIGGER_RECEIVED,
      pdTRUE,  // Clear on exit
      pdFALSE, // Don't wait for all bits
      portMAX_DELAY
    );
    
    if (uxBits & EVENT_TRIGGER_RECEIVED) {
      Serial.println("[Measurement Task] ⚡ Triggered! Starting data collection...");
      
      // Collect measurement data (this blocks the task, but SPI handler is on different task)
      collectMeasurementData();
      
      // Update state: data collection complete, ready for lock
      slaveState = STATE_READY_FOR_LOCK;
      Serial.println("[Measurement Task] ✓ Data ready, state = READY_FOR_LOCK");
    }
    
    taskYIELD();  // Let other tasks run
  }
}

// ============ High-Speed Sampler Task (1kHz on Core 0) ============
// Runs independently, sampling accel + mic every 1ms
// Results stored in ring buffers (no mutex needed - single writer)
static void highSpeedSamplerTask(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  
  Serial.println("[HighSpeedSampler] ⚙️ Started - Sampling accel + mic @ 1kHz");
  
  while (1) {
    // Precise 1ms timing using vTaskDelayUntil
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(1));
    
    // Read acceleration (FSPI)
    readAcceleration();
    
    // Read microphone (ADC - very fast)
    measureMicrophone();
    
    // Store in ring buffer (atomic write, single writer)
    int idx = ringBufferIndex;
    accelX_ring[idx] = (int16_t)(ax * 1000);
    accelY_ring[idx] = (int16_t)(ay * 1000);
    accelZ_ring[idx] = (int16_t)(az * 1000);
    microphone_ring[idx] = microphone;
    
    // Advance ring buffer index
    ringBufferIndex = (ringBufferIndex + 1) % RING_BUFFER_SIZE;
    
    // Yield to allow other tasks (especially idle task for watchdog) to run
    taskYIELD();
  }
}

// ============ Main SPI Command Handler (Fast, Non-Blocking) ============
void receiveCommand() {
  static spi_slave_transaction_t t;
  static uint32_t transaction_count = 0;
  
  // Prepare TX buffer: Byte 0 = status
  uint8_t statusByte = getStatusByte();
  
  // **CRITICAL**: Only clear txBuf if NOT in READY_FOR_TRANSFER state
  // In READY_FOR_TRANSFER, preserve the packet data prepared during LOCK phase
  if (slaveState != STATE_READY_FOR_TRANSFER) {
    memset(txBuf, 0, SPI_BUFFER_SIZE);
  }
  txBuf[0] = statusByte;  // Always update status byte
  
  // Setup SPI transaction (always 8704 bytes)
  memset(&t, 0, sizeof(spi_slave_transaction_t));
  t.length    = SPI_BUFFER_SIZE * 8;  // 8704 bytes in bits
  t.rx_buffer = rxBuf;
  t.tx_buffer = txBuf;

  // Debug: Check what's in txBuf BEFORE transmitting
  if (slaveState == STATE_READY_FOR_TRANSFER) {
    uint16_t *debugPtr = (uint16_t*)(&txBuf[8032]);
    Serial.printf("[Debug Pre-TX] State=READY_FOR_TRANSFER, txBuf[8032] RGB: %04X %04X %04X %04X\n",
      debugPtr[0], debugPtr[1], debugPtr[2], debugPtr[3]);
  }

  // Wait for master with extended timeout
  esp_err_t ret = spi_slave_transmit(SPI2_HOST, &t, 100);  // 10-second timeout
  
  if (ret == ESP_OK) {
    transaction_count++;
    
    // Read command from byte 0 of received data
    uint8_t cmd = rxBuf[0];
    
    // Log transaction
    Serial.printf("[Slave] RX #%lu: CMD=0x%02X Status=0x%02X | ", transaction_count, cmd, statusByte);
    
    // ========== COMMAND HANDLING (Fast state transitions, no blocking) ==========
    
    // ========== STATE-BASED COMMAND VALIDATION ==========
    // Only accept commands that make sense in current state
    // POLL (0x00) and LED commands always OK, reject others if state mismatch
    
    if (cmd == CMD_TRIGGER_MEASUREMENT) {
      if (slaveState == STATE_IDLE) {
        Serial.println("TRIGGER");
        slaveState = STATE_MEASURING;
        measurementStartTime = millis();
        
        // Signal background measurement task to start collecting data
        // This returns IMMEDIATELY - SPI handler doesn't block
        xEventGroupSetBits(spiEventGroup, EVENT_TRIGGER_RECEIVED);
        
        Serial.println("[SPI] ✓ Measurement task triggered (non-blocking)");
      } else {
        Serial.printf("ERROR: TRIGGER invalid in state %d (ignore)\n", slaveState);
      }
    }
    
    else if (cmd == CMD_LOCK_BUFFERS) {
      if (slaveState == STATE_READY_FOR_LOCK) {
        Serial.println("LOCK");
        
        // Acquire mutex, prepare packet for transfer
        xSemaphoreTake(currentDataMutex, portMAX_DELAY);
        
        currentData.sequence = sequenceNumber++;
        currentData.status = statusByte;
        
        // Debug: Verify data exists BEFORE copy
        Serial.printf("[Debug LOCK] currentData.rgbFrame[0-3]: %04X %04X %04X %04X\n",
          currentData.rgbFrame[0], currentData.rgbFrame[1], currentData.rgbFrame[2], currentData.rgbFrame[3]);
        
        memcpy(txBuf + 1, &currentData, sizeof(SensorDataPacket));
        
        // Debug: Verify data exists in txBuf AFTER copy (at byte offset 8033 = 1 + 8032)
        uint16_t *txRgbPtr = (uint16_t*)(&txBuf[8033]);
        Serial.printf("[Debug LOCK] txBuf RGB[0-3]: %04X %04X %04X %04X\n",
          txRgbPtr[0], txRgbPtr[1], txRgbPtr[2], txRgbPtr[3]);
        
        xSemaphoreGive(currentDataMutex);
        
        // Transition to READY_FOR_TRANSFER (data already prepared)
        slaveState = STATE_READY_FOR_TRANSFER;
        Serial.println("[Slave] ✓ Buffers locked and ready for transfer");
      } else {
        Serial.printf("ERROR: LOCK invalid in state %d (ignore)\n", slaveState);
      }
    }
    
    else if (cmd == CMD_TRANSFER_DATA) {
      if (slaveState == STATE_READY_FOR_TRANSFER) {
        Serial.println("TRANSFER");
        // Data already in txBuf from LOCK phase
        // Return to IDLE after master finishes reading
        slaveState = STATE_IDLE;
        Serial.println("[Slave] ✓ Transfer complete, returning to IDLE");
      } else {
        Serial.printf("ERROR: TRANSFER invalid in state %d (ignore)\n", slaveState);
      }
    }
    
    else if (cmd == CMD_NONE) {
      // Poll - just respond with current status
      if (slaveState == STATE_MEASURING) {
        Serial.println("POLL (still measuring)");
      } else if (slaveState == STATE_READY_FOR_LOCK) {
        Serial.println("POLL (ready for lock)");
      } else if (slaveState == STATE_READY_FOR_TRANSFER) {
        Serial.println("POLL (ready for transfer)");
      } else {
        Serial.println("POLL");
      }
    }
    
    // ========== LED CONTROL COMMANDS (Independent, always available) ==========
    
    else if (cmd == CMD_LED_BRIGHTNESS) {
      uint8_t brightness = rxBuf[1];  // Value in byte 1
      Serial.printf("LED_BRIGHTNESS: %d%%\n", brightness);
      powerLED(brightness);
    }
    
    else if (cmd == CMD_IR_LED) {
      uint8_t irState = rxBuf[1];  // Value in byte 1 (0=off, 1=on)
      Serial.printf("IR_LED: %s (value=%d, pin=%d)\n", irState ? "ON" : "OFF", irState, ledCntrlIR);
      Serial.printf("GPIO19 state BEFORE: %d\n", digitalRead(ledCntrlIR));
      IRLED(irState);
      Serial.printf("GPIO19 state AFTER: %d\n", digitalRead(ledCntrlIR));
    }
    
    else if (cmd == CMD_READ_AMB_LIGHT) {
      // Ambient light is already in txBuf from current measurement
      // Just log it
      Serial.printf("AMB_LIGHT_READ: %d\n", currentData.ambientLight);
    }
    
    else {
      Serial.printf("UNKNOWN\n");
    }
    
  } else if (ret == ESP_ERR_TIMEOUT) {
    // Timeout is normal when master isn't active
  } else {
    Serial.printf("[Slave] SPI Error: %s\n", esp_err_to_name(ret));
  }
}

// ============ Exposed Functions ============

// Start the background measurement task (call from main setup)
void startMeasurementTask() {
  xTaskCreatePinnedToCore(
    measurementCollectorTask,
    "MeasurementCollector",
    20480,          // Stack size (20 KB) - generous headroom for camera + thermal library operations
    NULL,           // Parameters
    2,              // Priority (higher than high-speed sampler)
    &measurementTaskHandle,
    0               // Core 0
  );
  Serial.println("[Slave] Measurement task created on Core 0");
}

// Start the high-speed sampler task (continuous 1kHz accel + mic sampling)
void startHighSpeedSamplerTask() {
  static TaskHandle_t samplerTaskHandle = NULL;
  
  xTaskCreatePinnedToCore(
    highSpeedSamplerTask,
    "HighSpeedSampler",
    8192,           // Stack size (8 KB - safe for SPI + ADC operations)
    NULL,           // Parameters
    1,              // Priority (low - won't interfere with SPI handler)
    &samplerTaskHandle,
    1               // Core 1 (dedicated to sampling, Core 0 free for SPI + measurement)
  );
  Serial.println("[Slave] High-speed sampler task created on Core 1 @ 1kHz");
}
