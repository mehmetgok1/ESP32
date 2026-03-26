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

// DMA-capable buffers
uint8_t *rxBuf;
uint8_t *txBuf;

// Global SPI transaction (initialized once, reused for all transactions)
static spi_slave_transaction_t slaveSpiTransaction = {};
static uint32_t transaction_count = 0;

// ============ High-Speed Sampling Ring Buffers (2kHz) ============
// 5000-sample buffer = 2.5 seconds of continuous data @ 2kHz
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

// State machine - simplified for new protocol
typedef enum {
  STATE_IDLE = 0,           // No measurement active
  STATE_MEASURING = 1,      // Measurement in progress
  STATE_READY_TRANSFER = 2, // Measurement complete, buffers locked, ready to send
  STATE_ERROR = 3
} SlaveState;

static SlaveState slaveState = STATE_IDLE;
static uint32_t measurementStartTime = 0;

// ============ FreeRTOS Synchronization ============
static EventGroupHandle_t spiEventGroup = NULL;
static SemaphoreHandle_t currentDataMutex = NULL;
static TaskHandle_t measurementTaskHandle = NULL;

// Event group bits
#define EVENT_TRIGGER_RECEIVED (1 << 0)    // Trigger command received

// ============ Cached Sensor Data (Double Buffering) ============
// IR cache structure
struct IRCache {
  uint16_t irFrame[192];
  float avgTemp;
  float Ta;
  uint32_t timestamp;
};

// BME cache structure
struct BMECache {
  float temp;
  float humidity;
  float pressure;
  float gas;
  uint32_t timestamp;
};

static IRCache irCache[2] = {0};          // Double buffer (avoid race conditions while reading)
static BMECache bmeCache[2] = {0};        // Double buffer
static volatile int irWriteIdx = 0;       // Background task writes to this index
static volatile int bmeWriteIdx = 0;      // Background task writes to this index
static SemaphoreHandle_t irCacheMutex = NULL;
static SemaphoreHandle_t bmeCacheMutex = NULL;
static TaskHandle_t irTaskHandle = NULL;
static TaskHandle_t bmeTaskHandle = NULL;

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
    .max_transfer_sz = SPI_BUFFER_SIZE,  // Supports full sensor packet with DMA
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
  irCacheMutex = xSemaphoreCreateMutex();
  bmeCacheMutex = xSemaphoreCreateMutex();
  
  if (!spiEventGroup || !currentDataMutex || !irCacheMutex || !bmeCacheMutex) {
    Serial.println("[Slave] Mutex/EventGroup creation failed!");
    while(1) delay(1000);
  }
  
  // Initialize global SPI transaction structure (done once, reused for all transactions)
  memset(&slaveSpiTransaction, 0, sizeof(spi_slave_transaction_t));
  slaveSpiTransaction.length    = SPI_BUFFER_SIZE * 8;
  slaveSpiTransaction.rx_buffer = rxBuf;
  slaveSpiTransaction.tx_buffer = txBuf;
  
  // Pre-fill permanent parts of txBuf
  txBuf[0] = 0x00;  // Dummy byte (always 0x00)
  txBuf[1] = 0x00;  // Status byte (updated on state changes)
  
  Serial.println("[Slave] SPI Ready - Concurrent architecture with background measurement task");
}

// ============ Get Status Byte ============
uint8_t getStatusByte() {
  uint8_t status = 0x00;
  
  if (slaveState == STATE_MEASURING) {
    status = STATUS_MEASURING;
  } 
  else if (slaveState == STATE_READY_TRANSFER) {
    // Buffers locked and ready for read
    status = STATUS_LOCKED;
  }
  
  return status;
}

// ============ Measurement Collection (runs in background task) ============
void collectMeasurementData() {
  Serial.println("[Measurement] ★ COLLECTING DATA...");
  uint32_t totalStartTime = millis();
  uint32_t stepTime = 0;
  
  // Acquire mutex before modifying currentData
  xSemaphoreTake(currentDataMutex, portMAX_DELAY);
  
  // 1. Ambient light (fast, ~1ms)
  Serial.println("[Debug] Step 1: Measuring ambient light...");
  stepTime = millis();
  measureAmbLight();
  currentData.ambientLight = ambLight;
  Serial.printf("[Debug] ✓ Ambient light: %d | Time: %lu ms\n", ambLight, millis() - stepTime);
  
  // 2. Grab high-speed accel + mic samples from ring buffer (last 2000 @ 2kHz = 1 second of data)
  Serial.println("[Debug] Step 2: Snapshot 2000 accel+mic samples from ring buffer...");
  stepTime = millis();
  
  // Calculate start index (2000 samples back from current write position)
  // Capture endIdx FIRST - even if sampler advances during copy, we use this snapshot
  int endIdx = ringBufferIndex;
  int startIdx = (endIdx - 2000 + RING_BUFFER_SIZE) % RING_BUFFER_SIZE;
  
  // Copy 2000 consecutive samples into packet
  // RACE CONDITION SAFE: 5000-sample buffer is large enough that sampler can't catch up
  // While copying (40ms max), sampler advances only ~80 positions (2000 samples = 1 second at 2kHz)
  // With 5000 total slots, there's always massive separation. No overwrite risk!
  for (int i = 0; i < 2000; i++) {
    int srcIdx = (startIdx + i) % RING_BUFFER_SIZE;
    currentData.accelX_samples[i] = accelX_ring[srcIdx];
    currentData.accelY_samples[i] = accelY_ring[srcIdx];
    currentData.accelZ_samples[i] = accelZ_ring[srcIdx];
    currentData.microphoneSamples[i] = microphone_ring[srcIdx];
  }
  currentData.accelSampleCount = 2000;  // Full 1 second of 2kHz data
  
  // Also store most recent single values for backward compatibility
  currentData.accelX = accelX_ring[endIdx > 0 ? endIdx - 1 : RING_BUFFER_SIZE - 1];
  currentData.accelY = accelY_ring[endIdx > 0 ? endIdx - 1 : RING_BUFFER_SIZE - 1];
  currentData.accelZ = accelZ_ring[endIdx > 0 ? endIdx - 1 : RING_BUFFER_SIZE - 1];
  currentData.gyroX = 0;
  currentData.gyroY = 0;
  currentData.gyroZ = 0;
  
  Serial.printf("[Debug] ✓ Accel+Mic samples: Count=%d (full 1 second @ 2kHz), Last=[X:%d Y:%d Z:%d Mic:%d] | Time: %lu ms\n", 
    2000, currentData.accelX, currentData.accelY, currentData.accelZ, microphone_ring[endIdx > 0 ? endIdx - 1 : RING_BUFFER_SIZE - 1], millis() - stepTime);
  
  // 3. IR temperature frame - GRAB FROM CACHE (background task updates every 200ms)
  Serial.println("[Debug] Step 3: Grabbing cached IR thermal...");
  stepTime = millis();
  
  xSemaphoreTake(irCacheMutex, portMAX_DELAY);
  int irReadIdx = 1 - irWriteIdx;  // Read from buffer NOT being written to
  for (int i = 0; i < 192; i++) {
    currentData.irFrame[i] = irCache[irReadIdx].irFrame[i];
  }
  currentData.temperature = irCache[irReadIdx].avgTemp;
  xSemaphoreGive(irCacheMutex);
  
  Serial.printf("[Debug] ✓ IR cached: avg temp = %.1f°C | Time: %lu ms\n", currentData.temperature, millis() - stepTime);
  
  // 3.5 BME688 Environment Data - GRAB FROM CACHE (background task updates every 200ms)
  Serial.println("[Debug] Step 3.5: Grabbing cached BME688...");
  stepTime = millis();
  
  xSemaphoreTake(bmeCacheMutex, portMAX_DELAY);
  int bmeReadIdx = 1 - bmeWriteIdx;  // Read from buffer NOT being written to
  currentData.humidity = bmeCache[bmeReadIdx].humidity;
  xSemaphoreGive(bmeCacheMutex);
  
  Serial.printf("[Debug] ✓ BME688 cached | Time: %lu ms\n", millis() - stepTime);

  // 4. RGB camera frame (slow, ~100ms)
  Serial.println("[Debug] Step 4: Capturing RGB frame...");
  stepTime = millis();
  
  // DEBUG MODE: Fill with 0xAAAA pattern
  if (0) {
    for (int i = 0; i < 4096; i++) {
      currentData.rgbFrame[i] = 0xAAAA;
    }
    Serial.println("[Debug] ✓ RGB frame filled with 0xAAAA (DEBUG MODE)");
  }
  else {
    // PRODUCTION: Actual camera capture (will restore later)
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
  }
  Serial.printf("[Debug] ✓ RGB frame complete | Time: %lu ms\n", millis() - stepTime);
  
  // 5. Timestamp
  Serial.println("[Debug] Step 5: Setting timestamp...");
  stepTime = millis();
  currentData.timestamp_ms = millis();
  Serial.printf("[Debug] ✓ Timestamp: %lu ms | Time: %lu ms\n", currentData.timestamp_ms, millis() - stepTime);
  
  // Release mutex after all updates complete
  xSemaphoreGive(currentDataMutex);
  
  uint32_t elapsed = millis() - totalStartTime;
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
      
      // Collect measurement data (this blocks the task, but SPI handler is on different core)
      collectMeasurementData();
      
      // Transition state: measurement complete, ready for LOCK
      // Update txBuf status to indicate measurement is done
      txBuf[1] = STATUS_MEASURED;
      // Don't change state here - master will see STATUS_MEASURED and send LOCK
      // which will transition to STATE_READY_TRANSFER
      
      Serial.println("[Measurement Task] ✓ Data ready, awaiting LOCK command from master");
    }
    
    taskYIELD();  // Let other tasks run
  }
}

// ============ IR Sensor Background Task (Every 200ms) ============
// Continuously reads IR sensor and caches result in double buffer
static void irSensorBackgroundTask(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  
  while (1) {
    // Precise 200ms timing
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(200));
    
    // Read IR sensor (blocking ~134ms)
    measureIRTemp();
    
    // Calculate average temperature
    float avgTemp = 0;
    for (int i = 0; i < 192; i++) {
      avgTemp += myIRcam.T_o[i];
    }
    avgTemp /= 192;
    
    // Write to cache with mutex protection
    xSemaphoreTake(irCacheMutex, portMAX_DELAY);
    int writeIdx = irWriteIdx;
    
    // Convert temperatures to fixed-point format (+ 40 offset, *100 scale)
    for (int i = 0; i < 192; i++) {
      irCache[writeIdx].irFrame[i] = (uint16_t)((myIRcam.T_o[i] + 40) * 100);
    }
    irCache[writeIdx].avgTemp = avgTemp;
    irCache[writeIdx].Ta = myIRcam.Ta;
    irCache[writeIdx].timestamp = millis();
    
    // Swap write index for next iteration (double buffering)
    irWriteIdx = 1 - irWriteIdx;
    
    xSemaphoreGive(irCacheMutex);
    
    taskYIELD();
  }
}

// ============ BME Sensor Background Task (Every 200ms) ============
// Continuously reads BME688 sensor and caches result in double buffer
static void bmeSensorBackgroundTask(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  
  while (1) {
    // Precise 200ms timing
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(200));
    
    // Read BME sensor (blocking ~123ms with optimized settings)
    measureBME688();
    
    // Write to cache with mutex protection
    xSemaphoreTake(bmeCacheMutex, portMAX_DELAY);
    int writeIdx = bmeWriteIdx;
    
    bmeCache[writeIdx].temp = bme_temp;
    bmeCache[writeIdx].humidity = bme_hum;
    bmeCache[writeIdx].pressure = bme_pres;
    bmeCache[writeIdx].gas = bme_gas;
    bmeCache[writeIdx].timestamp = millis();
    
    // Swap write index for next iteration (double buffering)
    bmeWriteIdx = 1 - bmeWriteIdx;
    
    xSemaphoreGive(bmeCacheMutex);
    
    taskYIELD();
  }
}

// ============ High-Speed Sampler Task (2kHz on Core 1) ============
// Precise microsecond-level sampling for DFT analysis
// 2000 samples/second = 500µs intervals
// Results stored in ring buffers (no mutex needed - single writer)
static void highSpeedSamplerTask(void *pvParameters) {
  int64_t lastSampleTime_us = 0;
  int64_t targetInterval_us = 500;  // 2kHz = one sample every 500 microseconds
  
  Serial.println("[HighSpeedSampler] ⚙️ Started - Sampling accel + mic @ 2kHz (precise microsecond timing)");
  
  // Initialize reference time
  lastSampleTime_us = esp_timer_get_time();
  
  while (1) {
    // Get current time
    int64_t now_us = esp_timer_get_time();
    
    // Check if time for next sample (500µs interval)
    if ((now_us - lastSampleTime_us) >= targetInterval_us) {
      lastSampleTime_us = now_us;
      
      // Read acceleration (FSPI - ~200µs)
      readAcceleration();
      
      // Read microphone (ADC - very fast ~10µs)
      measureMicrophone();
      
      // Store in ring buffer (atomic write, single writer)
      int idx = ringBufferIndex;
      accelX_ring[idx] = (int16_t)(ax * 1000);
      accelY_ring[idx] = (int16_t)(ay * 1000);
      accelZ_ring[idx] = (int16_t)(az * 1000);
      microphone_ring[idx] = microphone;
      
      // Advance ring buffer index
      ringBufferIndex = (ringBufferIndex + 1) % RING_BUFFER_SIZE;
    }
    
    // Yield to allow other tasks to run (watchdog, SPI handler, etc)
    // This prevents starving the whole system
    taskYIELD();
  }
}

// ============ Main SPI Command Handler - Address-Based Protocol ============
// Called once per transaction. txBuf pre-filled globally on state changes
void receiveCommand() {
  // ===== STEP 1: EXECUTE SINGLE TRANSACTION (BLOCKING) =====
  // Global slaveSpiTransaction reused - initialized once in initSPIComm()
  // txBuf[0] = dummy, txBuf[1] = status (both maintained globally)
  // txBuf[2+] = sensor data (prefilled after LOCK)
  
  esp_err_t ret = spi_slave_transmit(SPI2_HOST, &slaveSpiTransaction, 100);
  
  if (ret != ESP_OK) {
    if (ret == ESP_ERR_TIMEOUT) {
      return;  // Normal - no master activity yet
    } else {
      Serial.printf("[Slave] SPI Error: %s\n", esp_err_to_name(ret));
      return;
    }
  }
  
  // ===== STEP 2: PROCESS RECEIVED COMMAND =====
  transaction_count++;
  
  uint8_t cmdByte = rxBuf[0];
  uint8_t isRead = (cmdByte & PROTO_CMD_READ) ? 1 : 0;
  uint8_t address = cmdByte & PROTO_ADDR_MASK;
  
  Serial.printf("[Slave] RX #%lu: CMD=0x%02X (ADDR=0x%02X) State=%d | ", 
    transaction_count, cmdByte, address, slaveState);
    
    // ========== ADDRESS-BASED DISPATCH (WRITE ONLY) ============
    // Master sends WRITE commands to control state and prefill txBuf
    // Master reads prefilled txBuf bytes directly (no special read addressing)
    
    if (!isRead) {
      uint8_t dataValue = rxBuf[2];  // Data is in byte 2 (protocol: byte 0=cmd, byte 1=0x00, byte 2=data)
      
      if (address == ADDR_CTRL) {
        // ===== CONTROL REGISTER: Measurement control =====
        if (dataValue == CTRL_TRIGGER_MEASUREMENT) {
          Serial.println("WRITE CTRL: TRIGGER_MEASUREMENT");
          
          // Accept from IDLE or READY_TRANSFER (allows seamless re-trigger after measurement)
          if (slaveState == STATE_IDLE || slaveState == STATE_READY_TRANSFER) {
            slaveState = STATE_MEASURING;
            measurementStartTime = millis();
            
            // Update txBuf status (now measuring)
            txBuf[1] = STATUS_MEASURING;
            
            // Signal background measurement task
            xEventGroupSetBits(spiEventGroup, EVENT_TRIGGER_RECEIVED);
            
            Serial.println("[SPI] ✓ Measurement triggered");
          } else {
            Serial.printf("ERROR: TRIGGER in state %d (ignore)\n", slaveState);
          }
        }
        
        else if (dataValue == CTRL_LOCK_BUFFERS) {
          Serial.println("WRITE CTRL: LOCK_BUFFERS");
          
          // Lock only if measurement is complete (status byte shows MEASURED)
          if (txBuf[1] == STATUS_MEASURED) {
            // Measurement done, fill txBuf starting at byte 2 with sensor data
            xSemaphoreTake(currentDataMutex, portMAX_DELAY);
            
            currentData.sequence = sequenceNumber++;
            currentData.status = STATUS_LOCKED;
            
            // Copy sensor packet to txBuf starting at byte 2
            // (Byte 0=dummy, Byte 1=status, Bytes 2+=data)
            memcpy(txBuf + 2, &currentData, sizeof(SensorDataPacket));
            
            xSemaphoreGive(currentDataMutex);
            
            // Transition to READY_TRANSFER
            slaveState = STATE_READY_TRANSFER;
            
            // Update txBuf status (buffers locked, ready for read)
            txBuf[1] = STATUS_LOCKED;
            
            Serial.println("[SPI] ✓ Sensor data prefilled at txBuf[2+], ready for read");
          } else {
            // Measurement not yet complete
            Serial.printf("ERROR: LOCK called but measurement not ready (status=0x%02X)\n", txBuf[1]);
          }
        }
        
        else {
          Serial.printf("ERROR: Unknown CTRL value 0x%02X\n", dataValue);
        }
        // Note: Don't clear txBuf, status byte [1] stays prefilled
      }
      
      else if (address == ADDR_IR_LED) {
        // ===== IR LED CONTROL (0x00=off, 0x01=on) =====
        Serial.printf("WRITE IR_LED: %s\n", dataValue ? "ON" : "OFF");
        IRLED(dataValue);
        // Status at [1] stays prefilled
      }
      
      else if (address == ADDR_BRIGHTNESS) {
        // ===== LED BRIGHTNESS (0-100) =====
        Serial.printf("WRITE BRIGHTNESS: %d%%\n", dataValue);
        powerLED(dataValue);
        // Status at [1] stays prefilled
      }
      
      else {
        Serial.printf("ERROR: WRITE unknown address 0x%02X\n", address);
      }
    } else {
      Serial.println("READ (no-op)");
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
  Serial.println("[Slave] High-speed sampler task created on Core 1 @ 2kHz");
}

// Start the IR sensor background task (continuous 200ms sampling with caching)
void startIRSensorTask() {
  xTaskCreatePinnedToCore(
    irSensorBackgroundTask,
    "IRSensorTask",
    8192,           // Stack size (8 KB - sufficient for MLX90641 library)
    NULL,           // Parameters
    1,              // Priority (same as sampler, won't interfere with measurement)
    &irTaskHandle,
    1               // Core 1 (background caching task)
  );
  Serial.println("[Slave] IR sensor task created on Core 1 @ 200ms");
}

// Start the BME sensor background task (continuous 200ms sampling with caching)
void startBMESensorTask() {
  xTaskCreatePinnedToCore(
    bmeSensorBackgroundTask,
    "BMESensorTask",
    8192,           // Stack size (8 KB - sufficient for BME688 library)
    NULL,           // Parameters
    1,              // Priority (same as sampler, won't interfere with measurement)
    &bmeTaskHandle,
    1               // Core 1 (background caching task)
  );
  Serial.println("[Slave] BME SENSOR task created on Core 1 @ 200ms");

}
