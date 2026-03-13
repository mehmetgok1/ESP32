#include "communication/communication.h"
#include "config/config.h"
#include "driver/spi_slave.h"
#include "leds/leds.h"
#include "measurement/measurement.h"

// Global instances
DataBuffer *g_dataBuffer = NULL;
ProtocolStateMachine *g_protocolState = NULL;

// DMA-capable buffers for SPI transactions (max 256 bytes per chunk)
uint8_t *rxBuf;
uint8_t *txBuf;
const uint16_t SPI_BUFFER_SIZE = 256;

// Track data chunk offset for multi-part reads
static uint16_t dataChunkOffset = 0;

void handleMasterCommand(uint8_t cmd) {
  switch (cmd) {
    case 0x00:
      // Idle/ping command - do nothing, just respond with status
      break;
      
    case CMD_TRIGGER_MEASUREMENT:
      Serial.println("[Slave] Command: TRIGGER_MEASUREMENT");
      // Only notify if state actually transitioned from IDLE
      if (g_protocolState->triggerMeasurement()) {
        if (measurementTaskHandle) {
          xTaskNotifyGive(measurementTaskHandle);
          Serial.println("[Slave] ► Waking measurement task");
        }
      } else {
        Serial.println("[Slave] ✗ Cannot trigger from non-IDLE state");
      }
      break;
      
    case CMD_LOCK_BUFFERS:
      Serial.println("[Slave] Command: LOCK_BUFFERS");
      if (g_protocolState->isReady()) {
        g_protocolState->beginTransfer();
        g_dataBuffer->lockForTransfer();
        dataChunkOffset = 0;  // Reset for new data read cycle
        Serial.println("[Slave] ✓ Transitioned to TRANSFERRING for data burst");
      } else {
        Serial.printf("[Slave] ✗ Cannot lock - not in READY state (current=%d)\n", 
                      g_protocolState->getState());
      }
      break;
      
    case CMD_TRANSFER_COMPLETE:
      Serial.println("[Slave] Command: TRANSFER_COMPLETE");
      g_protocolState->completeTransfer();
      dataChunkOffset = 0;  // Reset for next measurement cycle
      break;
      
    case CMD_READ_DATA:
      // Read data chunk while in TRANSFERRING state
      // Response is built by sendStatusAndData() automatically
      break;
      
    case LED_CONTROL_CMD:
      // Extract value from rxBuf[1]
      Serial.printf("[Slave] Legacy LED Command: Value=%d\n", rxBuf[1]);
      powerLED(rxBuf[1]);
      break;
      
    case IR_CONTROL_CMD:
      // Extract value from rxBuf[1]
      Serial.printf("[Slave] Legacy IR Command: Value=%d\n", rxBuf[1]);
      IRLED(rxBuf[1]);
      break;
      
    default:
      Serial.printf("[Slave] Unknown Command: 0x%02X\n", cmd);
      break;
  }
}

void sendStatusAndData(uint8_t *txBuffer, uint16_t bufferSize) {
  // Byte 0: Always status
  txBuffer[0] = g_protocolState->getStatusByte();
  
  // If not transferring, only send status byte
  if (!g_protocolState->isTransferring()) {
    return;
  }
  
  // We're in TRANSFERRING state - send data chunks
  const SensorDataFrame *data = g_dataBuffer->getReadableBuffer();
  uint16_t dataStructSize = sizeof(SensorDataFrame);
  uint8_t *dataPtr = (uint8_t *)data;
  
  // Each SPI transaction can send max (bufferSize - 1) bytes of sensor data
  // because byte 0 is reserved for status
  uint16_t bytesPerChunk = bufferSize - 1;  // 255 bytes max per chunk
  
  // Calculate how many bytes to send in this chunk
  uint16_t bytesRemaining = dataStructSize - dataChunkOffset;
  uint16_t bytesToSend = (bytesRemaining > bytesPerChunk) ? bytesPerChunk : bytesRemaining;
  
  // Copy chunk data to txBuffer[1:]
  if (bytesToSend > 0) {
    memcpy(&txBuffer[1], &dataPtr[dataChunkOffset], bytesToSend);
    dataChunkOffset += bytesToSend;
    
    // Debug: show progress every 10 chunks
    static uint16_t chunkCount = 0;
    if (++chunkCount % 10 == 0) {
      Serial.printf("[Slave] Sending data chunk: offset=%u, sent=%u bytes\n", 
                    dataChunkOffset - bytesToSend, bytesToSend);
    }
  }
}

void receiveCommand() {
  memset(rxBuf, 0, SPI_BUFFER_SIZE);
  // NOTE: txBuf is NOT cleared - keep previous response prepared
  
  // IMPORTANT: Prepare response with CURRENT state BEFORE SPI transaction
  // This ensures master receives the correct status byte
  sendStatusAndData(txBuf, SPI_BUFFER_SIZE);

  // SPI transaction: 256 bytes max per transfer
  spi_slave_transaction_t t = {};
  t.length    = SPI_BUFFER_SIZE * 8;  // bits
  t.rx_buffer = rxBuf;
  t.tx_buffer = txBuf;

  // Wait for Master to initiate transfer (200ms timeout to prevent watchdog hang)
  esp_err_t ret = spi_slave_transmit(SPI2_HOST, &t, pdMS_TO_TICKS(200));
  if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
    Serial.printf("[Slave] SPI Transmit Error: %s\n", esp_err_to_name(ret));
    return;
  }

  // Parse command from master
  uint8_t cmd = rxBuf[0];
  uint8_t param = rxBuf[1];  // Secondary parameter if needed
  
  Serial.printf("[Slave] RX: CMD=0x%02X | PARAM=%d | State=%d\n", 
                cmd, param, g_protocolState->getState());

  // Handle command (updates state for next cycle's response)
  handleMasterCommand(cmd);
}

void initSPIComm() {
  // Initialize global instances
  if (!g_dataBuffer) {
    g_dataBuffer = new DataBuffer();
  }
  if (!g_protocolState) {
    g_protocolState = new ProtocolStateMachine();
  }

  // Allocate DMA-capable buffers (256 bytes for larger transfers)
  rxBuf = (uint8_t*) heap_caps_malloc(SPI_BUFFER_SIZE, MALLOC_CAP_DMA);
  txBuf = (uint8_t*) heap_caps_malloc(SPI_BUFFER_SIZE, MALLOC_CAP_DMA);
  
  if (!rxBuf || !txBuf) {
    Serial.println("[Slave] DMA buffer allocation failed!");
    while (true) delay(1000);
  }

  memset(rxBuf, 0, SPI_BUFFER_SIZE);
  memset(txBuf, 0, SPI_BUFFER_SIZE);
  
  // Initialize txBuf byte 0 with IDLE status (0x00) for first transaction
  txBuf[0] = g_protocolState->getStatusByte();

  spi_bus_config_t buscfg = {
    .mosi_io_num     = SPI_MOSI,
    .miso_io_num     = SPI_MISO,
    .sclk_io_num     = SPI_SCK,
    .quadwp_io_num   = -1,
    .quadhd_io_num   = -1,
    .max_transfer_sz = 0,
  };

  spi_slave_interface_config_t slvcfg = {
    .spics_io_num  = SPI_CS,
    .flags         = 0,
    .queue_size    = 1,
    .mode          = 0, // Mode 0: CPOL=0, CPHA=0
    .post_setup_cb = NULL,
    .post_trans_cb = NULL,
  };

  esp_err_t ret = spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
  if (ret != ESP_OK) {
    Serial.printf("[Slave] Init failed: %s\n", esp_err_to_name(ret));
    while (true) delay(1000);
  }

  Serial.println("=== SPI Slave Initialized (New Protocol) ===");
}