#include "communication/communication.h"
#include "config/config.h"
#include "driver/spi_slave.h"
#include "esp_task_wdt.h"

// DMA-capable buffers (256 bytes per transaction - reliable data packets)
uint8_t *rxBuf;
uint8_t *txBuf;
const uint16_t SPI_BUFFER_SIZE = 256;  // 256 bytes per transaction

// Task handles (global)
TaskHandle_t spiTaskHandle = NULL;
TaskHandle_t dataTaskHandle = NULL;

// Global data buffer
extern DataBuffer g_dataBuffer;

// SPI Communication Task - polls for master requests
void spiCommunicationTask(void *pvParameters) {
  (void)pvParameters;
  
  // Remove this task from watchdog (SPI can block waiting for master)
  esp_task_wdt_delete(NULL);
  
  Serial.println("[SPI Task] Started - waiting for master transfers");
  
  while (1) {
    // Block until master initiates SPI transfer
    receiveCommand();
    
    // Small delay to prevent tight loop
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// Data Update Task - simulates sensor readings and updates buffer
void dataUpdateTask(void *pvParameters) {
  (void)pvParameters;
  
  Serial.println("[Data Task] Started - updating sensor readings every 100ms");
  
  static uint16_t counter = 0;
  
  while (1) {
    // Simulate sensor readings
    g_dataBuffer.updateAmbLight(1000 + (counter % 100));        // Simulate varying light
    g_dataBuffer.updateTemperature(25.0f + (counter * 0.01f));  // Slowly increasing
    g_dataBuffer.updateHumidity(50.0f + (counter * 0.05f));     // Slowly increasing
    g_dataBuffer.updateIMU(
      100 * sin(counter * 0.01f),   // Accel X
      100 * cos(counter * 0.01f),   // Accel Y
      100,                          // Accel Z (constant)
      10 * sin(counter * 0.02f),    // Gyro X
      10 * cos(counter * 0.02f),    // Gyro Y
      5                             // Gyro Z (constant)
    );
    g_dataBuffer.updateTimestamp();
    
    counter++;
    vTaskDelay(pdMS_TO_TICKS(100));  // Update every 100ms
  }
}

void initSPIComm() {
  // Allocate DMA buffers
  rxBuf = (uint8_t*) heap_caps_malloc(SPI_BUFFER_SIZE, MALLOC_CAP_DMA);
  txBuf = (uint8_t*) heap_caps_malloc(SPI_BUFFER_SIZE, MALLOC_CAP_DMA);
  
  if (!rxBuf || !txBuf) {
    Serial.println("[Slave] DMA allocation failed!");
    while(1) delay(1000);
  }
  
  // Initialize data buffer
  g_dataBuffer.init();
  
  // Clear initial buffers
  memset(rxBuf, 0, SPI_BUFFER_SIZE);
  memset(txBuf, 0, SPI_BUFFER_SIZE);
  
  spi_bus_config_t buscfg = {
    .mosi_io_num     = SPI_MOSI,
    .miso_io_num     = SPI_MISO,
    .sclk_io_num     = SPI_SCK,
    .quadwp_io_num   = -1,
    .quadhd_io_num   = -1,
    .max_transfer_sz = 4096,
  };

  spi_slave_interface_config_t slvcfg = {
    .spics_io_num  = SPI_CS,
    .flags         = 0,
    .queue_size    = 1,
    .mode          = 0,
    .post_setup_cb = NULL,
    .post_trans_cb = NULL,
  };

  esp_err_t ret = spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
  if (ret != ESP_OK) {
    Serial.printf("[Slave] Init failed: %s\n", esp_err_to_name(ret));
    while(1) delay(1000);
  }
  
  Serial.println("[Slave] SPI Initialized (256-byte data packets)");
}

void receiveCommand() {
  static spi_slave_transaction_t t;  
  static int transaction_count = 0;
  
  // Prepare current sensor data in txBuf
  g_dataBuffer.prepareTxBuffer(txBuf, SPI_BUFFER_SIZE);
  
  // Setup transaction
  memset(&t, 0, sizeof(spi_slave_transaction_t));
  t.length    = SPI_BUFFER_SIZE * 8;  // 256 bytes in bits
  t.rx_buffer = rxBuf;
  t.tx_buffer = txBuf;

  // Wait for master (300ms timeout)
  esp_err_t ret = spi_slave_transmit(SPI2_HOST, &t, 300);
  if (ret == ESP_OK) {
    transaction_count++;
    Serial.printf("[Slave] TX #%d: Sent 256 bytes of sensor data\n", transaction_count);
  } else if (ret == ESP_ERR_TIMEOUT) {
    // Timeout is normal - master not always sending
  } else {
    Serial.printf("[Slave] Error: %s (code: 0x%x)\n", esp_err_to_name(ret), ret);
  }
}