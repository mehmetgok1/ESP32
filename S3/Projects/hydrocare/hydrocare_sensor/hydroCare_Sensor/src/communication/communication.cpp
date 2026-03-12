#include "communication/communication.h"
#include "config/config.h"
#include "driver/spi_slave.h"
#include "leds/leds.h"
#include "measurement/measurement.h"

// --- DMA-capable buffers (4 bytes allocated) ---
uint8_t *rxBuf;
uint8_t *txBuf;

void receiveCommand() {

  memset(txBuf, 0, 4);
  txBuf[0] = ACK_OK;               // Byte 0: Sent while Master sends SYNC (0xAA)
  txBuf[1] = (uint8_t)(ambLight >> 8);   // Byte 1: Sent while Master sends CMD
  txBuf[2] = (uint8_t)(ambLight & 0xFF); // Byte 2: Sent while Master sends VAL/Dummy

  spi_slave_transaction_t t = {};
  t.length    = 24;        // 3 bytes × 8 bits
  t.rx_buffer = rxBuf;
  t.tx_buffer = txBuf;

  // Wait for Master (This blocks until the Master starts the SPI clock)
  esp_err_t ret = spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);
  if (ret != ESP_OK) {
      Serial.printf("[Slave] SPI Transmit Error: %s\n", esp_err_to_name(ret));
      return;
  }
    // 3. Parse Incoming Data from Master
  uint8_t sync  = rxBuf[0];
  uint8_t cmd   = rxBuf[1];
  uint8_t value = rxBuf[2];

  // Debug Log
  Serial.printf("[Slave] RX -> SYNC: 0x%02X | CMD: 0x%02X | VAL: %d\n", sync, cmd, value);

    // 4. Handle Commands received from the Master
  switch (cmd) {
    case 0x01: // LED_CONTROL_CMD
        Serial.printf("[Slave] LED Setting: %d\n", value);
        powerLED(value);
        break;

    case 0x02: // IR_CONTROL_CMD
        Serial.printf("[Slave] IR Setting: %d\n", value);
        IRLED(value);
        break;

    case 0x03: // AMB_LIGHT_CMD
        // Note: The lightVal was already sent to the Master during the hardware transaction above.
        Serial.printf("[Slave] Ambient Light Streamed: %d\n", ambLight);
        break;

    default:
        // Only report unknown if the sync byte was correct (prevents logging noise)
        if (sync == 0xAA) {
            Serial.printf("[Slave] Unknown Command Received: 0x%02X\n", cmd);
        }
        break;
    }
}
void initSPIComm(){

  // Allocate DMA-capable buffers
  rxBuf = (uint8_t*) heap_caps_malloc(4, MALLOC_CAP_DMA);
  txBuf = (uint8_t*) heap_caps_malloc(4, MALLOC_CAP_DMA);
  if (!rxBuf || !txBuf) {
    Serial.println("[Slave] DMA buffer allocation failed!");
    while (true) delay(1000);
  }
  memset(rxBuf, 0, 4);
  memset(txBuf, 0, 4);

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

  Serial.println("=== SPI Slave Initialized & Ready ===");
}