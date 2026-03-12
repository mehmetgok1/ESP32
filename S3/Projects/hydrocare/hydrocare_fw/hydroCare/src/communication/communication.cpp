#include "communication/communication.h"
#include <Arduino.h>
#include <SPI.h>
#include "config/config.h"
#include "measurement/measurement.h""

#define SPI_SYNC_BYTE    0xAA
#define LED_CONTROL_CMD  0x01
#define IR_CONTROL_CMD   0x02
#define AMB_LIGHT_CMD    0x03
#define SPI_CLOCK_HZ     16000000 

// Global variable to store the latest reading
SPIClass spi(HSPI);

void initSPIComm(){

  spi.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);
}

void readAmbientLight_Int() {
  spi.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(SPI_CS, LOW);
  delayMicroseconds(10);

  // Byte 0: Send Sync, Receive ACK (Ignore ACK for now)
  spi.transfer(SPI_SYNC_BYTE);
  
  // Byte 1: Send Command 0x03, Receive High Byte
  uint8_t highByte = spi.transfer(AMB_LIGHT_CMD);
  
  // Byte 2: Send Dummy (0x00), Receive Low Byte
  uint8_t lowByte = spi.transfer(0x00);

  delayMicroseconds(10);
  digitalWrite(SPI_CS, HIGH);
  spi.endTransaction();

  // Combine bytes: (High << 8) | Low
  ambLight_Int = (uint16_t)((highByte << 8) | lowByte);
}

void sendIRLED(bool state){

  spi.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(SPI_CS, LOW);
  delayMicroseconds(10);

  spi.transfer(SPI_SYNC_BYTE);     // Byte 0: sync
  spi.transfer(IR_CONTROL_CMD);   // Byte 1: command
  spi.transfer(state);        // Byte 2: brightness

  delayMicroseconds(10);
  digitalWrite(SPI_CS, HIGH);
  spi.endTransaction();

  Serial.printf("[Master] SYNC=0x%02X  CMD=0x%02X  IR=%d\n",
                SPI_SYNC_BYTE, IR_CONTROL_CMD, state);
}
void sendBrightness(uint8_t brightness) {

  spi.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(SPI_CS, LOW);
  delayMicroseconds(10);

  spi.transfer(SPI_SYNC_BYTE);     // Byte 0: sync
  spi.transfer(LED_CONTROL_CMD);   // Byte 1: command
  spi.transfer(brightness);        // Byte 2: brightness

  delayMicroseconds(10);
  digitalWrite(SPI_CS, HIGH);
  spi.endTransaction();

  Serial.printf("[Master] SYNC=0x%02X  CMD=0x%02X  Brightness=%3d\n",
                SPI_SYNC_BYTE, LED_CONTROL_CMD, brightness);
}