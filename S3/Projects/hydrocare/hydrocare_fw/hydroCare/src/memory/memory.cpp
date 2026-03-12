#include <SPI.h>
#include <SD.h>
#include "memory/memory.h"
#include "Arduino.h"
#include "EEPROM.h"
#include "ble/ble.h"
#include "config/config.h"

String currentFileName = "/data.csv";

uint8_t eepromRead(uint8_t address)
{
    return EEPROM.read(address);
}
void eepromWrite(uint8_t address, uint8_t value)
{ 
    EEPROM.write(address, value);
    EEPROM.commit();
}
void initEEPROM()
{
    if (!EEPROM.begin(EEPROM_SIZE)) {
        Serial.println("Failed to initialise EEPROM");
        while (true);  // Stop execution
    }
    byte value = eepromRead(deviceRoleAddress);
    Serial.print("Read byte from EEPROM: ");
    Serial.println(value, HEX);
}
void initSD(){

  Serial.print("Mounting SD Card... ");

  if (!SD.begin(SD_CS)) {
    for(int i=0; i<5; i++){
        Serial.println("Error: Card not found or wiring incorrect.");
        delay(500);
    }
  }
  else
    Serial.println("Success.");
}
// Set the filename (ensure it starts with /)
void setFileName(String name) {
  if (!name.startsWith("/")) {
    currentFileName = "/" + name;
  } else {
    currentFileName = name;
  }
  if (!currentFileName.endsWith(".csv")) {
    currentFileName += ".csv";
  }
  Serial.println("Filename set to: " + currentFileName);
}
// Create the file and write the Header (e.g., Lux,PIR,Distance)
void openFileAndHeader(String header) {
  File file = SD.open(currentFileName, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.println(header)) {
    Serial.println("Header written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}
// Append a row of data to the CSV
void logData(String data) {
  // FILE_APPEND is crucial so you don't overwrite previous data
  File file = SD.open(currentFileName, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  
  if (file.println(data)) {
    Serial.println("Data logged: " + data);
  } else {
    Serial.println("Append failed");
  }
  file.close(); // Always close after writing to prevent data corruption
}