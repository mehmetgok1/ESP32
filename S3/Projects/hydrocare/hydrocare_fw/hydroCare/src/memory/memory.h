#ifndef MEMORY_H
#define MEMORY_H

#include "Arduino.h"

#define EEPROM_SIZE       1        // Size in bytes
#define deviceRoleAddress 0        // Address to read/write

extern String currentFileName;
extern String sessionFolder;      // Root session folder (e.g., /20260325_143022/)

void initEEPROM();
uint8_t eepromRead(uint8_t address);
void eepromWrite(uint8_t address, uint8_t value);
void initSD();
void initSessionFolder();          // Create timestamped folder structure
void createSubfolder(String folderName);  // Create subfolder in session
String getSensorDataPath();        // Get path for sensor CSV
String getColorCameraPath();       // Get path for color camera folder
String getThermalCameraPath();     // Get path for thermal camera folder
String getMicAccelPath();          // Get path for mic&accel CSV
void logMicAccelSamples(int16_t* accelX, int16_t* accelY, int16_t* accelZ, uint16_t* mic, uint16_t sampleCount);
void logData(String data);
void openFileAndHeader(String header);
void openMicAccelFile();      // Initialize mic&accel CSV with header
void setFileName(String name);

#endif