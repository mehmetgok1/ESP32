#ifndef MEMORY_H
#define MEMORY_H

#include "Arduino.h"

#define EEPROM_SIZE       1        // Size in bytes
#define deviceRoleAddress 0        // Address to read/write

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
void logMicAccelSamples(int16_t* accelX, int16_t* accelY, int16_t* accelZ, uint16_t* mic, uint16_t sampleCount, uint32_t timestamp);
void logData(String data);
void logSensorData(uint32_t timestamp, float batteryPct, float ambLight, float pir, float mmWaveDist, float mmWaveEnergy, 
                   float staticDist, float staticEnergy, float detectionDist, float ambLight_slave, float humidity, float temperature);
void openMicAccelFile(uint32_t timestamp = 0);           // Initialize mic&accel CSV with header
void initSensorDataFile();         // Initialize sensor data CSV with header
void openSensorDataFile(uint32_t timestamp = 0);       // Open sensor data file (for rotation)
void openRGBSessionFile(uint32_t timestamp = 0);       // Open persistent RGB file at session start
void openIRSessionFile(uint32_t timestamp = 0);        // Open persistent IR file at session start
void closeSessionFiles();          // Close persistent RGB/IR files at session stop
void checkAndRotateFiles(uint32_t currentTime);  // Check if 60 seconds elapsed and rotate
void saveRGBImage(uint16_t* rgbFrame, uint32_t timestamp);  // Append RGB frame to stream
void saveIRImage(uint16_t* irFrame, uint32_t timestamp);    // Append IR frame to stream

#endif