# ESP32-S3

## Versions
Esp-idf -> 5.3.1  
lvgl    -> 8.3.11

## File Structure
### src/display_controller
  this file initialize GC9A01 sensor driver in idf envirounment  
  how it is doing basically, it create SPI bus then assign sensor spisific IO  
  to this bus and once spi interface is okay, return handle to further used by LVGL
### src/touch_controller
  Nearly same with GC9A01 but this time sensor is CST816S, this buddy uses I2C  
  so we i2c initialization is and sensor spesific io registered to driver and driver gives  
  handle file to use this sensor which will be used by LVGL.  
### src/dptp_mng
  this buddy simply takes this handle files and introduce them to LVGL hence lvgl know how to  
  communicate with display and touch controller. (meanwhile lvgl needs times etc this file initialize them also)  
### src/ble 
  This file initiates bluetooth low energy drivers for this to be used, in menuconfig ble should be enabled.  
  GAP GATTC handling is done here.
  ALSO ble.c contain information about ble server and characteristic UUID change as you wish (defaults are a little long but names are REMOTE_SERVICE_UUID and REMOTE_CHAR_UUID)
  
### src/ui 
  User interface related files are inside this source files.

## NOTE
  UI and BLE files are somehow interleaved, that's because some bluetooth functions and variables are kept in ui and  
  of course vice versa is also true.

## Please enable blueetooth component in idf.py menuconfig -> component config -> Bluetooth 

## How to build
  create empty project with idf.py create-project "project_name"
  go inside and replace the main directory completely with downloaded main from this repo  
  type idf.py set-target esp32s3 to choose s3 target.
  idf.py menuconfig to adjust configuration (go component config )
    1-) adjust fonts, go component config -> lvgl -> font -> enable montserra 16 18 20 22 24 26 fonts 
     
  idf.py build flash monitor
