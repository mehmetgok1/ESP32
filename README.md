# ESP32

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
  communicate with display and touch controller.
