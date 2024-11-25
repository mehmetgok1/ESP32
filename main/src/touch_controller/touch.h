#include "lvgl.h"
#include "driver/i2c_master.h"
#include "esp_lcd_types.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_cst816s.h"

extern esp_lcd_touch_handle_t tp;
void init_touch(void);