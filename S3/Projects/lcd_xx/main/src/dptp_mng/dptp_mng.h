#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "touch.h"
#include "display.h"
#include "esp_mac.h"

#define EXAMPLE_LVGL_TICK_PERIOD_MS    2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY     2

extern lv_disp_t *disp;
bool example_lvgl_lock(int timeout_ms);
void example_lvgl_unlock(void);
extern void example_lvgl_demo_ui(lv_disp_t *disp);
void init_mng(void);


