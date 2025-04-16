#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_monitor.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <inttypes.h>
#include "freertos/queue.h"

#define SAMPLE_RATE         80000
#define ADC_HIGH_TH         3000
#define ADC_LOW_TH          1000
#define EXAMPLE_ADC_UNIT                    ADC_UNIT_1
#define _EXAMPLE_ADC_UNIT_STR(unit)         #unit
#define EXAMPLE_ADC_UNIT_STR(unit)          _EXAMPLE_ADC_UNIT_STR(unit)
#define EXAMPLE_ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define EXAMPLE_ADC_ATTEN                   ADC_ATTEN_DB_0
#define EXAMPLE_ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH
#define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type2.channel)
#define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type2.data)
#define EXAMPLE_READ_LEN                    1024
//GPIO defines
#define GPIO_OUTPUT_PIN 18  // Using GPIO18 as output

extern volatile int calculate;
extern volatile int high_count;
extern volatile int low_count;
extern TaskHandle_t s_task_handle;
extern adc_channel_t channel[];
extern adc_continuous_handle_t handle;
void adc_init();

