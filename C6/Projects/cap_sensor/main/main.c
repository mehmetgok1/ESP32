#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_monitor.h"
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/queue.h"
#include "driver/gpio.h"
#include <math.h>
/// ADC DEFINES
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
#define ADC_LOW_TH                          1500
#define ADC_HIGH_TH                         2500
#define SAMPLE_RATE                         10000 
//GPIO defines
#define GPIO_OUTPUT_PIN 18  // Using GPIO18 as output


static adc_channel_t channel[1] = {ADC_CHANNEL_2};

static TaskHandle_t s_task_handle;
static const char *TAG = "EXAMPLE";
static int calculate =0;
static int low_count=0;
static int high_count=0;


/*try store data */
#define MAX_ADC_VALUES 1024*1024*10  // optional upper cap
static uint16_t *adc_values = NULL;
static size_t adc_count = 0;
static size_t adc_capacity = 0;
void store_adc_value(uint16_t value) {
    if (adc_count >= adc_capacity) {
        // Expand capacity: double it or start with 64
        size_t new_capacity = adc_capacity == 0 ? 64 : adc_capacity * 2;
        if (new_capacity > MAX_ADC_VALUES) new_capacity = MAX_ADC_VALUES;

        uint16_t *new_data = realloc(adc_values, new_capacity * sizeof(uint16_t));
        if (!new_data) {
            // Handle allocation error
            return;
        }

        adc_values = new_data;
        adc_capacity = new_capacity;
    }

    // Store the value
    adc_values[adc_count++] = value;
}
void reset_adc_buffer() {
    adc_count = 0;
}

void free_adc_buffer() {
    free(adc_values);
    adc_values = NULL;
    adc_capacity = 0;
    adc_count = 0;
}

static bool IRAM_ATTR monitor_event_high(adc_monitor_handle_t monitor_handle, const adc_monitor_evt_data_t *event_data, void *user_data){
    calculate=0; 
    high_count++;
    gpio_set_direction(GPIO_OUTPUT_PIN,GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_OUTPUT_PIN, 0);
    return true;
}
static bool IRAM_ATTR monitor_event_low(adc_monitor_handle_t monitor_handle, const adc_monitor_evt_data_t *event_data, void *user_data){
	calculate=1; 
    low_count++;
    gpio_set_direction(GPIO_OUTPUT_PIN,GPIO_MODE_INPUT);
	return true;
}

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);
    return (mustYield == pdTRUE);
}
#include <math.h>

double extract_capacitance_charging(double V0, double Vt, double Vfinal, double t, double R) {
    double ln_argument = (Vfinal - Vt) / (Vfinal - V0);
    if (ln_argument <= 0) return -1.0;
    return -t / (R * log(ln_argument));
}

double average_capacitance_from_data(uint16_t *adc_data, int length, double vref, double Vfinal, double R) {
    int mid = length / 2;
    double sum = 0.0;
    int valid_count = 0;
    //ESP_LOGI(TAG,"location1=%d data1=%d, location2=%d,data2=%d",10000,adc_data[10000],mid+10000,adc_data[mid+10000]);
    for (int i = 0; i < mid; i=i+50) {
        double V0 = ((double)adc_data[i] * vref) / 4096.0;
        double Vt = ((double)adc_data[mid + i] * vref) / 4096.0;
        double t = (double)mid/SAMPLE_RATE;

        double C = extract_capacitance_charging(V0, Vt, Vfinal, t, R);
        if (C > 0) {
            sum += C;
            valid_count++;
        }
    }

    if (valid_count == 0) return -1.0;  // No valid data
    return (sum / valid_count)*1e9;
}
void app_main(void)
{
    //gpio initialization
    // Configure the GPIO pin as output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_OUTPUT_PIN),  // Bit mask for GPIO18
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(GPIO_OUTPUT_PIN, 0);
    //ADC Configuration
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[EXAMPLE_READ_LEN] = {0};
    memset(result, 0xcc, EXAMPLE_READ_LEN);

    s_task_handle = xTaskGetCurrentTaskHandle();  	
	
    adc_continuous_handle_t handle = NULL;
    adc_continuous_handle_cfg_t adc_config = {
	.max_store_buf_size = 1024,
	.conv_frame_size = 256,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));
	
    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = SAMPLE_RATE,
        .conv_mode = EXAMPLE_ADC_CONV_MODE,
        .format = EXAMPLE_ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = 1;
	adc_pattern[0].atten = EXAMPLE_ADC_ATTEN;
	adc_pattern[0].channel = channel[0] & 0x7;
	adc_pattern[0].unit = EXAMPLE_ADC_UNIT;
	adc_pattern[0].bit_width = EXAMPLE_ADC_BIT_WIDTH;

	ESP_LOGI(TAG, "adc_pattern[%d].atten is :%"PRIx8, 0, adc_pattern[0].atten);
	ESP_LOGI(TAG, "adc_pattern[%d].channel is :%"PRIx8, 0, adc_pattern[0].channel);
	ESP_LOGI(TAG, "adc_pattern[%d].unit is :%"PRIx8, 0, adc_pattern[0].unit);

    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));
	
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
	
    //START ADC_MONITOR
    adc_monitor_handle_t monitor_handle;
    adc_monitor_config_t monitor_config = {
	.adc_unit = ADC_UNIT_1,
        .channel = ADC_CHANNEL_2,
        .h_threshold = ADC_HIGH_TH,     
	    .l_threshold = ADC_LOW_TH,
    };
	
    adc_monitor_evt_cbs_t cbs_monitor = {
        .on_over_high_thresh = monitor_event_high,
        .on_below_low_thresh = monitor_event_low,
    };
	
    ESP_ERROR_CHECK(adc_new_continuous_monitor(handle, &monitor_config, &monitor_handle));
    ESP_ERROR_CHECK(adc_continuous_monitor_register_event_callbacks(monitor_handle, &cbs_monitor, NULL));
    ESP_ERROR_CHECK(adc_continuous_monitor_enable(monitor_handle));
    //END ADC_MONITOR
	
	int countflag=0;
    ESP_ERROR_CHECK(adc_continuous_start(handle));
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        char unit[] = EXAMPLE_ADC_UNIT_STR(EXAMPLE_ADC_UNIT);
            ret = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, 0);
            if (ret == ESP_OK) {
                //ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                    uint32_t chan_num = EXAMPLE_ADC_GET_CHANNEL(p);
                    uint32_t data = EXAMPLE_ADC_GET_DATA(p);
                    if (chan_num < SOC_ADC_CHANNEL_NUM(EXAMPLE_ADC_UNIT)) {
                        if(calculate==1 && data>ADC_LOW_TH){
                            countflag=0;
                            store_adc_value((uint16_t)data);
                        }else{
                            if(countflag==0 && adc_count>=5000){
                                ESP_LOGI(TAG, "adc data count:%d", adc_count);
                                double C=average_capacitance_from_data(adc_values,adc_count,(double)1.1,(double)3.3,600e6);
                                ESP_LOGI(TAG, "Average Capacitance: %.5e nF", C);
                                free_adc_buffer();
                                reset_adc_buffer();
                            }else if(adc_count<5000){
                                free_adc_buffer();
                                reset_adc_buffer();
                            }
                            countflag=1;
                        }
                    } else {
                        ESP_LOGW(TAG, "Invalid data [%s_%"PRIu32"_%"PRIx32"]", unit, chan_num, data);
                    }
                }
                
            } else if (ret == ESP_ERR_TIMEOUT) {
                //We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
                break;
            } 
            //ESP_LOGI(TAG, "low count:%d, high count:%d", low_count,high_count);
            //vTaskDelay(1);
        }

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}