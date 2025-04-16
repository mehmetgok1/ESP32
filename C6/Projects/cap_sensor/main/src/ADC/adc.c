#include "adc.h"
#define TAG "AD_MODULE"
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
void adc_init( )
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
    handle = NULL;
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

    ESP_LOGI(TAG, "adc_pattern[0].atten is :%" PRIx8, adc_pattern[0].atten);
    ESP_LOGI(TAG, "adc_pattern[0].channel is :%" PRIx8, adc_pattern[0].channel);
    ESP_LOGI(TAG, "adc_pattern[0].unit is :%" PRIx8, adc_pattern[0].unit);
    dig_cfg.adc_pattern = adc_pattern;

    printf("here11111");
    if (handle == NULL) {
        printf("Handle creation failed!\n");
    }else {
        printf("naber");
    }
    printf("dig_cfg.sample_freq_hz: %d\n", (int)dig_cfg.sample_freq_hz);
    printf("dig_cfg.pattern_num: %d\n", (int)dig_cfg.pattern_num);
    printf("dig_cfg.adc_pattern[0].channel: %d\n", dig_cfg.adc_pattern[0].channel);
    printf("dig_cfg.adc_pattern[0].atten: %d\n", dig_cfg.adc_pattern[0].atten);
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));
    printf("here2222");	
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    // Start ADC Monitor
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
    ESP_ERROR_CHECK(adc_continuous_start(handle));
}
