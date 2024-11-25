#include "touch.h"
void init_touch(){
    i2c_master_bus_handle_t i2c_bus_handle;
    #define BOARD_I2C_PORT I2C_NUM_1
    #define BOARD_I2C_SDA 6
    #define BOARD_I2C_SCL 7
    #define CONFIG_LCD_H_RES 240
    #define CONFIG_LCD_V_RES 240
    #define BOARD_TOUCH_IRQ (5)
    #define BOARD_TOUCH_RST (13)
    i2c_master_bus_config_t i2c_bus_config = {
          .i2c_port = BOARD_I2C_PORT,
          .sda_io_num = BOARD_I2C_SDA,
          .scl_io_num = BOARD_I2C_SCL,
          .clk_source = I2C_CLK_SRC_DEFAULT,
          .glitch_ignore_cnt = 7,
          .intr_priority = 0,
          .flags.enable_internal_pullup = 1};
     ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle)); 

    esp_lcd_panel_io_handle_t tp_io_handle=NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config =
        ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
    tp_io_config.scl_speed_hz = (400 * 1000);
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus_handle, &tp_io_config,
                                             &tp_io_handle));
                                             
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = CONFIG_LCD_H_RES,
        .y_max = CONFIG_LCD_V_RES,
        .rst_gpio_num = BOARD_TOUCH_RST,
        .int_gpio_num = BOARD_TOUCH_IRQ,
        .levels =
            {
                .reset = 0,
                .interrupt = 0,
            },
        .flags =
            {
                .swap_xy = 0,
                .mirror_x = 0,
                .mirror_y = 0,
            },
        .interrupt_callback = NULL,
        .process_coordinates = NULL,
    };
    esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &tp);
}
