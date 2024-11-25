idf_component_register(SRCS "spi_lcd_touch_example_main.c" "lvgl_demo_ui.c" "src/touch_controller/touch.c" "src/display_controller/display.c" "src/dptp_mng/dptp_mng.c" "src/ble/ble.c" "src/ui/ui.c"
                       
            INCLUDE_DIRS "." "src/touch_controller" "src/display_controller" "src/dptp_mng" "src/ble" "src/ui"
            )

                       