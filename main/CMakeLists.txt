idf_component_register(SRCS "main.c" "src/touch_controller/touch.c" "src/display_controller/display.c" "src/dptp_mng/dptp_mng.c" "src/ble/ble.c" "src/ui/ui.c"
                       
            INCLUDE_DIRS "." "src/touch_controller" "src/display_controller" "src/dptp_mng" "src/ble" "src/ui"
            )

                       