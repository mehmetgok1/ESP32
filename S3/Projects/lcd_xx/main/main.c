
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "ui.h"

void app_main(void){  
    
    init_mng();
    init_ui();
    
}

