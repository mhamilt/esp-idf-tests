//  Blink Example

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
   
static const gpio_num_t BLINK_GPIO = 2;

void app_main(void)
{
    gpio_reset_pin(BLINK_GPIO);    
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    
    while (1) {    
        gpio_set_level(BLINK_GPIO, true);               
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(BLINK_GPIO, false);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}