/* GPIO Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "bm64_driver.h"
#include "ag1171_driver.h"

#define RELAY_1         25
#define RELAY_2         26
#define RELAY_PIN_SEL  ((1ULL<<RELAY_1) | (1ULL<<RELAY_2))

void init_relays()
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = RELAY_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_set_level(RELAY_1, 0);
    gpio_set_level(RELAY_2, 0);
}

void relay_task()
{
    while(1)
    {
        gpio_set_level(RELAY_1, 0);
        gpio_set_level(RELAY_2, 0);
        vTaskDelay(2000 / portTICK_RATE_MS);
        gpio_set_level(RELAY_1, 1);
        gpio_set_level(RELAY_2, 1);
        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}

void app_main(void)
{
    vTaskDelay(2000 / portTICK_RATE_MS);

    // xTaskCreate(relay_task, "relay_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);

    //init_relays();
    //bm64_init();
    ag1171_init();
    vTaskDelay(1000 / portTICK_RATE_MS);
    ag1171_ring_loop();
    
    while(1) {
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

