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
#include "line.h"



void app_main(void)
{
    vTaskDelay(2000 / portTICK_RATE_MS);

    line_init();
    //xTaskCreate(relay_task, "relay_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);

    //bm64_init();
    ag1171_init();
    //vTaskDelay(1000 / portTICK_RATE_MS);

    //ag1171_ring_loop();

    while(1) {
        printf("Phone cranking : %s\n", (line_is_cranking() ? "true": "false"));
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

