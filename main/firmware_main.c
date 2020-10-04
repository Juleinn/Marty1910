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

/**
 * Brief:
 * This test code shows how to configure gpio and how to use gpio interrupt.
 *
 * GPIO status:
 * GPIO18: output
 * GPIO19: output
 * GPIO4:  input, pulled up, interrupt from rising edge and falling edge
 * GPIO5:  input, pulled up, interrupt from rising edge.
 *
 * Test:
 * Connect GPIO18 with GPIO4
 * Connect GPIO19 with GPIO5
 * Generate pulses on GPIO18/19, that triggers interrupt on GPIO4/5
 *
 */

#define P2_0        18
#define EAN         19
#define MFB         13
#define RST         12
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<P2_0) | (1ULL<<EAN) | (1ULL<<MFB) | (1ULL<<RST))

void bm64_reset(void)
{
    gpio_set_level(MFB, 0);
    gpio_set_level(RST, 0);
    vTaskDelay(499 / portTICK_RATE_MS);

    gpio_set_level(MFB, 1);

    vTaskDelay(1 / portTICK_RATE_MS);

    gpio_set_level(RST, 1);
}

void app_main(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
    gpio_set_level(P2_0, 1);
    gpio_set_level(EAN, 1);
    gpio_set_level(MFB, 1);
    gpio_set_level(RST, 1);

    bm64_reset();

    while(1) {
        vTaskDelay(1000 / portTICK_RATE_MS);
        printf("Running...\r\n");
    }
}

