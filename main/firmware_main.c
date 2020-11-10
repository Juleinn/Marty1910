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

// https://stackoverflow.com/questions/9907160/how-to-convert-enum-names-to-string-in-c
#define FOREACH_STATE(STATE) \
        STATE(IDLE)   \
        STATE(RINGING)  \
        STATE(COMMUNICATION)   \
        STATE(WAIT_ON_HOOK)  \
        STATE(WAIT_OFF_HOOK)  \
        STATE(WAIT_CRANK_END)

#define GENERATE_ENUM(ENUM) ENUM,
#define GENERATE_STRING(STRING) #STRING,

typedef enum State {
    FOREACH_STATE(GENERATE_ENUM)
} State;

static const char __attribute__((unused)) *STATE_NAMES[] = {
    FOREACH_STATE(GENERATE_STRING)
};

static State current_state = IDLE;

TaskHandle_t taskMainLoop_Handle;


static volatile SemaphoreHandle_t xSemaphore = NULL;
// override weak function
void bm64_on_incomming_call()
{
    printf("Incomming call (main override)\n");
    // go to ringing state
    xSemaphoreGiveFromISR(xSemaphore, NULL);
}


void task_main_loop()
{
    printf("main_loop...\n");
    while(1)
    {
        switch(current_state)
        {
            case IDLE:
                //line is disconnected in idle mode
                line_connect(false);

                printf("Idling...\n");
                xSemaphoreTake(xSemaphore,portMAX_DELAY);
                printf("Idling complete ! \n");

                if(line_is_cranking())
                {

                }
                else
                {
                    printf("Line is not being crancked. Connecting line...\n");
                    line_connect(true);
                    // give relays time to flip
                    vTaskDelay(500 / portTICK_RATE_MS);
                    // check off-hook
                    if(ag1171_is_offhook())
                    {
                        // already off-hook : reject call
                        printf("Off-hook : rejecting call\n");
                    }
                    else
                    {
                        // TODO start ringing here
                        printf("on-hook : ringing start\n");
                    }

                }

                break;
            case RINGING:
                break;
            case COMMUNICATION:
                break;
            case WAIT_ON_HOOK:
                break;
            case WAIT_OFF_HOOK:
                break;
            case WAIT_CRANK_END:
                break;
        }
    }
}

void app_main(void)
{
    line_init();
    line_connect(true);
    vTaskDelay(1000 / portTICK_RATE_MS);

    bm64_init();
    ag1171_init();

    // create main loop task
    xSemaphore = xSemaphoreCreateBinary();

    task_main_loop();

    while(1) {
        printf("Phone cranking : %s\n", (line_is_cranking() ? "true": "false"));
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

