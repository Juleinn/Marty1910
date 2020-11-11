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

#define FOREACH_EVENT(EVENT)    \
        EVENT(INCOMMING_CALL)   \
        EVENT(OFF_HOOK)         \
        EVENT(ON_HOOK)          \
        EVENT(CALL_ENDED)

typedef enum Event {
    FOREACH_EVENT(GENERATE_ENUM)
} Event;

static const char __attribute__((unused)) *EVENT_NAMES[] = {
    FOREACH_EVENT(GENERATE_STRING)
};

static State current_state = IDLE;

TaskHandle_t taskMainLoop_Handle;


static volatile SemaphoreHandle_t semaphore_idle = NULL;
static volatile SemaphoreHandle_t semaphore_ringing = NULL;

static xQueueHandle queue_idle = NULL;
static xQueueHandle queue_ringing = NULL;
static xQueueHandle queue_communication = NULL;

// override weak function
void bm64_on_incomming_call()
{
    printf("Incomming call (main override)\n");
    // go to ringing state
    static Event event = INCOMMING_CALL;
    xQueueSend(queue_idle, &event, 0); 
}

void ag1171_on_phone_offhook()
{
    printf("Phone off-hook (main override)\n");
    static Event event = OFF_HOOK;
    xQueueSendFromISR(queue_ringing, &event, 0);
}

void ag1171_on_phone_onhook()
{
    printf("Phone on-hook (main override)\n");
    static Event event = ON_HOOK;
    xQueueSendFromISR(queue_communication, &event, 0);
}

void task_main_loop()
{
    printf("main_loop...\n");
    while(1)
    {
        Event event;
        printf("current_state : %s\n", STATE_NAMES[current_state]);
        switch(current_state)
        {
            case IDLE:
                //line is disconnected in idle mode
                line_connect(false);

                printf("Idling...\n");
                //xSemaphoreTake(semaphore_idle,portMAX_DELAY);
                xQueueReceive(queue_idle, &event, portMAX_DELAY);
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
                        ag1171_start_ringing();
                        current_state = RINGING;
                        break;
                    }
                }
                break;
            case RINGING:
                printf("RINGING...\n");
                // wait for the phone to be picked up
                xQueueReceive(queue_ringing, &event, portMAX_DELAY);
                printf("Received event phone off-hook\n");
                if(ag1171_is_offhook())
                {
                    printf("Phone is off-hook : accepting the call\n");
                    ag1171_stop_ringing();
                    bm64_accept_call();
                    current_state = COMMUNICATION;
                    break;
                }

                break;
            case COMMUNICATION:
                // TODO monitor back on-hook and call ended (on smartphone side)
                xQueueReceive(queue_communication, &event, portMAX_DELAY);
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

    queue_idle = xQueueCreate(10, sizeof(Event));
    queue_ringing = xQueueCreate(10, sizeof(Event));
    queue_communication = xQueueCreate(10, sizeof(Event));

    line_init();
    line_connect(true);
    vTaskDelay(200 / portTICK_RATE_MS);

    bm64_init();
    ag1171_init();

    // create main loop task
    semaphore_idle = xSemaphoreCreateBinary();
    semaphore_ringing = xSemaphoreCreateBinary();

    task_main_loop();

    while(1) {
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

