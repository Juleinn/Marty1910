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
        STATE(WAIT_CRANK_END)  \
        STATE(WAIT_OFF_HOOK)

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
        EVENT(OUTGOING_CALL)    \
        EVENT(OFF_HOOK)         \
        EVENT(ON_HOOK)          \
        EVENT(CALL_ENDED)       \
        EVENT(CRANKING_STATUS)  

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
static xQueueHandle queue_wait_off_hook = NULL;
static xQueueHandle queue_communication = NULL;
static xQueueHandle queue_cranking = NULL;

#define LOG(fmt, ...) printf("[%s] " fmt, STATE_NAMES[current_state], ##__VA_ARGS__)

// override weak function
void bm64_on_incomming_call()
{
    LOG("Incomming call (main override)\n");
    // go to ringing state
    static Event event = INCOMMING_CALL;
    xQueueSend(queue_idle, &event, 0); 
}

void bm64_on_call_status_idle()
{
    LOG("Call status idle (main override)\n");
    static Event event = CALL_ENDED;
    xQueueSend(queue_communication, &event, 0); 
    xQueueSend(queue_ringing, &event, 0); 
}

void ag1171_on_phone_offhook()
{
    LOG("Phone off-hook (main override)\n");
    static Event event = OFF_HOOK;
    xQueueSendFromISR(queue_ringing, &event, 0);
    xQueueSendFromISR(queue_wait_off_hook, &event, 0);
}

void ag1171_on_phone_onhook()
{
    LOG("Phone on-hook (main override)\n");
    static Event event = ON_HOOK;
    xQueueSendFromISR(queue_communication, &event, 0);
}

void bm64_on_outgoing_call()
{
    LOG("Outgoing call from smartphone\n");
    static Event event = OUTGOING_CALL;
    xQueueSend(queue_idle, &event, 0);
}

static void empty_queue(xQueueHandle q)
{
    uint32_t data;
    while(xQueueReceive(q, &data, 0));
}

void line_on_cranking_changed()
{
    LOG("Line cranking status changed : %d\n", line_is_cranking());
    static Event event = CRANKING_STATUS;
    xQueueSend(queue_idle, &event, 0); 
    xQueueSend(queue_cranking, &event, 0); 
}


void task_main_loop()
{
    LOG("main_loop...\n");
    while(1)
    {
        Event event;

        LOG("current_state : %s\n", STATE_NAMES[current_state]);
        switch(current_state)
        {
            case IDLE:
                //line is disconnected in idle mode
                line_connect(false);

                LOG("Idling...\n");

                // clear queue in case of duplicate events
                empty_queue(queue_idle);
                //xSemaphoreTake(semaphore_idle,portMAX_DELAY);
                xQueueReceive(queue_idle, &event, portMAX_DELAY);
                LOG("Idling complete ! \n");

                if(line_is_cranking())
                {
                    LOG("line_is_cranking (main loop) !\n");
                    current_state = WAIT_CRANK_END;
                    break;
                }
                else
                {
                    line_connect(true); // always connect the line
                    if(event == INCOMMING_CALL)
                    {
                        LOG("Line is not being crancked. Connecting line...\n");
                        // give relays time to flip
                        vTaskDelay(500 / portTICK_RATE_MS);
                        // check off-hook
                        if(ag1171_is_offhook())
                        {
                            // already off-hook : reject call
                            LOG("Off-hook : rejecting call\n");
                        }
                        else
                        {
                            ag1171_start_ringing();
                            current_state = RINGING;
                            break;
                        }
                    }
                    else if(event == OUTGOING_CALL)
                    {
                        // line is not cranking, as we previously checked above
                        // line is also connected 
                        // we must make sure we're offhook otherwise notify user to either go off-hook or switch its communication device to internal MIC/headphones/speaker
                        if(!ag1171_is_offhook())
                        {
                            ag1171_ring_short();
                        }
                        // now wait for phone to go off-hook
                        current_state = WAIT_OFF_HOOK;
                        break;
                    }
                }
                break;
            case RINGING:
                LOG("RINGING...\n");
                // clear queue in case of duplicate events
                empty_queue(queue_ringing);

                // wait for the phone to be picked up
                xQueueReceive(queue_ringing, &event, portMAX_DELAY);
                if(event == CALL_ENDED)
                {
                    LOG("Call ended : back to idle\n");
                    ag1171_stop_ringing();
                    current_state = IDLE;
                    break;
                }
                if(event == OFF_HOOK || ag1171_is_offhook())
                {
                    LOG("Phone is off-hook : accepting the call\n");
                    ag1171_stop_ringing();
                    bm64_accept_call();
                    current_state = COMMUNICATION;
                    break;
                } 

                break;

            case WAIT_OFF_HOOK:
                LOG("Waiting for phone to go off hook\n");
                empty_queue(queue_wait_off_hook);
                xQueueReceive(queue_wait_off_hook, &event, portMAX_DELAY);
                // now we can go to communication state because even if the callee doesn't answer the call
                // we can only wait to go back on hook
                current_state = COMMUNICATION;
                break;
            case COMMUNICATION:
                // clear queue in case of duplicate events
                empty_queue(queue_communication);
                xQueueReceive(queue_communication, &event, portMAX_DELAY);
                LOG("Event : %s\n", EVENT_NAMES[event]);
                if(event == ON_HOOK || !ag1171_is_offhook()) 
                {
                    LOG("On-hook : end call\n");
                    bm64_end_call();
                }
                if(event == CALL_ENDED)
                {
                    LOG("Call ended: back to idle\n");
                }
                vTaskDelay(200 / portTICK_RATE_MS);
                current_state = IDLE;
                line_connect(false); // this will be done in IDLE anyway
                break;
            case WAIT_CRANK_END:
                empty_queue(queue_cranking);
                xQueueReceive(queue_cranking, &event, portMAX_DELAY);
                if(!line_is_cranking())
                {
                    LOG("Line cranking complete\n");
                    current_state = IDLE;
                }
                break;


        }
    }
}

void app_main(void)
{

    queue_idle = xQueueCreate(1, sizeof(Event));
    queue_ringing = xQueueCreate(1, sizeof(Event));
    queue_wait_off_hook = xQueueCreate(1, sizeof(Event));
    queue_communication = xQueueCreate(1, sizeof(Event));
    queue_cranking = xQueueCreate(1, sizeof(Event));

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

