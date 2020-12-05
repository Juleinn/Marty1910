#include "ag1171_driver.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define RM 22
#define FR 23
#define SHK 34

#define GPIO_OUTPUT_PIN_SEL ((1ULL << RM) | (1ULL << FR))
#define GPIO_INPUT_PIN_SEL ((1ULL << SHK))
#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle gpio_evt_queue = NULL;
static void IRAM_ATTR ag1171_shk_isr_handler(void* args)
{
    uint32_t value = 0;
    xQueueSendFromISR(gpio_evt_queue, &value, NULL);
}


static void ag1171_ring_task();

void __attribute__((weak)) ag1171_on_phone_offhook()
{
    printf("Phone gone off-hook (weak)\n");
}

void __attribute__((weak)) ag1171_on_phone_onhook()
{
    printf("Phone gone on-hook (weak)\n");
}

static void ag1171_task(void* arg)
{
    uint32_t last_value = 0;
    for(;;) {
        uint32_t value1 = gpio_get_level(SHK); 
        if(value1 != last_value) 
        {
            vTaskDelay(15 / portTICK_RATE_MS); // ag1171 doc recommends >= 10ms
            uint32_t value2 = gpio_get_level(SHK); 
            if(value1 == value2)
            {
                if(value1)
                {
                    printf("Phone off-hook\n");
                    ag1171_on_phone_offhook();
                }
                else
                {
                    printf("Phone on-hook\n");
                    ag1171_on_phone_onhook();
                }
                last_value = value1;
            }

        }
        vTaskDelay(15 / portTICK_RATE_MS); // Lets not loop around too fast
    }
}

bool ag1171_is_offhook()
{
    return gpio_get_level(SHK);
}

void ag1171_init()
{
    printf("portTICK_RATE_MS : %d\n", portTICK_RATE_MS);
    gpio_config_t io_conf;

    // output config
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_level(FR, 1);
    gpio_set_level(RM, 0);

    // input config
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pull_down_en = 1;
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(1, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(ag1171_task, "ag1171_task", 2048, NULL, 10, NULL);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(SHK, ag1171_shk_isr_handler, NULL); 
}

void ring_once()
{
    gpio_set_level(RM, 1);

    int i;
    for(i=0;i<20;i++)
    {
        // 25ms * 2 = 50ms = 20Hz (ring tone freq from original Marty 1910 phones)
        gpio_set_level(FR, 0);
        vTaskDelay(25 / portTICK_RATE_MS);
        gpio_set_level(FR, 1);
        vTaskDelay(25 / portTICK_RATE_MS);
    }
    
    // ag1171 datasheet recommends wait >10ms after FR back to high
    // this is achieved above but for safety lets do it here again
    vTaskDelay(15 / portTICK_RATE_MS);

    gpio_set_level(RM, 0);
}

static volatile bool ringing = false;

void ag1171_start_ringing()
{
    ringing = true;
    xTaskCreate(ag1171_ring_task, "ag1171_ring_task", 2048, NULL, 10, NULL);
}

void ag1171_stop_ringing()
{
    ringing = false;
}

static void ag1171_ring_task()
{
    while(ringing)
    {
        gpio_set_level(RM, 1);

        int i;
        for(i=0;i<20;i++)
        {
            // 25ms * 2 = 50ms = 20Hz (ring tone freq from original Marty 1910 phones)
            gpio_set_level(FR, 0);
            if(!ringing) break; // try to exit as quickly as possible
            vTaskDelay(25 / portTICK_RATE_MS);
            gpio_set_level(FR, 1);
            if(!ringing) break; // try to exit as quickly as possible
            vTaskDelay(25 / portTICK_RATE_MS);
        }

        // ag1171 datasheet recommends wait >10ms after FR back to high
        // this is achieved above but for safety lets do it here again
        vTaskDelay(15 / portTICK_RATE_MS);

        gpio_set_level(RM, 0);

        if(!ringing) break; // try to exit as quickly as possible
        vTaskDelay(3000 / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}

void ag1171_ring_short()
{
    // ring 3 times to make sure bells are hit at least 2 times
    // as hammer might be stuck on one side
    // but since we need to go back to FR=1 at the end, needs to be even, so 4
    gpio_set_level(RM, 1);

    int i;
    for(i=0;i<4;i++)
    {
        // 25ms * 2 = 50ms = 20Hz (ring tone freq from original Marty 1910 phones)
        gpio_set_level(FR, 0);
        vTaskDelay(25 / portTICK_RATE_MS);
        gpio_set_level(FR, 1);
        vTaskDelay(25 / portTICK_RATE_MS);
    }

    gpio_set_level(RM, 0);
}
