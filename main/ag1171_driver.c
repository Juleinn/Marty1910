#include "ag1171_driver.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define RM 23
#define FR 22
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

static void ag1171_task(void* arg)
{
    uint32_t value;
    long lastTickCount = xTaskGetTickCount();
    #define DEBOUNCE_TIME_MS 30
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &value, portMAX_DELAY)) {
           
            if((xTaskGetTickCount() - lastTickCount) * portTICK_RATE_MS >= DEBOUNCE_TIME_MS)
            {
                value = gpio_get_level(SHK); 
                vTaskDelay(DEBOUNCE_TIME_MS/portTICK_RATE_MS); // debounce 15ms
                if(value == gpio_get_level(SHK))
                {
                    if(value)
                    {
                        printf("Phone off-hook\n");
                    }
                    else
                    {
                        printf("Phone on-hook\n");
                    }
                }
            }
            else
            {
                // do nothing actually
            }
            lastTickCount = xTaskGetTickCount();
        }
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

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
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
        vTaskDelay(50 / portTICK_RATE_MS);
        gpio_set_level(FR, 1);
        vTaskDelay(50 / portTICK_RATE_MS);
    }
    
    // ag1171 datasheet recommends wait >10ms after FR back to high
    // this is achieved above but for safety lets do it here again
    vTaskDelay(15 / portTICK_RATE_MS);

    gpio_set_level(RM, 0);
}

void ag1171_ring_loop()
{
    while(1)
    {
        ring_once();
        vTaskDelay(3000 / portTICK_RATE_MS);
    }
}
