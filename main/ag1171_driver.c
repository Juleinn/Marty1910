#include "ag1171_driver.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#define RM 23
#define FR 22
#define SHK 34

#define GPIO_OUTPUT_PIN_SEL ((1ULL << RM) | (1ULL << FR))
#define GPIO_INPUT_PIN_SEL ((1ULL << SHK))

void ag1171_init()
{
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

    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);
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
