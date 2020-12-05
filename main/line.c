#include "line.h"

#if CONFIG_IDF_TARGET_ESP32
static const adc_channel_t channel = ADC_CHANNEL_5;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
#elif CONFIG_IDF_TARGET_ESP32S2
static const adc_channel_t channel = ADC_CHANNEL_5;     // GPIO7 if ADC1, GPIO17 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_13;
#endif
static const adc_atten_t atten = ADC_ATTEN_DB_11;

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   54          //Multisampling

// calibration curve parameters
static esp_adc_cal_characteristics_t adc_chars;

#define RELAY_1         26
#define RELAY_PIN_SEL  ((1ULL<<RELAY_1))

static void line_init_relays()
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
}

static uint32_t line_AC_mV;

static void line_relay_task()
{
    while(1)
    {
        line_AC_mV = line_sample_AC_mV();
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

static bool line_connected = false;

void line_connect(bool connect)
{
    if(connect) 
    {
        printf("Line connect !!\n");
        gpio_set_level(RELAY_1, 1);
        line_connected = true;
    }
    else 
    {
        gpio_set_level(RELAY_1, 0);
        line_connected = false;
    }
}

bool line_is_connected()
{
    return line_connected;
}

#define CRANK_THRESHOLD_mV 600

bool line_is_cranking()
{
    return line_sample_AC_mV() > CRANK_THRESHOLD_mV; 
}

void line_init()
{
    adc1_config_width(width);
    adc1_config_channel_atten(channel, atten);

    //Characterize ADC
    esp_adc_cal_characterize(ADC_UNIT_1, atten, width, DEFAULT_VREF, &adc_chars);

    line_init_relays();

    xTaskCreate(line_relay_task, "line_relay_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
}

uint32_t line_sample_AC_mV()
{
    uint32_t adc_reading = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        adc_reading += adc1_get_raw((adc1_channel_t)channel);
    }
    adc_reading /= NO_OF_SAMPLES;
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars);

    return voltage;
}

