#ifndef LINE_H
#define LINE_H

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

uint32_t line_sample_AC_mV();
void line_init();

void line_connect(bool connect);

bool line_is_connected();
bool line_is_cranking();

#endif
