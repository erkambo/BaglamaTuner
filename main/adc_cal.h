#pragma once

#include <stdbool.h>
#include "esp_adc/adc_continuous.h"   // for adc_digi_output_data_t

// A full conversion frame is conv_frame_size = 1024 bytes.
// Each ESP32 sample is sizeof(adc_digi_output_data_t) bytes (2 on the ESP32,
// since SOC_ADC_DIGI_RESULT_BYTES == 2), so a full frame yields up to 512 samples.
// Buffers that receive a frame MUST be at least this many elements.
#define ADC_FRAME_SAMPLES (1024 / sizeof(adc_digi_output_data_t))

void ADC_Initialization(void);
bool read_ADC_continuous(float *output_buffer);
