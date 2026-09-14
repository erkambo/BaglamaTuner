#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"

#include "adc_cal.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SAMPLING_RATE 20000
#define YIN_MAX_LAG  250      
#define YIN_WINDOW_SIZE  300 
#define BUF_SIZE 1024

// ADC_FRAME_SAMPLES comes from adc_cal.h (= 512 on ESP32)

float yin_buffer[YIN_MAX_LAG];
float sin_buffer[BUF_SIZE];



float yin_algorithm(float *buffer) {
    yin_buffer[0] = 1.0f; 
    for(int i = 1; i < YIN_MAX_LAG; i++) {
        yin_buffer[i] = 0.0f;
    }

    // Step 1: Difference Function
    // Tau is our time shift. We compare the signal within the window with a time shifted bersion of itself.
    // If tau equals the period of the wave, the difference will be very close to zero (as they line up perfectly)
    for(int tau = 1; tau < YIN_MAX_LAG; tau++){
        for(int i = 0; i < YIN_WINDOW_SIZE; i++){
            float delta = (buffer[i] - buffer[i+tau]);
            yin_buffer[tau] += (delta * delta);
        } 
    }

    // Step 2: CMNDF (Cumulative mean normalized difference)
    // We wil lnormalize the Difference Function by dividing by the average over shorter lag values
    float running_sum = 0.0f;
    for (int tau = 1; tau < YIN_MAX_LAG; tau++){ 
        running_sum += yin_buffer[tau];
        yin_buffer[tau] = yin_buffer[tau] * tau / running_sum; 
    }
    
    // Step 3: Absolute Thresholding
    // We have errors due to perfect periodicity. This leads to Octave errors To fix this, set a small threshold of 0.1 
    // and choose the smallest lag value below this threshold
    int period = 0;
    for (int tau = 1; tau < YIN_MAX_LAG - 1; tau++) {
        if(yin_buffer[tau] < 0.1f && yin_buffer[tau + 1] > yin_buffer[tau]) {
            period = tau;   // save tge index.
            break;
        }
    }

    if (period == 0) return 0.0f;

    // Step 4: Parabolic Interpolation (Limits quantization error)
    // We shift the discrete signal sample by sample tofind the pitch. However, we can have half samples, quarter samples, etc.
    // In that case, we can use interpolation to estimate the frequency by lookign at the neighbours.
    float s0 = yin_buffer[period - 1];
    float s1 = yin_buffer[period];
    float s2 = yin_buffer[period + 1];

    float adjustment = 0.5f * (s0 - s2) / (s0 - 2.0f * s1 + s2);
    float exact_period = (float)period + adjustment;

    return (float)SAMPLING_RATE / exact_period;
}

//ESP32 I2S DMA will be here. Goal is to read the pitch of noise using the YIN algorithm and a circular buffer.

// DSP Task - Pinned to Core 1
void core1_task(void *pvParam){ 
    ADC_Initialization(); //init adc

    float audio_buffer[ADC_FRAME_SAMPLES];

    while(1) {
        if(read_ADC_continuous(audio_buffer)){
            //print sample from array:
            char line[64];
            snprintf(line, sizeof(line), "Core 1 ADC Sample: %d", (int)audio_buffer[0]);
            puts(line);
        }
    }
}

// Display/Peripheral Task - Pinned to Core 0
void core0_task(void *pvParam){ 
    while(1){
        printf("Core 0: Awaiting OLED Integration...\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void) {
    printf("Initializing DSP Tuner Pipeline...\n");
    xTaskCreatePinnedToCore(core1_task, "DSP_Task", 8192, NULL, 1, NULL, 1);    //1 and 0 are for core ids.
    xTaskCreatePinnedToCore(core0_task, "Display_Task", 2048, NULL, 1, NULL, 0);
}