#include "esp_adc/adc_cali.h"
#include "esp_err.h"
#include "esp_adc/adc_continuous.h"
#include "hal/adc_types.h"
#include "adc_cal.h"

#define DEFAULT_VREF 1100


static adc_continuous_handle_t handle = NULL;

adc_continuous_handle_cfg_t adc_handle;
void ADC_Initialization(){

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 4096, //DMA ring buffer
        .conv_frame_size = 1024,    //bytes delivered per read.
    };

    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));   // Popualate the handle.

    adc_continuous_config_t config =
    {   
        .sample_freq_hz = 20000,
        .pattern_num = 1,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
        .adc_pattern = (adc_digi_pattern_config_t[]){{
            .atten = ADC_ATTEN_DB_12,
            .channel   = ADC_CHANNEL_6,
            .unit      = ADC_UNIT_1,
            .bit_width = ADC_BITWIDTH_12,
        }},

    };

    ESP_ERROR_CHECK(adc_continuous_config(handle,&config));
    ESP_ERROR_CHECK(adc_continuous_start(handle));
}




bool read_ADC_continuous(float* output_buffer){

    uint8_t raw_buf[1024]; //buf to hold bytes delivered per read
    uint32_t out_length;

    //Start reading now
    esp_err_t ret = adc_continuous_read(handle,raw_buf,sizeof(raw_buf),&out_length,1000);
     
    if (ret == ESP_OK) {
        int sample_index = 0;
        // Parse the raw bytes and cast them directly to floats for YIN.
        // Each sample is sizeof(adc_digi_output_data_t) bytes (2 on ESP32), so a full
        // 1024-byte frame yields up to ADC_FRAME_SAMPLES (512) samples. Bound the loop by
        // BOTH the bytes actually read AND the output-buffer capacity so we never overflow.
        for (uint32_t i = 0;
             i + sizeof(adc_digi_output_data_t) <= out_length && sample_index < ADC_FRAME_SAMPLES;
             i += sizeof(adc_digi_output_data_t)) {
            adc_digi_output_data_t *p = (adc_digi_output_data_t*)&raw_buf[i];
            output_buffer[sample_index] = (float)(p->type1.data);
            sample_index++;
        }
        return true; // Successfully read and parsed
    }
    return false; // Read timed out or failed



}



 