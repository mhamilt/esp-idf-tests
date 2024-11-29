/*
 * Simple Analog Read
 */
//-----------------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <esp_log.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_timer.h>
//-----------------------------------------------------------------------------
const static char *TAG = "EXAMPLE";
//-----------------------------------------------------------------------------
// ADC1 Channels
#define EXAMPLE_ADC_ATTEN ADC_ATTEN_DB_12
static adc_channel_t adc_channels[] = {0, 2, 3, 4, 5, 6};
static const size_t num_channels = sizeof(adc_channels) / sizeof(adc_channel_t);
//-----------------------------------------------------------------------------
static int adc_raw[14];
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
//-----------------------------------------------------------------------------
void app_main(void)
{
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };

    adc_cali_handle_t adc1_cali_handles[num_channels];
    bool do_calibration1[num_channels];

    for (int i = 0; i < num_channels; i++)
    {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, adc_channels[i], &config));
        adc1_cali_handles[i] = NULL;
        do_calibration1[i] = example_adc_calibration_init(ADC_UNIT_1, adc_channels[i], EXAMPLE_ADC_ATTEN, adc1_cali_handles + i);
    }

    size_t num_terations = 0;
    size_t max_num_terations = 300;
    int64_t start_time = esp_timer_get_time();
    while (1)
    {        
        

        for (int j = 0; j < num_channels; j++)
        {
            for (int i = 0; i < num_channels; i++)
            {
                for (size_t av = 0; av < 3; av++)
                {
                    int temp;
                    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, adc_channels[i], &temp));
                    adc_raw[i + num_channels * j] += temp;
                }
                adc_raw[i + num_channels * j] /= 3;
            }
        }

        memset(adc_raw, 0, sizeof(adc_raw));
        num_terations++;

        if (num_terations == max_num_terations)
        {
            vTaskDelay(1);
            num_terations = 0;
            int64_t duration = (esp_timer_get_time() - start_time) / 1000ll;
            ESP_LOGI(TAG, "100 Iterations: %lldms", duration);
            ESP_LOGI(TAG, "Single Iteration: %lldms", duration/max_num_terations);
            start_time = esp_timer_get_time();
        }
    }
}

//-----------------------------------------------------------------------------
//        ADC Calibration
//-----------------------------------------------------------------------------
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Calibration Success");
    }
    else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated)
    {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    }
    else
    {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}
