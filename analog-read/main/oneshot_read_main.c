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
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

//-----------------------------------------------------------------------------
const static char *TAG = "EXAMPLE";

#define ECHO_TEST_TXD (CONFIG_EXAMPLE_UART_TXD)
#define ECHO_TEST_RXD (CONFIG_EXAMPLE_UART_RXD)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)
#define ECHO_UART_PORT_NUM      (CONFIG_EXAMPLE_UART_PORT_NUM)
#define ECHO_UART_BAUD_RATE     (CONFIG_EXAMPLE_UART_BAUD_RATE)
#define ECHO_TASK_STACK_SIZE    (CONFIG_EXAMPLE_TASK_STACK_SIZE)
#define BUF_SIZE (1024)
//-----------------------------------------------------------------------------
// ADC1 Channels
#define EXAMPLE_ADC_ATTEN ADC_ATTEN_DB_12
static adc_channel_t adc_channels[] = {0, 1, 2, 3, 4, 5, 6};
static const size_t num_channels = sizeof(adc_channels) / sizeof(adc_channel_t);
static const size_t num_mux_channels = 7;
static int adc_raw[49];
// static const size_t num_sensors = sizeof(adc_raw) / sizeof(int);

//-----------------------------------------------------------------------------

static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
//-----------------------------------------------------------------------------
static void analog_read_task(void *arg)
{
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    gpio_reset_pin(20);    
    gpio_set_direction(20, GPIO_MODE_OUTPUT);
    gpio_reset_pin(22);    
    gpio_set_direction(22, GPIO_MODE_OUTPUT);
    gpio_reset_pin(12);    
    gpio_set_direction(12, GPIO_MODE_OUTPUT);
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


    char data[100];
    memset(data,'#',100);

    while (1)
    {
        for (int mux = 0; mux < num_mux_channels; mux++)
        {
            // switch mux channel
            gpio_set_level(20, (mux >> 0) & 0x1);               
            gpio_set_level(22, (mux >> 1) & 0x1);               
            gpio_set_level(12, (mux >> 2) & 0x1);                       

            for (int adc = 0; adc < num_channels; adc++)
            {
                int i = mux + (adc * num_channels);
                
                int total = 0;
                for (size_t av = 0; av < 3; av++)
                {
                    int temp;
                    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, adc_channels[adc], &temp));
                    total += temp;
                }
                adc_raw[i] = total / 3;
            }
        }
        
        uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) data, 100);
        
        num_terations++;

        if (num_terations == max_num_terations)
        {
            vTaskDelay(1);
            num_terations = 0;
            int64_t duration = (esp_timer_get_time() - start_time) / 1000ll;
            ESP_LOGI(TAG, "%d Iterations: %lldms",max_num_terations, duration);
            ESP_LOGI(TAG, "Single Iteration: %lldms", duration / max_num_terations);
            start_time = esp_timer_get_time();
        }
    }
}
//-----------------------------------------------------------------------------

void app_main(void)
{
    xTaskCreate(analog_read_task, "analog_read_task", 4096, NULL, 2, NULL);
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
