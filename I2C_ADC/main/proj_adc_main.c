#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "esp_log.h"

static uint8_t ADS_ADDR = 0x90;
static uint8_t ADS_Config_Pointer = 0x01;
static uint8_t ADS_Conversion_Pointer = 0x00;
static uint8_t ADS_Conf1 = 0x02;  //00000010b
static uint8_t ADS_Conf2 = 0xE0; //11100000b;

static const char *TAG = "ADC";

static esp_err_t I2C_INIT()
{
    i2c_config_t ADS1115conf;
    ADS1115conf.mode = I2C_MODE_MASTER;
    ADS1115conf.sda_io_num = GPIO_NUM_2;
    ADS1115conf.sda_pullup_en = 0x1;
    ADS1115conf.scl_io_num = GPIO_NUM_0;
    ADS1115conf.scl_pullup_en = 0x1;
    ADS1115conf.clk_stretch_tick = 300;
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, ADS1115conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &ADS1115conf));
    return ESP_OK;
}
static esp_err_t ADC_READ(i2c_port_t i2c_num, uint8_t Target, uint8_t Data)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS_ADDR << 1 | I2C_MASTER_WRITE, 0x1);
    i2c_master_write_byte(cmd, ADS_Conversion_Pointer, 0x1);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK)
    {
        return ret;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS_ADDR << 1 | I2C_MASTER_READ, 0x1);
    i2c_master_read_byte(cmd, Data, 0x2);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t ADC_SETUP()
{
    int ret;
    vTaskDelay(100/portTICK_RATE_MS);
    I2C_INIT();
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS_ADDR << 1 | I2C_MASTER_WRITE, 0x1);
    i2c_master_write_byte(cmd, ADS_Config_Pointer, 0x1);
    i2c_master_write_byte(cmd, ADS_Conf1, 0x1);
    i2c_master_write_byte(cmd, ADS_Conf2, 0x1);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/ portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static void ADC_Task()
{

    uint8_t data_out;
    double temp;
    int ret; 
    ret = ADC_SETUP();

    while(1)
    {
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "ADS1115 Detected\n");
            //ESP_LOGI(TAG, "ADC");
        }
        else
        {
            ESP_LOGI(TAG, "ADS1115 Not Detected\n");
        }
    }
    vTaskDelay(1000 / portTICK_RATE_MS);

    //Testing how ESP_LOGI function
    
    /*while (1)
    {
        ESP_LOGI(TAG, "***************\n");
        ESP_LOGI(TAG, "*******1*******\n");
        ESP_LOGI(TAG, "*******2*******\n");
        ESP_LOGI(TAG, "*******3*******\n");
        ESP_LOGI(TAG, "*******4*******\n");
        ESP_LOGI(TAG, "*******5*******\n");
        ESP_LOGI(TAG, "***************\n");
        vTaskDelay(100/portTICK_RATE_MS);
    }*/
    
}

void app_main(void)
{
    xTaskCreate(ADC_Task, "ADC_Task", 2048, NULL, 10, NULL);
}