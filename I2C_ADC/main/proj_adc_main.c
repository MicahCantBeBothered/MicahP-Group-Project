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
#include "proj_adc.h"

/*static uint8_t ADS_ADDR = 0x48;
static uint8_t ADS_Config_Pointer = 0x01;
static uint8_t ADS_Conversion_Pointer = 0x00;
static uint8_t ADS_Conf1 = 0x42;  //00000010b
static uint8_t ADS_Conf2 = 0xE3; //11100000b;*/

static const char *TAG = "ADC";
static const char *TAG1 = "I2C";

/*static esp_err_t I2C_INIT(i2c_port_t i2c_num, i2c_config_t i2c_conf)
{
    ESP_ERROR_CHECK(i2c_driver_install(i2c_num, i2c_conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_num, &i2c_conf));
    return ESP_OK;
}
static esp_err_t ADC_SETUP()
{
    int ret;
    i2c_config_t ads;
    ads.mode = I2C_MODE_MASTER;
    ads.sda_io_num = 0x02;
    ads.sda_pullup_en = 1;
    ads.scl_io_num = 0x00;
    ads.scl_pullup_en = 1;
    ads.clk_stretch_tick = 300;
    I2C_INIT(I2C_NUM_0, ads);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, ADS_ADDR << 1 | I2C_MASTER_WRITE, 0x1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, ADS_Config_Pointer, 0x1));
    uint8_t ads_config[2] = {0x43, 0xE3};
    ESP_ERROR_CHECK(i2c_master_write(cmd, ads_config, 2, 0x1));
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/ portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    vTaskDelay(1000/portTICK_RATE_MS);
    return ret;
}

static esp_err_t i2c_write(i2c_port_t i2c_num, uint8_t i2c_address, uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, i2c_address << 1 | I2C_MASTER_WRITE, 0x1);
    i2c_master_write_byte(cmd, reg_address, 0x1);
    i2c_master_write(cmd, data, data_len, 0x1);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t i2c_read(i2c_port_t i2c_num, uint8_t i2c_address, uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, i2c_address << 1 | I2C_MASTER_WRITE, 0x1);
    i2c_master_write_byte(cmd, reg_address, 0x1);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, i2c_address << 1 | I2C_MASTER_READ, 0x1);
    i2c_master_read(cmd, data, data_len, 0x2);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}*/

/*static void ADC_Conversion (uint8_t adc_out[])
{
    uint8_t data_out[2];
    i2c_read(I2C_NUM_0, ADS_ADDR, ADS_Conversion_Pointer, data_out, 2);
    uint16_t adc_data = data_out[0] << 8 | data_out[1];
    ESP_LOGI(TAG1, "ADC Output: %d\n", adc_data);
    return;
}*/

static void ADC_Task()
{
    int iter = 0;
    ADC_SETUP();
    uint8_t data_out [2];
    i2c_read(I2C_NUM_0, ADS_ADDR, ADS_Conversion_Pointer, data_out, 2);
    while (iter < 10)
        {
        ESP_LOGI(TAG1, "********************\n");
        ESP_LOGI(TAG1, "ADC Output: %d. \n", data_out[0] + data_out[1]);
        ESP_LOGI(TAG1, "********************\n");
        }
    iter = 0;
}

void app_main(void)
{
    xTaskCreate(ADC_Task, "ADC_Task", 2048, NULL, 10, NULL);
    vTaskDelay(500 / portTICK_RATE_MS);
    //esp_restart();
}