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

#ifndef PROJ_ADC_H
#define PROJ_ADC_H

static uint8_t ADS_ADDR = 0x48;
static uint8_t ADS_Config_Pointer = 0x01;
static uint8_t ADS_Conversion_Pointer = 0x00;
static uint8_t ADS_Conf1 = 0x42;  //00000010b
static uint8_t ADS_Conf2 = 0xE3; //11100000b;

static esp_err_t I2C_INIT(i2c_port_t i2c_num, i2c_config_t i2c_conf)
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
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, ADS_ADDR << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, ADS_Config_Pointer, I2C_MASTER_ACK));
    uint8_t ads_config[2] = {0x43, 0xE3};
    ESP_ERROR_CHECK(i2c_master_write(cmd, ads_config, 2, I2C_MASTER_ACK));
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
    i2c_master_write_byte(cmd, i2c_address << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, reg_address, I2C_MASTER_ACK);
    i2c_master_write(cmd, data, data_len, I2C_MASTER_ACK);
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
    i2c_master_write_byte(cmd, i2c_address << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, reg_address, I2C_MASTER_ACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, i2c_address << 1 | I2C_MASTER_READ, I2C_MASTER_ACK);
    i2c_master_read(cmd, data, data_len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}
#endif