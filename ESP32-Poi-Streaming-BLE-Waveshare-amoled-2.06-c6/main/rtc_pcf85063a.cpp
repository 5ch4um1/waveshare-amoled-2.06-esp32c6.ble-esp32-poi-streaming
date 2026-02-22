#include "rtc_pcf85063a.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include <string.h> // For memcpy
#include <stdio.h>  // For snprintf

static const char *TAG = "RTC_PCF85063A";

// RTC registers
#define PCF85063A_REG_CONTROL_1     0x00
#define PCF85063A_REG_CONTROL_2     0x01
#define PCF85063A_REG_OFFSET        0x02
#define PCF85063A_REG_RAM_BYTE      0x03
#define PCF85063A_REG_SECONDS       0x04
#define PCF85063A_REG_MINUTES       0x05
#define PCF85063A_REG_HOURS         0x06
#define PCF85063A_REG_DAYS          0x07
#define PCF85063A_REG_WEEKDAYS      0x08
#define PCF85063A_REG_MONTHS        0x09
#define PCF85063A_REG_YEARS         0x0A
#define PCF85063A_REG_MINUTE_ALARM  0x0B
#define PCF85063A_REG_HOUR_ALARM    0x0C
#define PCF85063A_REG_DAY_ALARM     0x0D
#define PCF85063A_REG_WEEKDAY_ALARM 0x0E
#define PCF85063A_REG_TIMER_VALUE   0x0F
#define PCF85063A_REG_TIMER_MODE    0x10

// BCD conversion helpers
static uint8_t bcd2bin(uint8_t val) { return val - 6 * (val >> 4); }
static uint8_t bin2bcd(uint8_t val) { return val + 6 * (val / 10); }

static i2c_master_dev_handle_t rtc_dev_handle = NULL;

esp_err_t rtc_pcf85063a_init(i2c_master_bus_handle_t i2c_bus_handle) {
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = RTC_PCF85063A_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ, // Assuming I2C_MASTER_FREQ_HZ is defined globally
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = 0
        }
    };
    esp_err_t ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_config, &rtc_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add PCF85063A device to I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // Reset the RTC (optional, but good for consistent start)
    uint8_t reg_val = 0x58; // Set to 0x58 to ensure proper operation after reset
    ret = i2c_master_transmit_receive(rtc_dev_handle, (uint8_t[]){PCF85063A_REG_CONTROL_1}, 1, &reg_val, 1, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write Control_1 register: %s", esp_err_to_name(ret));
        return ret;
    }

    // Clear the Stop bit (bit 5 of Control_1) to start the oscillator
    // Read Control_1, clear bit 5, write back
    uint8_t control1;
    ret = i2c_master_transmit_receive(rtc_dev_handle, (uint8_t[]){PCF85063A_REG_CONTROL_1}, 1, &control1, 1, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read Control_1 register for stop bit: %s", esp_err_to_name(ret));
        return ret;
    }
    control1 &= ~(1 << 5); // Clear OS (Oscillator Stop) bit
    ret = i2c_master_transmit_receive(rtc_dev_handle, (uint8_t[]){PCF85063A_REG_CONTROL_1}, 1, &control1, 1, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear stop bit in Control_1 register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "PCF85063A RTC initialized successfully.");
    return ESP_OK;
}

esp_err_t rtc_pcf85063a_get_time(struct tm *timeinfo) {
    if (!rtc_dev_handle) {
        return ESP_FAIL;
    }

    uint8_t data[7]; // Seconds, Minutes, Hours, Days, Weekdays, Months, Years
    uint8_t reg_addr = PCF85063A_REG_SECONDS;
    esp_err_t ret = i2c_master_transmit_receive(rtc_dev_handle, &reg_addr, 1, data, 7, 1000);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read time from PCF85063A: %s", esp_err_to_name(ret));
        return ret;
    }

    timeinfo->tm_sec = bcd2bin(data[0] & 0x7F); // Mask out OS bit
    timeinfo->tm_min = bcd2bin(data[1] & 0x7F);
    timeinfo->tm_hour = bcd2bin(data[2] & 0x3F); // Mask out 24/12 hour bit
    timeinfo->tm_mday = bcd2bin(data[3] & 0x3F);
    timeinfo->tm_wday = bcd2bin(data[4] & 0x07); // Weekday 0-6
    timeinfo->tm_mon = bcd2bin(data[5] & 0x1F) - 1; // Months 0-11
    timeinfo->tm_year = bcd2bin(data[6]) + 100; // Years since 2000, tm_year is years since 1900

    timeinfo->tm_isdst = -1; // Not known

    return ESP_OK;
}

esp_err_t rtc_pcf85063a_set_time(const struct tm *timeinfo) {
    if (!rtc_dev_handle) {
        return ESP_FAIL;
    }

    uint8_t data[7]; // Seconds, Minutes, Hours, Days, Weekdays, Months, Years
    data[0] = bin2bcd(timeinfo->tm_sec);
    data[1] = bin2bcd(timeinfo->tm_min);
    data[2] = bin2bcd(timeinfo->tm_hour);
    data[3] = bin2bcd(timeinfo->tm_mday);
    data[4] = bin2bcd(timeinfo->tm_wday);
    data[5] = bin2bcd(timeinfo->tm_mon + 1); // Months 1-12
    data[6] = bin2bcd(timeinfo->tm_year % 100); // Years since 2000

    uint8_t write_buffer[8];
    write_buffer[0] = PCF85063A_REG_SECONDS;
    memcpy(&write_buffer[1], data, 7);

    esp_err_t ret = i2c_master_transmit(rtc_dev_handle, write_buffer, 8, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write time to PCF85063A: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "PCF85063A RTC time set successfully.");
    return ESP_OK;
}