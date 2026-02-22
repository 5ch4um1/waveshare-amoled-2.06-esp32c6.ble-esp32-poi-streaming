#ifndef RTC_PCF85063A_H
#define RTC_PCF85063A_H

#include <time.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

// PMU I2C Config (copying from main.cpp to ensure I2C_MASTER_FREQ_HZ is defined)
#ifndef CONFIG_I2C_MASTER_FREQUENCY
#define CONFIG_I2C_MASTER_FREQUENCY 400000
#endif
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY
#define I2C_MASTER_TIMEOUT_MS 1000 // Also copied, as it's used in the driver

#ifdef __cplusplus
extern "C" {
#endif

// RTC I2C address
#define RTC_PCF85063A_ADDR 0x51

// Function to initialize the PCF85063A RTC
esp_err_t rtc_pcf85063a_init(i2c_master_bus_handle_t i2c_bus_handle);

// Function to get time from the PCF85063A RTC
esp_err_t rtc_pcf85063a_get_time(struct tm *timeinfo);

// Function to set time to the PCF85063A RTC (useful for initial setup)
esp_err_t rtc_pcf85063a_set_time(const struct tm *timeinfo);

#ifdef __cplusplus
}
#endif

#endif // RTC_PCF85063A_H