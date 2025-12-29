/*! \copyright 2025 Zorxx Software. All rights reserved.
 *  \brief ina219 library esp-idf example application
 *  See LICENSE file in the root of this project source tree
 */
#include "ina219/ina219.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "INA219"

/* The following definitions may change, based on the ESP device,
   RTC device configuration, and wiring between them. */
#define ESP_I2C_PORT         I2C_NUM_0
#define ESP_I2C_SDA          GPIO_NUM_21
#define ESP_I2C_SCL          GPIO_NUM_22
#define DEVICE_I2C_ADDRESS   INA219_ADDR_GND_GND
#define SHUNT_RESISTANCE     1.0 // ohm

void app_main(void)
{
   ESP_ERROR_CHECK(nvs_flash_init());
   ESP_ERROR_CHECK(esp_event_loop_create_default() );

   i2c_lowlevel_config config = {0};
   config.port = ESP_I2C_PORT;
   config.pin_sda = ESP_I2C_SDA;
   config.pin_scl = ESP_I2C_SCL;
   ina219_t ctx = ina219_init(&config, DEVICE_I2C_ADDRESS, INA219_BUS_RANGE_32V,
      INA219_GAIN_0_125, INA219_RES_12BIT_1S, INA219_RES_12BIT_1S, INA219_MODE_CONT_SHUNT_BUS);
   if(NULL == ctx)
   {
      ESP_LOGE(TAG, "Initialization failed");
   }
   else if(!ina219_calibrate(ctx, SHUNT_RESISTANCE))
   {
      ESP_LOGE(TAG, "Calibration failed");
   }
   else
   {
      float bus_voltage, shunt_voltage, current, power;

      for(int i = 0; i < 100; ++i)
      {
         if(!ina219_get_bus_voltage(ctx, &bus_voltage)
         || !ina219_get_shunt_voltage(ctx, &shunt_voltage)
         || !ina219_get_current(ctx, &current)
         || !ina219_get_power(ctx, &power))
         {
            ESP_LOGE(TAG, "Query failed");
         }
         else
         {
            ESP_LOGI(TAG, "Bus voltage: %.2f V", bus_voltage);
            ESP_LOGI(TAG, "Shunt voltage: %.2f V", bus_voltage);
            ESP_LOGI(TAG, "Current: %.2f A", current);
            ESP_LOGI(TAG, "Power: %.2f W", power);
         }
         vTaskDelay(pdMS_TO_TICKS(500));
      }
      ina219_free(ctx);
   }

   ESP_LOGI(TAG, "Test application finished");

   for(;;)
      vTaskDelay(portMAX_DELAY);
}
