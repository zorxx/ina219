/*! \copyright 2025 Zorxx Software. All rights reserved.
 *  See LICENSE file in the root of this project source tree
 */
#include <stdbool.h>
#include <stdio.h>
#include "ina219/ina219.h"

#define I2C_BUS "/dev/i2c-0"
#define DEVICE_I2C_ADDRESS   INA219_ADDR_GND_GND
#define SHUNT_RESISTANCE     1.0 // ohm

#define ERR(...) fprintf(stderr, __VA_ARGS__)
#define MSG(...) fprintf(stderr, __VA_ARGS__)

int main(int argc, char *argv[])
{
   i2c_lowlevel_config config;
   config.device = I2C_BUS;
   ina219_t ctx = ina219_init(&config, DEVICE_I2C_ADDRESS, INA219_BUS_RANGE_32V,
      INA219_GAIN_0_125, INA219_RES_12BIT_1S, INA219_RES_12BIT_1S, INA219_MODE_CONT_SHUNT_BUS);
   if(NULL == ctx)
   {
      ERR("Initialization failed\n");
   }
   else if(!ina219_calibrate(ctx, SHUNT_RESISTANCE))
   {
      ERR("Calibration failed\n");
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
            ERR("Query failed\n");
         }
         else
         {
            MSG("Bus voltage: %.2f V\n", bus_voltage);
            MSG("Shunt voltage: %.2f V\n", bus_voltage);
            MSG("Current: %.2f A\n", current);
            MSG("Power: %.2f W\n", power);
         }
         usleep(500000);
      }
      ina219_free(ctx);
   }

   MSG("Test application finished\n");
}
