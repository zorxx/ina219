# ina219 Library
This library provides support for the ina219 voltage/current/power sensor

This library currently supports the following platforms:
* esp-idf
* linux

Add this component to an esp-idf project with the following command:
```bash
idf.py add-dependency "zorxx/ina219"
```

Source for this project may be found at [https://github.com/zorxx/ina219](https://github.com/zorxx/ina219).

# Usage

The API for this library can be found in the `include/ina219.h` header file.

Fully-functional example applications for each supported platform can be found in the `example` directory.

Example esp-idf code-snippet:
```c
#include "ina219/ina219.h"
i2c_lowlevel_config config;
config.port = I2C_NUM_0;
config.pin_sda = GPIO_NUM_21;
config.pin_scl = GPIO_NUM_22;
ina219_t ctx = ina219_init(&config, INA219_ADDR_GND_GND, INA219_BUS_RANGE_32V,
   INA219_GAIN_0_125, INA219_RES_12BIT_1S, INA219_RES_12BIT_1S, INA219_MODE_CONT_SHUNT_BUS);
if(NULL != ctx)
{
   float bus_voltage, shunt_voltage;
   if(ina219_get_bus_voltage(ctx, &bus_voltage)
   && ina219_get_shunt_voltage(ctx, &shunt_voltage))
   {
      ESP_LOGI(TAG, "Bus: %.2f V, shunt: %.2f V", bus_voltage, shunt_voltage);
   }
   ina219_free(ctx);
}
```

Example linux code-snippet:
```c
#include "ina219/ina219.h"
i2c_lowlevel_config config;
config.device = "/dev/i2c-0";
ina219_t ctx = ina219_init(&config, INA219_ADDR_GND_GND, INA219_BUS_RANGE_32V,
   INA219_GAIN_0_125, INA219_RES_12BIT_1S, INA219_RES_12BIT_1S, INA219_MODE_CONT_SHUNT_BUS);
if(NULL != ctx)
{
   float bus_voltage, shunt_voltage;
   if(ina219_get_bus_voltage(ctx, &bus_voltage)
   && ina219_get_shunt_voltage(ctx, &shunt_voltage))
   {
      fprintf(stderr, "Bus: %.2f V, shunt: %.2f V", bus_voltage, shunt_voltage);
   }
   ina219_free(ctx);
}
```

# License
All files delivered with this library are released under the BSD license. See the `LICENSE` file for details.
