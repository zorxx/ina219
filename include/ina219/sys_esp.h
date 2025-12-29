/*! \copyright 2025 Zorxx Software. All rights reserved.
 *  \brief esp-idf lowlevel portability interface
 *  See LICENSE file in the root of this project source tree
 */
#ifndef _SYS_ESP_IDF_H
#define _SYS_ESP_IDF_H

#include "hal/i2c_types.h"
#include "driver/i2c_master.h"

typedef struct i2c_lowlevel_s
{
   /* If bus == NULL, port, pin_sda, and pin_scl will be used to
      initialize the I2C bus, otherwise it's assumed that a previous
      call was made to i2c_new_master_bus, and the resulting handle
      is assigned to this 'bus' variable.  */
   i2c_master_bus_handle_t *bus;

   /* If bus != NULL, the following variables will be used to
      initialize the I2C bus. */
   i2c_port_t port;
   int pin_sda;
   int pin_scl;
} i2c_lowlevel_config;

#endif /* _SYS_ESP_IDF_H */
