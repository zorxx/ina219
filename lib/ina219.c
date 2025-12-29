/**
 * @file ina219.c
 *
 * ESP-IDF driver for INA219/INA220 Zerø-Drift, Bidirectional
 * Current/Power Monitor
 *
 * ported from esp-idf-lib
 *
 * Copyright 2025 Zorxx Software
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include "helpers.h"
#include "ina219/ina219.h"
#include "sys.h"
#include <malloc.h>
#include <string.h>
#include <math.h>

#define I2C_TRANSFER_TIMEOUT  50 /* (milliseconds) give up on i2c transaction after this timeout */
#define I2C_FREQ_HZ           1000000  /* Supports up to 2.56 MHz */

#define INA219_REG_CONFIG          0
#define INA219_REG_SHUNT_VOLTAGE   1
#define INA219_REG_BUS_VOLTAGE     2
#define INA219_REG_POWER           3
#define INA219_REG_CURRENT         4
#define INA219_REG_CALIBRATION     5

/* Config register bits/masks */
#define INA219_CFG_BIT_RST   15
#define INA219_CFG_BIT_BRNG  13
#define INA219_CFG_BIT_PG0   11
#define INA219_CFG_BIT_BADC0 7
#define INA219_CFG_BIT_SADC0 3
#define INA219_CFG_BIT_MODE  0
#define INA219_CFG_MASK_PG   (3   << INA219_CFG_BIT_PG0)
#define INA219_CFG_MASK_BADC (0xf << INA219_CFG_BIT_BADC0)
#define INA219_CFG_MASK_SADC (0xf << INA219_CFG_BIT_SADC0)
#define INA219_CFG_MASK_MODE (7   << INA219_CFG_BIT_MODE)
#define INA219_CFG_MASK_BRNG (1   << INA219_CFG_BIT_BRNG)

static const float u_shunt_max[] =
{
    [INA219_GAIN_1]     = 0.04,
    [INA219_GAIN_0_5]   = 0.08,
    [INA219_GAIN_0_25]  = 0.16,
    [INA219_GAIN_0_125] = 0.32,
};

typedef struct
{
   i2c_lowlevel_config i2c_config;
   i2c_lowlevel_context i2c_ctx;
   ina219_bus_voltage_range_t bus_range;
   ina219_gain_t gain;
   ina219_resolution_t bus_res;
   ina219_resolution_t shunt_res;
   ina219_mode_t mode;
   float r_shunt;
   float i_lsb;
   float p_lsb;
   bool calibrated;
} ina219_context_t;

static __inline bool ina219_read(ina219_context_t *ctx, uint8_t reg, uint16_t *result)
{
   uint8_t data[2];
   if(!i2c_ll_read_reg(ctx->i2c_ctx, reg, data, sizeof(data)))
      return false;
   *result = (uint16_t) (data[0] << 8) | (data[1] >> 8);
   return true;
}

static __inline bool ina219_write(ina219_context_t *ctx, uint8_t reg, uint16_t value)
{
   uint8_t data[2] = { value >> 8, value };
   return i2c_ll_write_reg(ctx->i2c_ctx, reg, data, sizeof(data));
}

static __inline bool ina219_get_config_bits(ina219_context_t *ctx, uint16_t mask, uint8_t bit, uint16_t *result)
{
   uint16_t value;
   if(!ina219_read(ctx, INA219_REG_CONFIG, &value))
      return false;

   *result = (value & mask) >> bit;
   return true;
}

#define ina219_get_config(ctx) \
 ((((uint16_t)((ctx)->bus_range)) << INA219_CFG_BIT_BRNG)  |  \
  (((uint16_t)((ctx)->gain))      << INA219_CFG_BIT_PG0)   |  \
  (((uint16_t)((ctx)->bus_res))   << INA219_CFG_BIT_BADC0) |  \
  (((uint16_t)((ctx)->shunt_res)) << INA219_CFG_BIT_SADC0) |  \
  (((uint16_t)((ctx)->mode))      << INA219_CFG_BIT_MODE))

/* --------------------------------------------------------------------------------------------------------
 * Exported Functions
 */

ina219_t ina219_init(i2c_lowlevel_config *config, uint8_t i2c_address,
   ina219_bus_voltage_range_t bus_range, ina219_gain_t gain,
   ina219_resolution_t bus_res, ina219_resolution_t shunt_res, ina219_mode_t mode)
{
   ina219_context_t *ctx;

   if(i2c_address < INA219_ADDR_GND_GND || i2c_address > INA219_ADDR_SCL_SCL)
   {
      SERR("[%s] Invalid I2C address 0x%02x", __func__, i2c_address);
      return false;
   }

   if(bus_range > INA219_BUS_RANGE_32V
   || gain      > INA219_GAIN_0_125
   || bus_res   > INA219_RES_12BIT_128S
   || shunt_res > INA219_RES_12BIT_128S
   || mode      > INA219_MODE_CONT_SHUNT_BUS)
   {
      SERR("[%s] Invalid device configuration", __func__);
      return false;
   }

   ctx = (ina219_context_t *) malloc(sizeof(*ctx));
   if(NULL == ctx)
      return NULL;
   memset(ctx, 0, sizeof(*ctx));

   ctx->i2c_ctx = i2c_ll_init(i2c_address, I2C_FREQ_HZ, I2C_TRANSFER_TIMEOUT, config);
   if(NULL == ctx->i2c_ctx)
   {
      SERR("[%s] i2c initialization failed", __func__);
   }
   else
   {
      ctx->bus_range = bus_range;
      ctx->gain = gain;
      ctx->bus_res = bus_res;
      ctx->shunt_res = shunt_res;
      ctx->mode = mode;
      if(!ina219_write(ctx, INA219_REG_CONFIG, ina219_get_config(ctx)))
      {
         SERR("[%s] Failed to write device configuration", __func__);
      }
      else
      {
         return ctx;  // success
      }
   }

   free(ctx);
   return NULL; // failed
}

bool ina219_free(ina219_t ina219)
{
   ina219_context_t *ctx = (ina219_context_t *) ina219;
   if(NULL == ctx)
      return false;
   free(ctx);
   return true;
}

bool ina219_calibrate(ina219_t ina219, float r_shunt)
{
   ina219_context_t *ctx = (ina219_context_t *) ina219;
   uint16_t cal;

   ctx->i_lsb = (uint16_t)(u_shunt_max[ctx->gain] / r_shunt / 32767 * 100000000);
   ctx->i_lsb /= 100000000;
   ctx->i_lsb /= 0.0001;
   ctx->i_lsb = ceil(ctx->i_lsb);
   ctx->i_lsb *= 0.0001;
   ctx->p_lsb = ctx->i_lsb * 20;
   cal = (uint16_t)((0.04096) / (ctx->i_lsb * r_shunt));

   if(!ina219_write(ctx, INA219_REG_CALIBRATION, cal))
   {
      SERR("[%s] Failed to write calibration", __func__);
      ctx->calibrated = false;
   }
   ctx->r_shunt = r_shunt;
   ctx->calibrated = true;

   return true;
}

bool ina219_trigger(ina219_t ina219)
{
   ina219_context_t *ctx = (ina219_context_t *) ina219;

   if(ctx->mode < INA219_MODE_TRIG_SHUNT || ctx->mode > INA219_MODE_TRIG_SHUNT_BUS)
   {
       SERR("[%s] Could not trigger conversion in this mode: %d", __func__, ctx->mode);
       return false;
   }

   return ina219_write(ctx, INA219_REG_CONFIG, ina219_get_config(ctx));
}

bool ina219_get_bus_voltage(ina219_t ina219, float *voltage)
{
   ina219_context_t *ctx = (ina219_context_t *) ina219;
   uint16_t value;

   if(!ina219_read(ctx, INA219_REG_BUS_VOLTAGE, &value))
   {
      SERR("[%s] Failed to read", __func__);
      return false;
   }

   *voltage = (value >> 3) * 0.004;
   return true;
}

bool ina219_get_shunt_voltage(ina219_t ina219, float *voltage)
{
   ina219_context_t *ctx = (ina219_context_t *) ina219;
   uint16_t value;

   if(!ina219_read(ctx, INA219_REG_SHUNT_VOLTAGE, &value))
   {
      SERR("[%s] Failed to read", __func__);
      return false;
   }

   *voltage = value / 100000.0;
   return true;
}

bool ina219_get_current(ina219_t ina219, float *current)
{
   ina219_context_t *ctx = (ina219_context_t *) ina219;
   uint16_t value;

   if(!ctx->calibrated)
      return false;

   if(!ina219_read(ctx, INA219_REG_CURRENT, &value))
   {
      SERR("[%s] Failed to read", __func__);
      return false;
   }

   *current = value * ctx->i_lsb;
   return true;
}

bool ina219_get_power(ina219_t ina219, float *power)
{
   ina219_context_t *ctx = (ina219_context_t *) ina219;
   uint16_t value;

   if(!ctx->calibrated)
      return false;

   if(!ina219_read(ctx, INA219_REG_POWER, &value))
   {
      SERR("[%s] Failed to read", __func__);
      return false;
   }

   *power = value * ctx->p_lsb;
   return true;
}
