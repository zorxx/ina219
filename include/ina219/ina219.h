/**
 * @file ina219.h
 * @defgroup ina219 ina219
 * @{
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
#ifndef __INA219_H__
#define __INA219_H__

#include <stdint.h>
#include <stdbool.h>

#if defined(__linux__)
   #include "ina219/sys_linux.h"
#elif defined(ESP_PLATFORM)
   #include "ina219/sys_esp.h"
#else
   #error "Supported OS type not detected"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define INA219_ADDR_GND_GND 0x40 //!< I2C address, A1 pin - GND, A0 pin - GND
#define INA219_ADDR_GND_VS  0x41 //!< I2C address, A1 pin - GND, A0 pin - VS+
#define INA219_ADDR_GND_SDA 0x42 //!< I2C address, A1 pin - GND, A0 pin - SDA
#define INA219_ADDR_GND_SCL 0x43 //!< I2C address, A1 pin - GND, A0 pin - SCL
#define INA219_ADDR_VS_GND  0x44 //!< I2C address, A1 pin - VS+, A0 pin - GND
#define INA219_ADDR_VS_VS   0x45 //!< I2C address, A1 pin - VS+, A0 pin - VS+
#define INA219_ADDR_VS_SDA  0x46 //!< I2C address, A1 pin - VS+, A0 pin - SDA
#define INA219_ADDR_VS_SCL  0x47 //!< I2C address, A1 pin - VS+, A0 pin - SCL
#define INA219_ADDR_SDA_GND 0x48 //!< I2C address, A1 pin - SDA, A0 pin - GND
#define INA219_ADDR_SDA_VS  0x49 //!< I2C address, A1 pin - SDA, A0 pin - VS+
#define INA219_ADDR_SDA_SDA 0x4a //!< I2C address, A1 pin - SDA, A0 pin - SDA
#define INA219_ADDR_SDA_SCL 0x4b //!< I2C address, A1 pin - SDA, A0 pin - SCL
#define INA219_ADDR_SCL_GND 0x4c //!< I2C address, A1 pin - SCL, A0 pin - GND
#define INA219_ADDR_SCL_VS  0x4d //!< I2C address, A1 pin - SCL, A0 pin - VS+
#define INA219_ADDR_SCL_SDA 0x4e //!< I2C address, A1 pin - SCL, A0 pin - SDA
#define INA219_ADDR_SCL_SCL 0x4f //!< I2C address, A1 pin - SCL, A0 pin - SCL

/**
 * Bus voltage range
 */
typedef enum
{
    INA219_BUS_RANGE_16V = 0, //!< 16V FSR
    INA219_BUS_RANGE_32V      //!< 32V FSR (default)
} ina219_bus_voltage_range_t;

/**
 * PGA gain for shunt voltage
 */
typedef enum
{
    INA219_GAIN_1 = 0, //!< Gain: 1, Range: +-40 mV
    INA219_GAIN_0_5,   //!< Gain: 1/2, Range: +-80 mV
    INA219_GAIN_0_25,  //!< Gain: 1/4, Range: +-160 mV
    INA219_GAIN_0_125  //!< Gain: 1/8, Range: +-320 mV (default)
} ina219_gain_t;

/**
 * ADC resolution/averaging
 */
typedef enum
{
    INA219_RES_9BIT_1S    = 0,  //!< 9 bit, 1 sample, conversion time 84 us
    INA219_RES_10BIT_1S   = 1,  //!< 10 bit, 1 sample, conversion time 148 us
    INA219_RES_11BIT_1S   = 2,  //!< 11 bit, 1 sample, conversion time 276 us
    INA219_RES_12BIT_1S   = 3,  //!< 12 bit, 1 sample, conversion time 532 us (default)
    INA219_RES_12BIT_2S   = 9,  //!< 12 bit, 2 samples, conversion time 1.06 ms
    INA219_RES_12BIT_4S   = 10, //!< 12 bit, 4 samples, conversion time 2.13 ms
    INA219_RES_12BIT_8S   = 11, //!< 12 bit, 8 samples, conversion time 4.26 ms
    INA219_RES_12BIT_16S  = 12, //!< 12 bit, 16 samples, conversion time 8.51 ms
    INA219_RES_12BIT_32S  = 13, //!< 12 bit, 32 samples, conversion time 17.02 ms
    INA219_RES_12BIT_64S  = 14, //!< 12 bit, 64 samples, conversion time 34.05 ms
    INA219_RES_12BIT_128S = 15, //!< 12 bit, 128 samples, conversion time 68.1 ms
} ina219_resolution_t;

/**
 * Operating mode
 */
typedef enum
{
    INA219_MODE_POWER_DOWN = 0, //!< Power-done
    INA219_MODE_TRIG_SHUNT,     //!< Shunt voltage, triggered
    INA219_MODE_TRIG_BUS,       //!< Bus voltage, triggered
    INA219_MODE_TRIG_SHUNT_BUS, //!< Shunt and bus, triggered
    INA219_MODE_DISABLED,       //!< ADC off (disabled)
    INA219_MODE_CONT_SHUNT,     //!< Shunt voltage, continuous
    INA219_MODE_CONT_BUS,       //!< Bus voltage, continuous
    INA219_MODE_CONT_SHUNT_BUS  //!< Shunt and bus, continuous (default)
} ina219_mode_t;

typedef void *ina219_t;

/**
 * @brief Initialize device descriptor
 * @param config OS/platform-specific configuration structure (e.g. see sys_linux.h or sys_esp.h)
 * @param i2c_address I2C slave address of INA219 device (one of INA219_ADDR_*)
 * @param u_range Bus voltage range
 * @param gain Shunt voltage gain
 * @param u_res Bus voltage resolution and averaging
 * @param i_res Shunt voltage resolution and averaging
 * @param mode Device operational mode
 * @return inas219_t on success, NULL on failure
 */
ina219_t ina219_init(i2c_lowlevel_config *config, uint8_t i2c_address,
   ina219_bus_voltage_range_t u_range, ina219_gain_t gain,
   ina219_resolution_t u_res, ina219_resolution_t i_res, ina219_mode_t mode);

/**
 * @brief Free device descriptor
 * @param ina219 obtained from a successful ina219_init() call
 * @return true on success, false on failure
 */
bool ina219_free(ina219_t ina219);

/**
 * @brief Get bus voltage range
 *
 * @param ina219 obtained from a successful ina219_init() call
 * @param[out] range Bus voltage range
 * @return true on success, false on failure
 */
bool ina219_get_bus_voltage_range(ina219_t ina219, ina219_bus_voltage_range_t *range);

/**
 * @brief Get shunt voltage gain
 *
 * @param ina219 obtained from a successful ina219_init() call
 * @param[out] gain Shunt voltage gain
 * @return true on success, false on failure
 */
bool ina219_get_gain(ina219_t ina219, ina219_gain_t *gain);

/**
 * @brief Get bus voltage resolution and averaging
 *
 * @param ina219 obtained from a successful ina219_init() call
 * @param[out] res Bus voltage resolution and averaging
 * @return true on success, false on failure
 */
bool ina219_get_bus_voltage_resolution(ina219_t ina219, ina219_resolution_t *res);

/**
 * @brief Get shunt voltage resolution and averaging
 *
 * @param ina219 obtained from a successful ina219_init() call
 * @param[out] res Shunt voltage resolution and averaging
 * @return true on success, false on failure
 */
bool ina219_get_shunt_voltage_resolution(ina219_t ina219, ina219_resolution_t *res);

/**
 * @brief Get operating mode
 *
 * @param ina219 obtained from a successful ina219_init() call
 * @param[out] mode Operating mode
 * @return true on success, false on failure
 */
bool ina219_get_mode(ina219_t ina219, ina219_mode_t *mode);

/**
 * @brief Perform calibration
 *
 * Current readings will be valid only after calibration
 *
 * @param ina219 obtained from a successful ina219_init() call
 * @param r_shunt Shunt resistance, Ohm
 * @return true on success, false on failure
 */
bool ina219_calibrate(ina219_t ina219, float r_shunt);

/**
 * @brief Trigger single conversion
 *
 * Function will return an error if current operating
 * mode is not `INA219_MODE_TRIG_SHUNT`/`INA219_MODE_TRIG_BUS`/`INA219_MODE_TRIG_SHUNT_BUS`
 *
 * @param ina219 obtained from a successful ina219_init() call
 * @return true on success, false on failure
 */
bool ina219_trigger(ina219_t ina219);

/**
 * @brief Read bus voltage
 *
 * @param ina219 obtained from a successful ina219_init() call
 * @param[out] voltage Bus voltage, V
 * @return true on success, false on failure
 */
bool ina219_get_bus_voltage(ina219_t ina219, float *voltage);

/**
 * @brief Read shunt voltage
 *
 * @param ina219 obtained from a successful ina219_init() call
 * @param[out] voltage Shunt voltage, V
 * @return true on success, false on failure
 */
bool ina219_get_shunt_voltage(ina219_t ina219, float *voltage);

/**
 * @brief Read current
 *
 * This function works properly only after calibration.
 *
 * @param ina219 obtained from a successful ina219_init() call
 * @param[out] current Current, A
 * @return true on success, false on failure
 */
bool ina219_get_current(ina219_t ina219, float *current);

/**
 * @brief Read power
 *
 * This function works properly only after calibration.
 *
 * @param ina219 obtained from a successful ina219_init() call
 * @param[out] power Power, W
 * @return true on success, false on failure
 */
bool ina219_get_power(ina219_t ina219, float *power);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __INA219_H__ */
