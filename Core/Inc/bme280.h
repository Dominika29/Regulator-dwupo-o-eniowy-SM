/**
 ******************************************************************************
 * @file            : bme280.h
 * @brief          : BME280 Temperature, Pressure, and Humidity Sensor Library
 * @author         : Siratul Islam
 * @version        : 1.0.0
 * @date           : May 30, 2025
 * @license        : MIT License
 * 
 * @description    : STM32 HAL-based driver for Bosch BME280 environmental sensor
 *                   Supports I2C communication with configurable oversampling,
 *                   filtering, and operating modes. Includes official Bosch
 *                   compensation algorithms for accurate measurements.
 * 
 * Copyright (c) 2025 Siratul Islam
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 ******************************************************************************
 */

#ifndef BME280_H_
#define BME280_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/
typedef struct {
	unsigned short dig_T1;
	short dig_T2;
	short dig_T3;
	unsigned short dig_P1;
	short dig_P2;
	short dig_P3;
	short dig_P4;
	short dig_P5;
	short dig_P6;
	short dig_P7;
	short dig_P8;
	short dig_P9;
	uint8_t dig_H1;
	short dig_H2;
	uint8_t dig_H3;
	short dig_H4;
	short dig_H5;
	char dig_H6;
} BME280_Calibration_TypeDef;

typedef struct {
	uint32_t pressure;    // 20-bit raw value
	uint32_t temperature; // 20-bit raw value
	uint16_t humidity;    // 16-bit raw value
} BME280_RawData_TypeDef;

typedef long signed int BME280_S32_t;
typedef long unsigned int BME280_U32_t;
typedef long long signed int BME280_S64_t;

/* Exported constants --------------------------------------------------------*/
// BME280 Register Addresses
#define BME280_I2C_ADDRESS  		 (0x76 << 1)
#define BME280_REG_CHIP_ID           0xD0
#define BME280_REG_RESET             0xE0
#define BME280_REG_CTRL_HUMIDITY     0xF2
#define BME280_REG_STATUS            0xF3
#define BME280_REG_CTRL_MEASURE      0xF4
#define BME280_REG_CONFIG            0xF5
#define BME280_REG_PRESSURE_MSB      0xF7
#define BME280_REG_CALIB_TEMP_PRESS  0x88
#define BME280_REG_CALIB_HUMIDITY 	 0xE1

// BME280 Oversampling Settings
#define BME280_OVERSAMPLE_SKIP       0x00
#define BME280_OVERSAMPLE_X1         0x01
#define BME280_OVERSAMPLE_X2         0x02
#define BME280_OVERSAMPLE_X4         0x03
#define BME280_OVERSAMPLE_X8         0x04
#define BME280_OVERSAMPLE_X16        0x05

// BME280 Operating Modes
#define BME280_SLEEP_MODE            0x00
#define BME280_FORCED_MODE           0x01
#define BME280_NORMAL_MODE           0x03

// BME280 Standby Time Settings (in ms)
#define BME280_STANDBY_0_5MS         0x00
#define BME280_STANDBY_62_5MS        0x01
#define BME280_STANDBY_125MS         0x02
#define BME280_STANDBY_250MS         0x03
#define BME280_STANDBY_500MS         0x04
#define BME280_STANDBY_1000MS        0x05
#define BME280_STANDBY_10MS          0x06
#define BME280_STANDBY_20MS          0x07

// BME280 IIR Filter Coefficients
#define BME280_FILTER_OFF            0x00
#define BME280_FILTER_COEFF_2        0x01
#define BME280_FILTER_COEFF_4        0x02
#define BME280_FILTER_COEFF_8        0x03
#define BME280_FILTER_COEFF_16       0x04


/* Exported function prototypes ----------------------------------------------*/
void read_sensor_id(void);
void read_calibration_data(void);
void read_sensor_data(void);
int bme280_init(I2C_HandleTypeDef *hi2c, uint8_t humidity_oversampling, uint8_t temp_oversampling,
		uint8_t pressure_oversampling, uint8_t sensor_mode,
		uint8_t standby_time, uint8_t filter_coeff);
void bme280_set_i2c_handle(I2C_HandleTypeDef *hi2c);
float bme280_get_temperature(void);
float bme280_get_pressure(void);
float bme280_get_humidity(void);
BME280_S32_t BME280_compensate_T_int32(BME280_S32_t adc_T);
BME280_U32_t BME280_compensate_P_int64(BME280_S32_t adc_P);
BME280_U32_t bme280_compensate_H_int32(BME280_S32_t adc_H);

#ifdef __cplusplus
}
#endif

#endif /* BME280_H_ */
