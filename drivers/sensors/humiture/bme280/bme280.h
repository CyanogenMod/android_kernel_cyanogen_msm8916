/*!
 * @section LICENSE
 * (C) Copyright 2013 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename bme280.h
 * @date     2014/02/13
 * @id       "7f466fb"
 *
 * @brief
 * Header of BME280 API
 *
 * Revision: 1.1(Pressure and Temperature compensation code revision is 1.1
 *               and Humidity compensation code revision is 1.0)
*/
/****************************************************************************/
#ifndef __BME280_H__
#define __BME280_H__
#if defined(__KERNEL__)
#include <linux/kernel.h>
#include <linux/math64.h>
#define BME280_U8_t  u8
#define BME280_S8_t  s8
#define BME280_U16_t u16
#define BME280_S16_t s16
#define BME280_U32_t u32
#define BME280_S32_t s32
#define BME280_U64_t u64
#define BME280_S64_t s64
#define BME280_64BITSUPPORT_PRESENT

#else
#include <limits.h>

#define BME280_U8_t unsigned char
#define BME280_S8_t signed char
/* Find correct data type for signed/unsigned 16 bit variables by
		checking max of unsigned variant */
#if USHRT_MAX == 0xFFFF
/* 16 bit achieved with short */
#define BME280_U16_t unsigned short
#define BME280_S16_t signed short
#elif UINT_MAX == 0xFFFF
/* 16 bit achieved with int */
#define BME280_U16_t unsigned int
#define BME280_S16_t signed int
#else
#error BME280_U16_t and BME280_S16_t could not be \
defined automatically, please do so manually
#endif

/* Find correct data type for 32 bit variables */
#if INT_MAX == 0x7FFFFFFF
/* 32 bit achieved with int */
#define BME280_U32_t unsigned int
#define BME280_S32_t signed int
#elif LONG_MAX == 0x7FFFFFFF
/* 32 bit achieved with long int */
#define BME280_U32_t unsigned long int
#define BME280_S32_t signed long int
#else
#error BME280_S32_t and BME280_U32_t could not be \
defined automatically, please do so manually
#endif

/* Find correct data type for 64 bit variables */
#if defined(LONG_MAX) && LONG_MAX == 0x7fffffffffffffffL
#define BME280_S64_t long int
#define BME280_64BITSUPPORT_PRESENT
#elif defined(LLONG_MAX) && (LLONG_MAX == 0x7fffffffffffffffLL)
#define BME280_S64_t long long int
#define BME280_64BITSUPPORT_PRESENT
#else
#warning Either the correct data type for signed 64 bit integer \
could not be found, or 64 bit integers are not supported in your environment.
#warning The API will only offer 32 bit pressure calculation.This will \
slightly impede accuracy(noise of ~1 pascal RMS will be added to output).
#warning If 64 bit integers are supported on your platform, \
please set BME280_S64_t manually and
#define(BME280_64BITSUPPORT_PRESENT) manually.
#endif
#endif/*__KERNEL__*/



/* If the user wants to support floating point calculations, please set
		the following #define. If floating point
		calculation is not wanted or allowed
		(e.g. in Linux kernel), please do not set the define. */
#if 0
#define BME280_ENABLE_FLOAT
#endif
/* If the user wants to support 64 bit integer calculation
		(needed for optimal pressure accuracy) please set
		the following #define. If int64 calculation is not wanted
		(e.g. because it would include
		large libraries), please do not set the define. */
#define BME280_ENABLE_INT64

/** defines the return parameter type of the BME280_WR_FUNCTION */
#define BME280_BUS_WR_RETURN_TYPE signed char

/**\brief links the order of parameters defined in
BME280_BUS_WR_PARAM_TYPE to function calls used inside the API*/
#define BME280_BUS_WR_PARAM_TYPES unsigned char, unsigned char,\
		unsigned char *, unsigned char

/**\brief links the order of parameters defined in
BME280_BUS_WR_PARAM_TYPE to function calls used inside the API*/
#define BME280_BUS_WR_PARAM_ORDER(device_addr, register_addr,\
		register_data, wr_len)

/* never change this line */
#define BME280_BUS_WRITE_FUNC(device_addr, register_addr,\
register_data, wr_len) bus_write(device_addr, register_addr,\
		register_data, wr_len)

/**\brief defines the return parameter type of the BME280_RD_FUNCTION
*/
#define BME280_BUS_RD_RETURN_TYPE signed char

/**\brief defines the calling parameter types of the BME280_RD_FUNCTION
*/
#define BME280_BUS_RD_PARAM_TYPES (unsigned char, unsigned char,\
		unsigned char *, unsigned char)

/**\brief links the order of parameters defined in \
BME280_BUS_RD_PARAM_TYPE to function calls used inside the API
*/
#define BME280_BUS_RD_PARAM_ORDER (device_addr, register_addr,\
		register_data)

/* never change this line */
#define BME280_BUS_READ_FUNC(device_addr, register_addr,\
		register_data, rd_len)bus_read(device_addr, register_addr,\
		register_data, rd_len)

/**\brief defines the return parameter type of the BME280_DELAY_FUNCTION
*/
#define BME280_DELAY_RETURN_TYPE void

/**\brief defines the calling parameter types of the BME280_DELAY_FUNCTION
*/
#define BME280_DELAY_PARAM_TYPES BME280_U16_t

/* never change this line */
#define BME280_DELAY_FUNC(delay_in_msec)\
		delay_func(delay_in_msec)

#define BME280_GET_BITSLICE(regvar, bitname)\
		((regvar & bitname##__MSK) >> bitname##__POS)

#define BME280_SET_BITSLICE(regvar, bitname, val)\
((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))


/* Constants */
#define BME280_NULL                          0
#define BME280_RETURN_FUNCTION_TYPE          signed char

#define SHIFT_RIGHT_4_POSITION                           4
#define SHIFT_LEFT_2_POSITION                2
#define SHIFT_LEFT_4_POSITION                4
#define SHIFT_LEFT_5_POSITION                5
#define SHIFT_LEFT_8_POSITION                8
#define SHIFT_LEFT_12_POSITION               12
#define SHIFT_LEFT_16_POSITION               16
#define BME280_Four_U8X                      4
#define BME280_Zero_U8X                      0
#define BME280_Eight_U8X                     8
#define	SUCCESS						(unsigned char)0

#define E_BME280_NULL_PTR                    ((signed char)-127)
#define E_BME280_COMM_RES                    ((signed char)-1)
#define E_BME280_OUT_OF_RANGE                ((signed char)-2)

#define BME280_I2C_ADDRESS1                  0x76
#define BME280_I2C_ADDRESS2                  0x77
#define BME280_I2C_ADDRESS                   BME280_I2C_ADDRESS2

/* Sensor Specific constants */
#define BME280_SLEEP_MODE                    0x00
#define BME280_FORCED_MODE                   0x01
#define BME280_NORMAL_MODE                   0x03
#define BME280_SOFT_RESET_CODE               0xB6

#define BME280_STANDBYTIME_1_MS              0x00
#define BME280_STANDBYTIME_63_MS             0x01
#define BME280_STANDBYTIME_125_MS			 0x02
#define BME280_STANDBYTIME_250_MS            0x03
#define BME280_STANDBYTIME_500_MS            0x04
#define BME280_STANDBYTIME_1000_MS           0x05
#define BME280_STANDBYTIME_10_MS             0x06
#define BME280_STANDBYTIME_20_MS             0x07

#define BME280_OVERSAMPLING_SKIPPED          0x00
#define BME280_OVERSAMPLING_1X               0x01
#define BME280_OVERSAMPLING_2X               0x02
#define BME280_OVERSAMPLING_4X               0x03
#define BME280_OVERSAMPLING_8X               0x04
#define BME280_OVERSAMPLING_16X              0x05

/*#define BME280_ULTRALOWPOWER_MODE            0x00
#define BME280_LOWPOWER_MODE                 0x01
#define BME280_STANDARDRESOLUTION_MODE       0x02
#define BME280_HIGHRESOLUTION_MODE           0x03
#define BME280_ULTRAHIGHRESOLUTION_MODE      0x04

#define BME280_ULTRALOWPOWER_OSRS_P          BME280_OVERSAMPLING_1X
#define BME280_ULTRALOWPOWER_OSRS_T          BME280_OVERSAMPLING_1X

#define BME280_LOWPOWER_OSRS_P               BME280_OVERSAMPLING_2X
#define BME280_LOWPOWER_OSRS_T               BME280_OVERSAMPLING_1X

#define BME280_STANDARDRESOLUTION_OSRS_P     BME280_OVERSAMPLING_4X
#define BME280_STANDARDRESOLUTION_OSRS_T     BME280_OVERSAMPLING_1X

#define BME280_HIGHRESOLUTION_OSRS_P         BME280_OVERSAMPLING_8X
#define BME280_HIGHRESOLUTION_OSRS_T         BME280_OVERSAMPLING_1X

#define BME280_ULTRAHIGHRESOLUTION_OSRS_P    BME280_OVERSAMPLING_16X
#define BME280_ULTRAHIGHRESOLUTION_OSRS_T    BME280_OVERSAMPLING_2X */

#define BME280_STANDARD_OSRS_H	BME280_OVERSAMPLING_1X
#define BME280_FILTERCOEFF_OFF               0x00
#define BME280_FILTERCOEFF_2                 0x01
#define BME280_FILTERCOEFF_4                 0x02
#define BME280_FILTERCOEFF_8                 0x03
#define BME280_FILTERCOEFF_16                0x04

#define T_INIT_MAX                             20
		/* 20/16 = 1.25 ms */
#define T_MEASURE_PER_OSRS_MAX                          37
		/* 37/16 = 2.3125 ms*/

#define T_SETUP_PRESSURE_MAX                            10
		/* 10/16 = 0.625 ms */

#define T_SETUP_HUMIDITY_MAX                            10
		/* 10/16 = 0.625 ms */

/*calibration parameters */
#define BME280_DIG_T1_LSB_REG                0x88
#define BME280_DIG_T1_MSB_REG                0x89
#define BME280_DIG_T2_LSB_REG                0x8A
#define BME280_DIG_T2_MSB_REG                0x8B
#define BME280_DIG_T3_LSB_REG                0x8C
#define BME280_DIG_T3_MSB_REG                0x8D
#define BME280_DIG_P1_LSB_REG                0x8E
#define BME280_DIG_P1_MSB_REG                0x8F
#define BME280_DIG_P2_LSB_REG                0x90
#define BME280_DIG_P2_MSB_REG                0x91
#define BME280_DIG_P3_LSB_REG                0x92
#define BME280_DIG_P3_MSB_REG                0x93
#define BME280_DIG_P4_LSB_REG                0x94
#define BME280_DIG_P4_MSB_REG                0x95
#define BME280_DIG_P5_LSB_REG                0x96
#define BME280_DIG_P5_MSB_REG                0x97
#define BME280_DIG_P6_LSB_REG                0x98
#define BME280_DIG_P6_MSB_REG                0x99
#define BME280_DIG_P7_LSB_REG                0x9A
#define BME280_DIG_P7_MSB_REG                0x9B
#define BME280_DIG_P8_LSB_REG                0x9C
#define BME280_DIG_P8_MSB_REG                0x9D
#define BME280_DIG_P9_LSB_REG                0x9E
#define BME280_DIG_P9_MSB_REG                0x9F

#define BME280_DIG_H1_REG                    0xA1

#define BME280_DIG_H2_LSB_REG                0xE1
#define BME280_DIG_H2_MSB_REG                0xE2
#define BME280_DIG_H3_REG                    0xE3
#define BME280_DIG_H4_MSB_REG                0xE4
#define BME280_DIG_H5_LSB_H4_LSB_REG         0xE5
#define BME280_DIG_H5_MSB_REG                0xE6
#define BME280_DIG_H6_REG                    0xE7


#define BME280_CRC_REG                       0xE8

#define BME280_CHIPID_REG                    0xD0  /*Chip ID Register */
#define BME280_RESET_REG                     0xE0  /*Softreset Register */
#define BME280_STATUS_REG                    0xF3  /*Status Register */
#define BME280_CTRLMEAS_REG                  0xF4  /*Ctrl Measure Register */
#define BME280_CTRLHUM_REG                   0xF2  /*Ctrl Humidity Register*/
#define BME280_CONFIG_REG                    0xF5  /*Configuration Register */
#define BME280_PRESSURE_MSB_REG              0xF7  /*Pressure MSB Register */
#define BME280_PRESSURE_LSB_REG              0xF8  /*Pressure LSB Register */
#define BME280_PRESSURE_XLSB_REG             0xF9  /*Pressure XLSB Register */
#define BME280_TEMPERATURE_MSB_REG           0xFA  /*Temperature MSB Reg */
#define BME280_TEMPERATURE_LSB_REG           0xFB  /*Temperature LSB Reg */
#define BME280_TEMPERATURE_XLSB_REG          0xFC  /*Temperature XLSB Reg */
#define BME280_HUMIDITY_MSB_REG              0xFD  /*Humidity MSB Reg */
#define BME280_HUMIDITY_LSB_REG              0xFE  /*Humidity LSB Reg */

/* Status Register */
#define BME280_STATUS_REG_MEASURING__POS           3
#define BME280_STATUS_REG_MEASURING__MSK           0x08
#define BME280_STATUS_REG_MEASURING__LEN           1
#define BME280_STATUS_REG_MEASURING__REG           BME280_STATUS_REG

#define BME280_STATUS_REG_IMUPDATE__POS            0
#define BME280_STATUS_REG_IMUPDATE__MSK            0x01
#define BME280_STATUS_REG_IMUPDATE__LEN            1
#define BME280_STATUS_REG_IMUPDATE__REG            BME280_STATUS_REG

/* Control Measurement Register */
#define BME280_CTRLMEAS_REG_OSRST__POS             5
#define BME280_CTRLMEAS_REG_OSRST__MSK             0xE0
#define BME280_CTRLMEAS_REG_OSRST__LEN             3
#define BME280_CTRLMEAS_REG_OSRST__REG             BME280_CTRLMEAS_REG

#define BME280_CTRLMEAS_REG_OSRSP__POS             2
#define BME280_CTRLMEAS_REG_OSRSP__MSK             0x1C
#define BME280_CTRLMEAS_REG_OSRSP__LEN             3
#define BME280_CTRLMEAS_REG_OSRSP__REG             BME280_CTRLMEAS_REG

#define BME280_CTRLMEAS_REG_MODE__POS              0
#define BME280_CTRLMEAS_REG_MODE__MSK              0x03
#define BME280_CTRLMEAS_REG_MODE__LEN              2
#define BME280_CTRLMEAS_REG_MODE__REG              BME280_CTRLMEAS_REG

#define BME280_CTRLHUM_REG_OSRSH__POS             0
#define BME280_CTRLHUM_REG_OSRSH__MSK             0x07
#define BME280_CTRLHUM_REG_OSRSH__LEN             3
#define BME280_CTRLHUM_REG_OSRSH__REG             BME280_CTRLHUM_REG

/* Configuration Register */
#define BME280_CONFIG_REG_TSB__POS                 5
#define BME280_CONFIG_REG_TSB__MSK                 0xE0
#define BME280_CONFIG_REG_TSB__LEN                 3
#define BME280_CONFIG_REG_TSB__REG                 BME280_CONFIG_REG

#define BME280_CONFIG_REG_FILTER__POS              2
#define BME280_CONFIG_REG_FILTER__MSK              0x1C
#define BME280_CONFIG_REG_FILTER__LEN              3
#define BME280_CONFIG_REG_FILTER__REG              BME280_CONFIG_REG

#define BME280_CONFIG_REG_SPI3WEN__POS             0
#define BME280_CONFIG_REG_SPI3WEN__MSK             0x01
#define BME280_CONFIG_REG_SPI3WEN__LEN             1
#define BME280_CONFIG_REG_SPI3WEN__REG             BME280_CONFIG_REG

/* Data Register */
#define BME280_PRESSURE_XLSB_REG_DATA__POS         4
#define BME280_PRESSURE_XLSB_REG_DATA__MSK         0xF0
#define BME280_PRESSURE_XLSB_REG_DATA__LEN         4
#define BME280_PRESSURE_XLSB_REG_DATA__REG         BME280_PRESSURE_XLSB_REG

#define BME280_TEMPERATURE_XLSB_REG_DATA__POS      4
#define BME280_TEMPERATURE_XLSB_REG_DATA__MSK      0xF0
#define BME280_TEMPERATURE_XLSB_REG_DATA__LEN      4
#define BME280_TEMPERATURE_XLSB_REG_DATA__REG      BME280_TEMPERATURE_XLSB_REG

#define BME280_WR_FUNC_PTR\
		char (*bus_write)(unsigned char, unsigned char,\
		unsigned char *, unsigned char)

#define BME280_RD_FUNC_PTR\
		char (*bus_read)(unsigned char, unsigned char,\
		unsigned char *, unsigned char)

#define BME280_MDELAY_DATA_TYPE BME280_U16_t
/** this structure holds all device specific calibration parameters */
struct bme280_calibration_param_t {
	BME280_U16_t dig_T1;
	BME280_S16_t dig_T2;
	BME280_S16_t dig_T3;
	BME280_U16_t dig_P1;
	BME280_S16_t dig_P2;
	BME280_S16_t dig_P3;
	BME280_S16_t dig_P4;
	BME280_S16_t dig_P5;
	BME280_S16_t dig_P6;
	BME280_S16_t dig_P7;
	BME280_S16_t dig_P8;
	BME280_S16_t dig_P9;

	BME280_U8_t  dig_H1;
	BME280_S16_t dig_H2;
	BME280_U8_t  dig_H3;
	BME280_S16_t dig_H4;
	BME280_S16_t dig_H5;
	BME280_S8_t  dig_H6;

	BME280_S32_t t_fine;
};
/** BME280 image registers data structure */
struct bme280_t {
	struct bme280_calibration_param_t cal_param;

	unsigned char chip_id;
	unsigned char dev_addr;

	unsigned char osrs_t;
	unsigned char osrs_p;
	unsigned char osrs_h;
	unsigned char crc;

	BME280_WR_FUNC_PTR;
	BME280_RD_FUNC_PTR;
	void(*delay_msec)(BME280_MDELAY_DATA_TYPE);
};
/* Function Declarations */
BME280_RETURN_FUNCTION_TYPE bme280_init(struct bme280_t *bme280);

BME280_RETURN_FUNCTION_TYPE bme280_read_ut(BME280_S32_t *utemperature);

BME280_S32_t bme280_compensate_T_int32(BME280_S32_t adc_t);

BME280_RETURN_FUNCTION_TYPE bme280_read_up(BME280_S32_t *upressure);

BME280_U32_t bme280_compensate_P_int32(BME280_S32_t adc_p);

BME280_RETURN_FUNCTION_TYPE bme280_read_uh(BME280_S32_t *uhumidity);

BME280_U32_t bme280_compensate_H_int32(BME280_S32_t adc_h);

BME280_RETURN_FUNCTION_TYPE bme280_read_uputuh(BME280_S32_t *upressure,
BME280_S32_t *utemperature, BME280_S32_t *uhumidity);

BME280_RETURN_FUNCTION_TYPE bme280_read_pth(BME280_U32_t *pressure,
BME280_S32_t *temperature, BME280_U32_t *humidity);

BME280_RETURN_FUNCTION_TYPE bme280_get_calib_param(void);

BME280_RETURN_FUNCTION_TYPE bme280_get_osrs_t(unsigned char *value);

BME280_RETURN_FUNCTION_TYPE bme280_set_osrs_t(unsigned char value);

BME280_RETURN_FUNCTION_TYPE bme280_get_osrs_p(unsigned char *value);

BME280_RETURN_FUNCTION_TYPE bme280_set_osrs_p(unsigned char value);

BME280_RETURN_FUNCTION_TYPE bme280_get_osrs_h(unsigned char *value);

BME280_RETURN_FUNCTION_TYPE bme280_set_osrs_h(unsigned char value);

BME280_RETURN_FUNCTION_TYPE bme280_get_mode(unsigned char *mode);

BME280_RETURN_FUNCTION_TYPE bme280_set_mode(unsigned char mode);

BME280_RETURN_FUNCTION_TYPE bme280_set_softreset(void);

BME280_RETURN_FUNCTION_TYPE bme280_get_spi3(unsigned char *enable_disable);

BME280_RETURN_FUNCTION_TYPE bme280_set_spi3(unsigned char enable_disable);

BME280_RETURN_FUNCTION_TYPE bme280_get_filter(unsigned char *value);

BME280_RETURN_FUNCTION_TYPE bme280_set_filter(unsigned char value);

BME280_RETURN_FUNCTION_TYPE bme280_get_standbydur(unsigned char *time);

BME280_RETURN_FUNCTION_TYPE bme280_set_standbydur(unsigned char time);

/*BME280_RETURN_FUNCTION_TYPE bme280_set_workmode(unsigned char mode);*/
BME280_RETURN_FUNCTION_TYPE bme280_get_forced_uputuh(BME280_S32_t *upressure,
BME280_S32_t *utemperature, BME280_S32_t *uhumidity);

BME280_RETURN_FUNCTION_TYPE bme280_write_register(unsigned char addr,
unsigned char *data, unsigned char len);

BME280_RETURN_FUNCTION_TYPE bme280_read_register(unsigned char addr,
unsigned char *data, unsigned char len);

#ifdef BME280_ENABLE_FLOAT
double bme280_compensate_T_double(BME280_S32_t adc_t);

double bme280_compensate_P_double(BME280_S32_t adc_p);

double bme280_compensate_H_double(BME280_S32_t adc_h);
#endif
#if defined(BME280_ENABLE_INT64) && defined(BME280_64BITSUPPORT_PRESENT)
BME280_U32_t bme280_compensate_P_int64(BME280_S32_t adc_p);
#endif
BME280_RETURN_FUNCTION_TYPE bme280_compute_wait_time(unsigned char
*v_delaytime_u8r);
#endif
