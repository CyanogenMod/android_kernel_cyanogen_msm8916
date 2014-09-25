/*!
 * @section LICENSE
 * @license$
 *
 * @filename bme280.c
 * @date     2014/02/13
 * @id       "7f466fb"
 *
 * @brief
 * API for accessing the BME280 sensor
 *
 * Revision: 1.1(Pressure and Temperature compensation code revision is 1.1
 *               and Humidity compensation code revision is 1.0)
*/
/****************************************************************************/
#include "bme280.h"
static struct bme280_t *p_bme280;                      /**< pointer to BME280 */
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief routine to initialize the function pointers
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_init(struct bme280_t *bme280)
{
	BME280_RETURN_FUNCTION_TYPE comres = BME280_Zero_U8X;
	unsigned char v_data_u8r = BME280_Zero_U8X;
	p_bme280 = bme280;
	/* assign BME280 ptr */
	p_bme280->dev_addr = BME280_I2C_ADDRESS;
	/* preset BME280 I2C_addr */
	comres += p_bme280->BME280_BUS_READ_FUNC(p_bme280->dev_addr,
	BME280_CHIPID_REG, &v_data_u8r, 1);
	/* read Chip Id */
	p_bme280->chip_id = v_data_u8r;

	bme280_get_calib_param();
	/* readout bme280 calibparam structure */
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief reads uncompensated temperature
 *
 *
 *
 *
 *  \param signed long utemperature : Pointer holding the
 *                                 uncompensated temperature.
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_read_ut(BME280_S32_t *utemperature)
{
	BME280_RETURN_FUNCTION_TYPE comres = BME280_Zero_U8X;
	unsigned char a_data_u8r[3] = {0, 0, 0};
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			comres += p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_TEMPERATURE_MSB_REG, a_data_u8r, 3);
			*utemperature = (BME280_S32_t)(((
			(BME280_U32_t) (a_data_u8r[0]))
			<< SHIFT_LEFT_12_POSITION) |
			(((BME280_U32_t)(a_data_u8r[1]))
			<< SHIFT_LEFT_4_POSITION)
			| ((BME280_U32_t)a_data_u8r[2] >>
			SHIFT_RIGHT_4_POSITION));
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Reads actual temperature from uncompensated temperature
 *                    and returns the value in 0.01 degree Centigrade
 *                    Output value of "5123" equals 51.23 DegC.
 *
 *
 *
 *  \param signed long : value of uncompensated temperature
 *
 *
 *
 *  \return
 *			signed long : actual temperature
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_S32_t bme280_compensate_T_int32(BME280_S32_t adc_t)
{
	BME280_S32_t v_x1_u32r = BME280_Zero_U8X;
	BME280_S32_t v_x2_u32r = BME280_Zero_U8X;
	BME280_S32_t temperature = BME280_Zero_U8X;

	v_x1_u32r  = ((((adc_t >> 3) - ((BME280_S32_t)
	p_bme280->cal_param.dig_T1 << 1))) *
	((BME280_S32_t)p_bme280->cal_param.dig_T2)) >> 11;
	v_x2_u32r  = (((((adc_t >> 4) -
	((BME280_S32_t)p_bme280->cal_param.dig_T1)) * ((adc_t >> 4) -
	((BME280_S32_t)p_bme280->cal_param.dig_T1))) >> 12) *
	((BME280_S32_t)p_bme280->cal_param.dig_T3)) >> 14;
	p_bme280->cal_param.t_fine = v_x1_u32r + v_x2_u32r;
	temperature  = (p_bme280->cal_param.t_fine * 5 + 128) >> 8;

	return temperature;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief reads uncompensated pressure
 *
 *
 *
 *
 *  \param  signed long upressure : Pointer holding the
 *                                 uncompensated pressure.
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_read_up(BME280_S32_t *upressure)
{
	BME280_RETURN_FUNCTION_TYPE comres = BME280_Zero_U8X;
	unsigned char a_data_u8r[3] = {0, 0, 0};
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			comres += p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_PRESSURE_MSB_REG, a_data_u8r, 3);
			*upressure = (BME280_S32_t)((
			((BME280_U32_t)(a_data_u8r[0]))
			<< SHIFT_LEFT_12_POSITION) |
			(((BME280_U32_t)(a_data_u8r[1]))
			<< SHIFT_LEFT_4_POSITION) |
			((BME280_U32_t)a_data_u8r[2] >>
			SHIFT_RIGHT_4_POSITION));
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Reads actual pressure from uncompensated pressure
 *							and returns the value in Pascal(Pa)
 *                          Output value of "96386" equals 96386 Pa =
 *                          963.86 hPa = 963.86 millibar
 *
 *
 *
 *  \param signed long : value of uncompensated pressure
 *
 *
 *
 *  \return
 *			unsigned long : actual pressure
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
unsigned int bme280_compensate_P_int32(BME280_S32_t adc_p)
{
	BME280_S32_t v_x1_u32r = BME280_Zero_U8X;
	BME280_S32_t v_x2_u32r = BME280_Zero_U8X;
	BME280_U32_t pressure = BME280_Zero_U8X;

	v_x1_u32r = (((BME280_S32_t)p_bme280->cal_param.t_fine) >> 1) -
	(BME280_S32_t)64000;
	v_x2_u32r = (((v_x1_u32r >> 2) * (v_x1_u32r >> 2)) >> 11) *
	((BME280_S32_t)p_bme280->cal_param.dig_P6);
	v_x2_u32r = v_x2_u32r + ((v_x1_u32r *
	((BME280_S32_t)p_bme280->cal_param.dig_P5)) << 1);
	v_x2_u32r = (v_x2_u32r >> 2) +
	(((BME280_S32_t)p_bme280->cal_param.dig_P4) << 16);
	v_x1_u32r = (((p_bme280->cal_param.dig_P3 * (((v_x1_u32r >> 2) *
	(v_x1_u32r >> 2)) >> 13)) >> 3) +
	((((BME280_S32_t)p_bme280->cal_param.dig_P2) *
	v_x1_u32r) >> 1)) >> 18;
	v_x1_u32r = ((((32768+v_x1_u32r)) *
	((BME280_S32_t)p_bme280->cal_param.dig_P1))	>> 15);
	if (v_x1_u32r == 0)
		return 0;
		/* Avoid exception caused by division by zero */
	pressure = (((BME280_U32_t)(((BME280_S32_t)1048576) - adc_p) -
	(v_x2_u32r >> 12))) * 3125;
	if (pressure < 0x80000000)
		pressure = (pressure << 1) / ((BME280_U32_t)v_x1_u32r);
	else
		pressure = (pressure / (BME280_U32_t)v_x1_u32r) * 2;
		v_x1_u32r = (((BME280_S32_t)p_bme280->cal_param.dig_P9) *
		((BME280_S32_t)(((pressure >> 3) * (pressure >> 3)) >> 13)))
		>> 12;
		v_x2_u32r = (((BME280_S32_t)(pressure >> 2)) *
		((BME280_S32_t)p_bme280->cal_param.dig_P8)) >> 13;
		pressure = (BME280_U32_t)((BME280_S32_t)pressure +
		((v_x1_u32r + v_x2_u32r + p_bme280->cal_param.dig_P7) >> 4));

	return pressure;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief reads uncompensated Humidity
 *
 *
 *
 *
 *  \param signed long uhumidity : Pointer holding the
 *                              uncompensated humidity.
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_read_uh(BME280_S32_t *uhumidity)
{
	BME280_RETURN_FUNCTION_TYPE comres = BME280_Zero_U8X;
	unsigned char a_data_u8r[2] = {0, 0};
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			comres += p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_HUMIDITY_MSB_REG, a_data_u8r, 2);
			*uhumidity = (BME280_S32_t)(
			(((BME280_U32_t)(a_data_u8r[0]))
			<< SHIFT_LEFT_8_POSITION)|
			((BME280_U32_t)(a_data_u8r[1])));
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Reads actual humidity from
 *        uncompensated humidity
 *        and returns the value in %rH as unsigned 32bit integer
 *        in Q22.10 format(22 integer 10 fractional bits).
 *        An output value of 42313
 *        represents 42313 / 1024 = 42.313 %rH
 *
 *
 *
 *  \param signed long : value of uncompensated humidity
 *
 *
 *
 *  \return
 *                unsigned long : actual relative humidity
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_U32_t bme280_compensate_H_int32(BME280_S32_t adc_h)
{
	BME280_S32_t v_x1_u32r;
	v_x1_u32r = (p_bme280->cal_param.t_fine - ((BME280_S32_t)76800));
	v_x1_u32r = (((((adc_h << 14) -
	(((BME280_S32_t)p_bme280->cal_param.dig_H4) << 20) -
	(((BME280_S32_t)p_bme280->cal_param.dig_H5) * v_x1_u32r)) +
	((BME280_S32_t)16384)) >> 15) *
	(((((((v_x1_u32r *
	((BME280_S32_t)p_bme280->cal_param.dig_H6)) >> 10) *
	(((v_x1_u32r * ((BME280_S32_t)p_bme280->cal_param.dig_H3)) >> 11) +
	((BME280_S32_t)32768))) >> 10) +
	((BME280_S32_t)2097152)) *
	((BME280_S32_t)p_bme280->cal_param.dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) *
	(v_x1_u32r >> 15)) >> 7) *
	((BME280_S32_t)p_bme280->cal_param.dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	return (BME280_U32_t)(v_x1_u32r>>12);
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief reads uncompensated pressure,temperature and humidity
 *
 *
 *
 *
 *  \param signed long upressure: Pointer holding the uncompensated pressure.
 *  \param signed long utemperature: Pointer holding
 *                    the uncompensated temperature.
 *  \param signed long uhumidity: Pointer holding the uncompensated humidity.
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_read_uputuh(BME280_S32_t *upressure,
BME280_S32_t *utemperature, BME280_S32_t *uhumidity)
{
	BME280_RETURN_FUNCTION_TYPE comres = BME280_Zero_U8X;
	unsigned char a_data_u8r[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			comres += p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_PRESSURE_MSB_REG, a_data_u8r, 8);
			/*Pressure*/
			*upressure = (BME280_S32_t)((
			((BME280_U32_t)(a_data_u8r[0]))
			<< SHIFT_LEFT_12_POSITION) |
			(((BME280_U32_t)(a_data_u8r[1]))
			<< SHIFT_LEFT_4_POSITION) |
			((BME280_U32_t)a_data_u8r[2] >>
			SHIFT_RIGHT_4_POSITION));

			/* Temperature */
			*utemperature = (BME280_S32_t)(((
			(BME280_U32_t) (a_data_u8r[3]))
			<< SHIFT_LEFT_12_POSITION) |
			(((BME280_U32_t)(a_data_u8r[4]))
			<< SHIFT_LEFT_4_POSITION)
			| ((BME280_U32_t)a_data_u8r[5]
			>> SHIFT_RIGHT_4_POSITION));

			/*Humidity*/
			*uhumidity = (BME280_S32_t)((
			((BME280_U32_t)(a_data_u8r[6]))
			<< SHIFT_LEFT_8_POSITION)|
			((BME280_U32_t)(a_data_u8r[7])));
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief reads pressure, temperature and humidity.
 *
 *
 *
 *
 *  \param unsigned long pressure : Pointer holding
 *                          the compensated pressure.
 * \param signed long temperature : Pointer holding
 *                      the compensated temperature.
 *  \param unsigned long humidity : Pointer holding
 *                         the compensated humidity.
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_read_pth(BME280_U32_t *pressure,
BME280_S32_t *temperature, BME280_U32_t *humidity)
{
	BME280_RETURN_FUNCTION_TYPE comres = BME280_Zero_U8X;
	BME280_S32_t upressure = BME280_Zero_U8X;
	BME280_S32_t utemperature = BME280_Zero_U8X;
	BME280_S32_t uhumidity = BME280_Zero_U8X;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			comres += bme280_read_uputuh(&upressure,
			&utemperature, &uhumidity);
			*temperature = bme280_compensate_T_int32(utemperature);
			*pressure = bme280_compensate_P_int32(upressure);
			*humidity = bme280_compensate_H_int32(uhumidity);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief reads calibration parameters used for calculation
 *
 *
 *
 *
 *  \param  None
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_get_calib_param()
{
	BME280_RETURN_FUNCTION_TYPE comres = BME280_Zero_U8X;
	unsigned char a_data_u8r[26] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0 , 0, 0, 0, 0, 0, 0};
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			comres += p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_DIG_T1_LSB_REG, a_data_u8r, 26);

			p_bme280->cal_param.dig_T1 = (BME280_U16_t)(((
			(BME280_U16_t)((unsigned char)a_data_u8r[1])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8r[0]);
			p_bme280->cal_param.dig_T2 = (BME280_S16_t)(((
			(BME280_S16_t)((signed char)a_data_u8r[3])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8r[2]);
			p_bme280->cal_param.dig_T3 = (BME280_S16_t)(((
			(BME280_S16_t)((signed char)a_data_u8r[5])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8r[4]);
			p_bme280->cal_param.dig_P1 = (BME280_U16_t)(((
			(BME280_U16_t)((unsigned char)a_data_u8r[7])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8r[6]);
			p_bme280->cal_param.dig_P2 = (BME280_S16_t)(((
			(BME280_S16_t)((signed char)a_data_u8r[9])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8r[8]);
			p_bme280->cal_param.dig_P3 = (BME280_S16_t)(((
			(BME280_S16_t)((signed char)a_data_u8r[11])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8r[10]);
			p_bme280->cal_param.dig_P4 = (BME280_S16_t)(((
			(BME280_S16_t)((signed char)a_data_u8r[13])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8r[12]);
			p_bme280->cal_param.dig_P5 = (BME280_S16_t)(((
			(BME280_S16_t)((signed char)a_data_u8r[15])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8r[14]);
			p_bme280->cal_param.dig_P6 = (BME280_S16_t)(((
			(BME280_S16_t)((signed char)a_data_u8r[17])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8r[16]);
			p_bme280->cal_param.dig_P7 = (BME280_S16_t)(((
			(BME280_S16_t)((signed char)a_data_u8r[19])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8r[18]);
			p_bme280->cal_param.dig_P8 = (BME280_S16_t)(((
			(BME280_S16_t)((signed char)a_data_u8r[21])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8r[20]);
			p_bme280->cal_param.dig_P9 = (BME280_S16_t)(((
			(BME280_S16_t)((signed char)a_data_u8r[23])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8r[22]);
			p_bme280->cal_param.dig_H1 = a_data_u8r[25];
			comres += p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_DIG_H2_LSB_REG, a_data_u8r, 8);
			p_bme280->cal_param.dig_H2 = (BME280_S16_t)(((
			(BME280_S16_t)((signed char)a_data_u8r[1])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8r[0]);
			p_bme280->cal_param.dig_H3 = a_data_u8r[2];
			p_bme280->cal_param.dig_H4 = (BME280_S16_t)(((
			(BME280_S16_t)((signed char)a_data_u8r[3])) <<
			SHIFT_LEFT_4_POSITION) | (((unsigned char)0x0F)
			& a_data_u8r[4]));
			p_bme280->cal_param.dig_H5 = (BME280_S16_t)(((
			(BME280_S16_t)((signed char)a_data_u8r[5])) <<
			SHIFT_LEFT_4_POSITION) | (a_data_u8r[4] >>
			SHIFT_RIGHT_4_POSITION));
			p_bme280->cal_param.dig_H6 = (signed char)a_data_u8r[6];
			p_bme280->crc = a_data_u8r[7];
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Used to get the temperature oversampling setting
 *
 *
 *
 *
 *  \param unsigned char value : Pointer holding the osrs_t value
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_get_osrs_t(
unsigned char *value)
{
	BME280_RETURN_FUNCTION_TYPE comres = BME280_Zero_U8X;
	unsigned char v_data_u8r = BME280_Zero_U8X;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			comres += p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CTRLMEAS_REG_OSRST__REG,
			&v_data_u8r, 1);
			*value = BME280_GET_BITSLICE(v_data_u8r,
			BME280_CTRLMEAS_REG_OSRST);

			p_bme280->osrs_t = *value;
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Used to set the temperature oversampling setting
 *
 *
 *
 *
 *  \param unsigned char value : Value of the temperature oversampling setting
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_set_osrs_t(
unsigned char value)
{
	BME280_RETURN_FUNCTION_TYPE comres = SUCCESS;
	unsigned char v_data_u8r = BME280_Zero_U8X;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			comres += p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CTRLMEAS_REG_OSRST__REG,
			&v_data_u8r, 1);
			if (comres == SUCCESS) {
				v_data_u8r =
				BME280_SET_BITSLICE(v_data_u8r,
				BME280_CTRLMEAS_REG_OSRST, value);
				comres +=
				p_bme280->BME280_BUS_WRITE_FUNC(
				p_bme280->dev_addr,
				BME280_CTRLMEAS_REG_OSRST__REG,
				&v_data_u8r, 1);

				p_bme280->osrs_t = value;
			}
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Used to get the pressure oversampling setting
 *
 *
 *
 *
 *  \param  *  \param unsigned char value : Pointer holding the osrs_p value
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_get_osrs_p(
unsigned char *value)
{
	BME280_RETURN_FUNCTION_TYPE comres = BME280_Zero_U8X;
	unsigned char v_data_u8r = BME280_Zero_U8X;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			comres += p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CTRLMEAS_REG_OSRSP__REG, &v_data_u8r, 1);
			*value = BME280_GET_BITSLICE(
			v_data_u8r,
			BME280_CTRLMEAS_REG_OSRSP);

			p_bme280->osrs_p = *value;
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Used to set the pressure oversampling setting
 *
 *
 *
 *
 *  \param unsigned char value : Value of the pressure oversampling setting
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_set_osrs_p(
unsigned char value)
{
	BME280_RETURN_FUNCTION_TYPE comres = SUCCESS;
	unsigned char v_data_u8r = BME280_Zero_U8X;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			comres += p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CTRLMEAS_REG_OSRSP__REG,
			&v_data_u8r, 1);
			if (comres == SUCCESS) {
				v_data_u8r =
				BME280_SET_BITSLICE(v_data_u8r,
				BME280_CTRLMEAS_REG_OSRSP, value);
				comres +=
				p_bme280->BME280_BUS_WRITE_FUNC(
				p_bme280->dev_addr,
				BME280_CTRLMEAS_REG_OSRSP__REG,
				&v_data_u8r, 1);

				p_bme280->osrs_p = value;
			}
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Used to get the humidity oversampling setting
 *
 *
 *
 *
 *  \param  *  \param unsigned char value : Pointer holding the osrs_h value
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_get_osrs_h(
unsigned char *value)
{
	BME280_RETURN_FUNCTION_TYPE comres = BME280_Zero_U8X;
	unsigned char v_data_u8r = BME280_Zero_U8X;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			comres += p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CTRLHUM_REG_OSRSH__REG,
			&v_data_u8r, 1);
			*value = BME280_GET_BITSLICE(
			v_data_u8r,
			BME280_CTRLHUM_REG_OSRSH);

			p_bme280->osrs_h = *value;
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Used to set the humidity oversampling setting
 *
 * The "BME280_CTRLHUM_REG_OSRSH" register sets the humidity
 * data acquisition options of the device.
 * changes to this registers only become effective after a write operation to
 * "BME280_CTRLMEAS_REG" register.
 * In the code automated reading and writing of "BME280_CTRLHUM_REG_OSRSH"
 * register first set the "BME280_CTRLHUM_REG_OSRSH" and then read and write
 * the "BME280_CTRLMEAS_REG" register in the function.
 *
 *
 *
 *  \param unsigned char value : Value of the humidity oversampling setting
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_set_osrs_h(
unsigned char value)
{
	BME280_RETURN_FUNCTION_TYPE comres = SUCCESS;
	unsigned char v_data_u8r = BME280_Zero_U8X;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			comres += p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CTRLHUM_REG_OSRSH__REG,
			&v_data_u8r, 1);
			if (comres == SUCCESS) {
				v_data_u8r =
				BME280_SET_BITSLICE(v_data_u8r,
				BME280_CTRLHUM_REG_OSRSH, value);
				comres +=
				p_bme280->BME280_BUS_WRITE_FUNC(
				p_bme280->dev_addr,
				BME280_CTRLHUM_REG_OSRSH__REG,
				&v_data_u8r, 1);
				p_bme280->osrs_h = value;
			}

			comres +=
			p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CTRLMEAS_REG, &v_data_u8r, 1);
			if (comres == SUCCESS) {
				comres +=
				p_bme280->BME280_BUS_WRITE_FUNC(
				p_bme280->dev_addr,
				BME280_CTRLMEAS_REG,
				&v_data_u8r, 1);
			}
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Used to get the Operational Mode from the sensor
 *
 *
 *
 *
 *  \param  *  \param unsigned char mode : Pointer holding the mode value
 *              0       -> BME280_SLEEP_MODE
 *              1 and 2 -> BME280_FORCED_MODE
 *              3       -> BME280_NORMAL_MODE
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_get_mode(unsigned char *mode)
{
	BME280_RETURN_FUNCTION_TYPE comres = BME280_Zero_U8X;
	unsigned char v_mode_u8r = BME280_Zero_U8X;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			comres += p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CTRLMEAS_REG_MODE__REG,
			&v_mode_u8r, 1);
			*mode = BME280_GET_BITSLICE(v_mode_u8r,
			BME280_CTRLMEAS_REG_MODE);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Used to set the Operational Mode for the sensor
 *
 *
 *
 *
 *  \param unsigned char mode : Value of the mode
 *              0       -> BME280_SLEEP_MODE
 *              1 and 2 -> BME280_FORCED_MODE
 *              3       -> BME280_NORMAL_MODE
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_set_mode(unsigned char mode)
{
	BME280_RETURN_FUNCTION_TYPE comres = BME280_Zero_U8X;
	unsigned char v_mode_u8r = BME280_Zero_U8X;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			if (mode < BME280_Four_U8X) {
				v_mode_u8r = (p_bme280->osrs_t <<
				SHIFT_LEFT_5_POSITION) +
				(p_bme280->osrs_p <<
				SHIFT_LEFT_2_POSITION) + mode;
				comres +=
				p_bme280->BME280_BUS_WRITE_FUNC(
				p_bme280->dev_addr,
				BME280_CTRLMEAS_REG_MODE__REG,
				&v_mode_u8r, 1);
			} else {
			comres = E_BME280_OUT_OF_RANGE;
			}
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Used to reset the sensor
 * The value 0xB6 is written to the 0xE0 register the device is reset using the
 * complete power-on-reset procedure.
 * Softreset can be easily set using bme280_set_softreset().
 * Usage Hint : bme280_set_softreset()
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_set_softreset()
{
	BME280_RETURN_FUNCTION_TYPE comres = BME280_Zero_U8X;
	unsigned char v_data_u8r = BME280_SOFT_RESET_CODE;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			comres += p_bme280->BME280_BUS_WRITE_FUNC(
			p_bme280->dev_addr,
			BME280_RESET_REG, &v_data_u8r, 1);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Gets the sensor communication type
 *
 *
 *
 *
 *  \param  unsigned char enable_disable : Pointer holding the
 *                                          spi3 enable or disable state.
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_get_spi3(unsigned char *enable_disable)
{
	BME280_RETURN_FUNCTION_TYPE comres = BME280_Zero_U8X;
	unsigned char v_data_u8r = BME280_Zero_U8X;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			comres += p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CONFIG_REG_SPI3WEN__REG,
			&v_data_u8r, 1);
			*enable_disable = BME280_GET_BITSLICE(
			v_data_u8r,
			BME280_CONFIG_REG_SPI3WEN);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Sets the sensor communication type to 3 wire SPI
 *
 *
 *
 *
 *  \param unsigned char enable_disable : Value of the enable or disable
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_set_spi3(unsigned char enable_disable)
{
	BME280_RETURN_FUNCTION_TYPE comres = SUCCESS;
	unsigned char v_data_u8r = BME280_Zero_U8X;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			comres += p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CONFIG_REG_SPI3WEN__REG,
			&v_data_u8r, 1);
			if (comres == SUCCESS) {
				v_data_u8r =
				BME280_SET_BITSLICE(v_data_u8r,
				BME280_CONFIG_REG_SPI3WEN,
				enable_disable);
				comres +=
				p_bme280->BME280_BUS_WRITE_FUNC(
				p_bme280->dev_addr,
				BME280_CONFIG_REG_SPI3WEN__REG,
				&v_data_u8r, 1);
			}
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Reads filter setting value
 *
 *
 *
 *
 *  \param  *  \param unsigned char *value : Pointer holding the filter value
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_get_filter(unsigned char *value)
{
	BME280_RETURN_FUNCTION_TYPE comres = BME280_Zero_U8X;
	unsigned char v_data_u8r = BME280_Zero_U8X;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			comres += p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CONFIG_REG_FILTER__REG,
			&v_data_u8r, 1);
			*value = BME280_GET_BITSLICE(v_data_u8r,
			BME280_CONFIG_REG_FILTER);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Writes filter setting to sensor
 *
 *
 *
 *
 *  \param unsigned char value : Value of the filter setting
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_set_filter(unsigned char value)
{
	BME280_RETURN_FUNCTION_TYPE comres = SUCCESS;
	unsigned char v_data_u8r = BME280_Zero_U8X;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			comres += p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CONFIG_REG_FILTER__REG, &v_data_u8r, 1);
			if (comres == SUCCESS) {
				v_data_u8r =
				BME280_SET_BITSLICE(
				v_data_u8r,
				BME280_CONFIG_REG_FILTER, value);
				comres +=
				p_bme280->BME280_BUS_WRITE_FUNC(
				p_bme280->dev_addr,
				BME280_CONFIG_REG_FILTER__REG,
				&v_data_u8r, 1);
			}
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Reads standby duration time from the sensor
 *
 * \param * \param unsigned char *time : Pointer holding
 *                      the standby duration time value.
 *              0x00 - BME280_STANDBYTIME_1_MS
 *              0x01 - BME280_STANDBYTIME_63_MS
 *              0x02 - BME280_STANDBYTIME_125_MS
 *              0x03 - BME280_STANDBYTIME_250_MS
 *              0x04 - BME280_STANDBYTIME_500_MS
 *              0x05 - BME280_STANDBYTIME_1000_MS
 *              0x06 - BME280_STANDBYTIME_10_MS
 *              0x07 - BME280_STANDBYTIME_20_MS
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_get_standbydur(unsigned char *time)
{
	BME280_RETURN_FUNCTION_TYPE comres = BME280_Zero_U8X;
	unsigned char v_data_u8r = BME280_Zero_U8X;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			comres += p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CONFIG_REG_TSB__REG,
			&v_data_u8r, 1);
			*time = BME280_GET_BITSLICE(
			v_data_u8r, BME280_CONFIG_REG_TSB);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Writes standby duration time from the sensor
 *
 * Normal mode comprises an automated perpetual cycling between an (active)
 * Measurement period and an (inactive) standby period.
 * The standby time is determined by the contents of the register t_sb.
 * Standby time can be set using BME280_STANDBYTIME_125_MS.
 *
 * Usage Hint : bme280_set_standbydur(BME280_STANDBYTIME_125_MS)
 *
 *  \param unsigned char time : Value of the standby duration
 *              0x00 - BME280_STANDBYTIME_1_MS
 *              0x01 - BME280_STANDBYTIME_63_MS
 *              0x02 - BME280_STANDBYTIME_125_MS
 *              0x03 - BME280_STANDBYTIME_250_MS
 *              0x04 - BME280_STANDBYTIME_500_MS
 *              0x05 - BME280_STANDBYTIME_1000_MS
 *              0x06 - BME280_STANDBYTIME_10_MS
 *              0x07 - BME280_STANDBYTIME_20_MS
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_set_standbydur(unsigned char time)
{
	BME280_RETURN_FUNCTION_TYPE comres = SUCCESS;
	unsigned char v_data_u8r = BME280_Zero_U8X;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			comres += p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CONFIG_REG_TSB__REG,
			&v_data_u8r, 1);
			if (comres == SUCCESS) {
				v_data_u8r =
				BME280_SET_BITSLICE(
				v_data_u8r,
				BME280_CONFIG_REG_TSB, time);
				comres +=
				p_bme280->BME280_BUS_WRITE_FUNC(
				p_bme280->dev_addr,
				BME280_CONFIG_REG_TSB__REG,
				&v_data_u8r, 1);
			}
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Writes the working mode to the sensor
 *
 *
 *
 *
 *  \param unsigned char : Mode to be set
 *				0 -> BME280_ULTRALOWPOWER_MODE
 *				1 -> BME280_LOWPOWER_MODE
 *				2 -> BME280_STANDARDRESOLUTION_MODE
 *				3 -> BME280_HIGHRESOLUTION_MODE
 *				4 -> BME280_ULTRAHIGHRESOLUTION_MODE
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
/*BME280_RETURN_FUNCTION_TYPE bme280_set_workmode(unsigned char mode)
{
BME280_RETURN_FUNCTION_TYPE comres = SUCCESS;
unsigned char v_data_u8r = BME280_Zero_U8X;
if (p_bme280 == BME280_NULL) {
	return E_BME280_NULL_PTR;
} else {
	if (mode <= BME280_Four_U8X) {
		comres += p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,	BME280_CTRLMEAS_REG,
			&v_data_u8r, 1);
		if (comres == SUCCESS) {
			switch (mode) {
			case BME280_ULTRALOWPOWER_MODE:
				p_bme280->osrs_t =
				BME280_ULTRALOWPOWER_OSRS_T;
				p_bme280->osrs_p =
				BME280_ULTRALOWPOWER_OSRS_P;
				break;
			case BME280_LOWPOWER_MODE:
				p_bme280->osrs_t = BME280_LOWPOWER_OSRS_T;
				p_bme280->osrs_p = BME280_LOWPOWER_OSRS_P;
				break;
			case BME280_STANDARDRESOLUTION_MODE:
				p_bme280->osrs_t =
				BME280_STANDARDRESOLUTION_OSRS_T;
				p_bme280->osrs_p =
				BME280_STANDARDRESOLUTION_OSRS_P;
				break;
			case BME280_HIGHRESOLUTION_MODE:
				p_bme280->osrs_t = BME280_HIGHRESOLUTION_OSRS_T;
				p_bme280->osrs_p = BME280_HIGHRESOLUTION_OSRS_P;
				break;
			case BME280_ULTRAHIGHRESOLUTION_MODE:
				p_bme280->osrs_t =
				BME280_ULTRAHIGHRESOLUTION_OSRS_T;
				p_bme280->osrs_p =
				BME280_ULTRAHIGHRESOLUTION_OSRS_P;
				break;
			}
			v_data_u8r = BME280_SET_BITSLICE(v_data_u8r,
				BME280_CTRLMEAS_REG_OSRST, p_bme280->osrs_t);
			v_data_u8r = BME280_SET_BITSLICE(v_data_u8r,
				BME280_CTRLMEAS_REG_OSRSP, p_bme280->osrs_p);
			comres += p_bme280->BME280_BUS_WRITE_FUNC(
				p_bme280->dev_addr,	BME280_CTRLMEAS_REG,
				&v_data_u8r, 1);
		}
	} else {
		comres = E_BME280_OUT_OF_RANGE;
	}
}
return comres;
}*/
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Read both uncompensated temperature,pressure and humidity
 *                                                       in forced mode
 *
 *
 * \param signed long upressure: Pointer holding
 *                     the uncompensated pressure.
 *  \param signed long utemperature: Pointer holding
 *                    the uncompensated temperature.
 *  \param signed long uhumidity: Pointer holding
 *                     the uncompensated humidity.
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_get_forced_uputuh(BME280_S32_t *upressure,
BME280_S32_t *utemperature, BME280_S32_t *uhumidity)
{
	BME280_RETURN_FUNCTION_TYPE comres = BME280_Zero_U8X;
	unsigned char v_data_u8r = BME280_Zero_U8X;
	unsigned char v_waittime_u8r = BME280_Zero_U8X;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			v_data_u8r = (p_bme280->osrs_h);
			comres += p_bme280->BME280_BUS_WRITE_FUNC(
			p_bme280->dev_addr, BME280_CTRLHUM_REG,
			&v_data_u8r, 1);
			v_data_u8r = (p_bme280->osrs_t
			<< SHIFT_LEFT_5_POSITION) +
			(p_bme280->osrs_p << SHIFT_LEFT_2_POSITION) +
			BME280_FORCED_MODE;
			comres += p_bme280->BME280_BUS_WRITE_FUNC(
			p_bme280->dev_addr, BME280_CTRLMEAS_REG,
			&v_data_u8r, 1);

			bme280_compute_wait_time(&v_waittime_u8r);
			p_bme280->delay_msec(v_waittime_u8r);
			comres += bme280_read_uputuh(
			upressure, utemperature, uhumidity);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief This API gives data to the given register and
 *                          the data is written in the corresponding register
 *							address
 *
 *
 *
 *  \param unsigned char addr, unsigned char data, unsigned char len
 *          addr -> Address of the register
 *          data -> Data to be written to the register
 *          len  -> Length of the Data
 *
 *
 *
 *  \return communication results.
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_write_register(unsigned char addr,
unsigned char *data, unsigned char len)
{
	BME280_RETURN_FUNCTION_TYPE comres = BME280_Zero_U8X;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			comres += p_bme280->BME280_BUS_WRITE_FUNC(
			p_bme280->dev_addr,
			addr, data, len);
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief This API reads the data from the given register
 *							address
 *
 *
 *
 *  \param unsigned char addr, unsigned char *data, unsigned char len
 *         addr -> Address of the register
 *         data -> address of the variable, read value will be kept
 *         len  -> Length of the data
 *
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_read_register(unsigned char addr,
unsigned char *data, unsigned char len)
{
	BME280_RETURN_FUNCTION_TYPE comres = BME280_Zero_U8X;
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			comres += p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			addr, data, len);
		}
	return comres;
}
#ifdef BME280_ENABLE_FLOAT
/*******************************************************************************
 * Description: *//**\brief Reads actual temperature from uncompensated temperature
 *							and returns the value in Degree centigrade
 *                          Output value of "51.23" equals 51.23 DegC.
 *
 *
 *
 *  \param signed long : value of uncompensated temperature
 *
 *
 *
 *  \return
 *			double : actual temperature in floating point
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
double bme280_compensate_T_double(BME280_S32_t adc_t)
{
	double v_x1_u32r = BME280_Zero_U8X;
	double v_x2_u32r = BME280_Zero_U8X;
	double temperature = BME280_Zero_U8X;

	v_x1_u32r  = (((double)adc_t) / 16384.0 -
	((double)p_bme280->cal_param.dig_T1) / 1024.0) *
	((double)p_bme280->cal_param.dig_T2);
	v_x2_u32r  = ((((double)adc_t) / 131072.0 -
	((double)p_bme280->cal_param.dig_T1) / 8192.0) *
	(((double)adc_t) / 131072.0 -
	((double)p_bme280->cal_param.dig_T1) / 8192.0)) *
	((double)p_bme280->cal_param.dig_T3);
	p_bme280->cal_param.t_fine = (BME280_S32_t)(v_x1_u32r + v_x2_u32r);
	temperature  = (v_x1_u32r + v_x2_u32r) / 5120.0;


	return temperature;
}
/*******************************************************************************
 * Description: *//**\brief Reads actual pressure from uncompensated pressure
 *							and returns pressure in Pa as double.
 *                          Output value of "96386.2"
 *                          equals 96386.2 Pa = 963.862 hPa.
 *
 *
 *  \param signed int : value of uncompensated pressure
 *
 *
 *
 *  \return
 *			double : actual pressure in floating point
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
double bme280_compensate_P_double(BME280_S32_t adc_p)
{
	double v_x1_u32r = BME280_Zero_U8X;
	double v_x2_u32r = BME280_Zero_U8X;
	double pressure = BME280_Zero_U8X;

	v_x1_u32r = ((double)p_bme280->cal_param.t_fine/2.0) - 64000.0;
	v_x2_u32r = v_x1_u32r * v_x1_u32r *
	((double)p_bme280->cal_param.dig_P6) / 32768.0;
	v_x2_u32r = v_x2_u32r + v_x1_u32r *
	((double)p_bme280->cal_param.dig_P5) * 2.0;
	v_x2_u32r = (v_x2_u32r / 4.0) +
	(((double)p_bme280->cal_param.dig_P4) * 65536.0);
	v_x1_u32r = (((double)p_bme280->cal_param.dig_P3) *
	v_x1_u32r * v_x1_u32r / 524288.0 +
	((double)p_bme280->cal_param.dig_P2) * v_x1_u32r) / 524288.0;
	v_x1_u32r = (1.0 + v_x1_u32r / 32768.0) *
	((double)p_bme280->cal_param.dig_P1);
	if (v_x1_u32r == 0.0)
		return 0;
		/* Avoid exception caused by division by zero */
	pressure = 1048576.0 - (double)adc_p;
	pressure = (pressure - (v_x2_u32r / 4096.0)) * 6250.0 / v_x1_u32r;
	v_x1_u32r = ((double)p_bme280->cal_param.dig_P9) *
	pressure * pressure / 2147483648.0;
	v_x2_u32r = pressure * ((double)p_bme280->cal_param.dig_P8) / 32768.0;
	pressure = pressure + (v_x1_u32r + v_x2_u32r +
	((double)p_bme280->cal_param.dig_P7)) / 16.0;

	return pressure;
}
/*******************************************************************************
 * Description: *//**\brief Reads actual humidity from uncompensated humidity
 *							and returns the value in relative humidity (%rH)
 *                          Output value of "42.12" equals 42.12 %rH
 *
 *  \param signed int : value of uncompensated humidity
 *
 *
 *
 *  \return
 *			double : actual humidity in floating point
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
double bme280_compensate_H_double(BME280_S32_t adc_h)
{
	double var_h = BME280_Zero_U8X;
	var_h = (((double)p_bme280->cal_param.t_fine)-76800.0);
	var_h = (adc_h-(((double)p_bme280->cal_param.dig_H4)*64.0 +
	((double)p_bme280->cal_param.dig_H5) / 16384.0 * var_h))*
	(((double)p_bme280->cal_param.dig_H2)/65536.0*(1.0 + ((double)
	p_bme280->cal_param.dig_H6)/67108864.0*var_h*(1.0+((double)
	p_bme280->cal_param.dig_H3)/67108864.0*var_h)));
	var_h = var_h * (1.0-((double)
	p_bme280->cal_param.dig_H1)*var_h/524288.0);
	if (var_h > 100.0)
		var_h = 100.0;
	else if (var_h < 0.0)
		var_h = 0.0;
	return var_h;

}
#endif
#if defined(BME280_ENABLE_INT64) && defined(BME280_64BITSUPPORT_PRESENT)
/*******************************************************************************
 * Description: *//**\brief Reads actual pressure from uncompensated pressure
 *                          and returns the value in Pa as unsigned 32 bit
 *                          integer in Q24.8 format (24 integer bits and
 *                          8 fractional bits). Output value of "24674867"
 *                          represents 24674867 / 256 = 96386.2 Pa = 963.862 hPa
 *
 *
 *
 *  \param signed long : value of uncompensated temperature
 *
 *
 *
 *  \return
 *			unsigned long : actual pressure
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BME280_U32_t bme280_compensate_P_int64(BME280_S32_t adc_p)
{
	BME280_S64_t v_x1_s64r = BME280_Zero_U8X;
	BME280_S64_t v_x2_s64r = BME280_Zero_U8X;
	BME280_S64_t pressure = BME280_Zero_U8X;
	v_x1_s64r = ((BME280_S64_t)p_bme280->cal_param.t_fine) - 128000;
	v_x2_s64r = v_x1_s64r * v_x1_s64r *
	(BME280_S64_t)p_bme280->cal_param.dig_P6;
	v_x2_s64r = v_x2_s64r + ((v_x1_s64r *
	(BME280_S64_t)p_bme280->cal_param.dig_P5) << 17);
	v_x2_s64r = v_x2_s64r +
	(((BME280_S64_t)p_bme280->cal_param.dig_P4) << 35);
	v_x1_s64r = ((v_x1_s64r * v_x1_s64r *
	(BME280_S64_t)p_bme280->cal_param.dig_P3) >> 8) +
	((v_x1_s64r * (BME280_S64_t)p_bme280->cal_param.dig_P2) << 12);
	v_x1_s64r = (((((BME280_S64_t)1) << 47) + v_x1_s64r)) *
	((BME280_S64_t)p_bme280->cal_param.dig_P1) >> 33;
	if (v_x1_s64r == 0)
		return 0;
		/* Avoid exception caused by division by zero */
	pressure = 1048576 - adc_p;
#if defined(__KERNEL__)
	pressure = div64_s64(((pressure << 31) - v_x2_s64r) * 3125, v_x1_s64r);
#else
	pressure = (((pressure << 31) - v_x2_s64r) * 3125) / v_x1_s64r;
#endif
	v_x1_s64r = (((BME280_S64_t)p_bme280->cal_param.dig_P9) *
	(pressure >> 13) * (pressure >> 13)) >> 25;
	v_x2_s64r = (((BME280_S64_t)p_bme280->cal_param.dig_P8) *
	pressure) >> 19;
	pressure = ((pressure + v_x1_s64r + v_x2_s64r) >> 8) +
	(((BME280_S64_t)p_bme280->cal_param.dig_P7) << 4);
	return (BME280_U32_t)pressure;
}
#endif
/*******************************************************************************
 * Description: *//**\brief Computing waiting time for sensor data read
 *
 *
 *
 *
 *  \param
 *			unsigned char : value of time
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BME280_RETURN_FUNCTION_TYPE bme280_compute_wait_time(unsigned char
*v_delaytime_u8r)
{
	BME280_RETURN_FUNCTION_TYPE comres = BME280_Zero_U8X;

	*v_delaytime_u8r = (T_INIT_MAX + T_MEASURE_PER_OSRS_MAX *
	(((1 << p_bme280->osrs_t) >> 1) + ((1 << p_bme280->osrs_p)
	>> 1) + ((1 << p_bme280->osrs_h) >> 1))+
	(p_bme280->osrs_p ? T_SETUP_PRESSURE_MAX : 0) +
	(p_bme280->osrs_h ? T_SETUP_HUMIDITY_MAX : 0) + 15) / 16;
	return comres;
}
