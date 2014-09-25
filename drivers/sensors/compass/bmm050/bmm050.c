/*
 * Last modified: Arp 28, 2011
 * Revision: V1.0
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 */

#include <linux/sensors/bmm050.h>

static struct bmc050 *p_bmc050;

BMC050_RETURN_FUNCTION_TYPE bmc050_init(struct bmc050 *bmc050)
{
	BMC050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char a_data_u8r[2];
	p_bmc050 = bmc050;

	p_bmc050->dev_addr = BMC050_I2C_ADDRESS;

	/* set device from suspend into sleep mode */
	bmc050_set_powermode(BMC050_ON);

	/* wait two millisecond for bmc to settle */
	p_bmc050->delay_msec(BMC050_DELAY_SETTLING_TIME);

	/*Read CHIP_ID and REv. info */
	comres = p_bmc050->BMC050_BUS_READ_FUNC(p_bmc050->dev_addr,
			BMC050_CHIP_ID, a_data_u8r, 2);
	p_bmc050->company_id = a_data_u8r[0];
	p_bmc050->revision_info = a_data_u8r[1];

	/* Function to initialise trim values */
	bmc050_init_trim_registers();
	return comres;
}

BMC050_RETURN_FUNCTION_TYPE bmc050_init_trim_registers(void)
{
	BMC050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char a_data_u8r[2];
	comres = p_bmc050->BMC050_BUS_READ_FUNC(p_bmc050->dev_addr,
			BMC050_DIG_X1, (unsigned char *)&p_bmc050->dig_x1, 1);
	comres |= p_bmc050->BMC050_BUS_READ_FUNC(p_bmc050->dev_addr,
			BMC050_DIG_Y1, (unsigned char *)&p_bmc050->dig_y1, 1);
	comres |= p_bmc050->BMC050_BUS_READ_FUNC(p_bmc050->dev_addr,
			BMC050_DIG_X2, (unsigned char *)&p_bmc050->dig_x2, 1);
	comres |= p_bmc050->BMC050_BUS_READ_FUNC(p_bmc050->dev_addr,
			BMC050_DIG_Y2, (unsigned char *)&p_bmc050->dig_y2, 1);
	comres |= p_bmc050->BMC050_BUS_READ_FUNC(p_bmc050->dev_addr,
			BMC050_DIG_XY1, (unsigned char *)&p_bmc050->dig_xy1, 1);
	comres |= p_bmc050->BMC050_BUS_READ_FUNC(p_bmc050->dev_addr,
			BMC050_DIG_XY2, (unsigned char *)&p_bmc050->dig_xy2, 1);

	/* shorts can not be recasted into (unsigned char*)
	 * due to possible mixup between trim data
	 * arrangement and memory arrangement */

	comres |= p_bmc050->BMC050_BUS_READ_FUNC(p_bmc050->dev_addr,
			BMC050_DIG_Z1_LSB, a_data_u8r, 2);
	p_bmc050->dig_z1 = (BMC050_U16)((((BMC050_U16)((unsigned char)
						a_data_u8r[1])) <<
				SHIFT_LEFT_8_POSITION) | a_data_u8r[0]);

	comres |= p_bmc050->BMC050_BUS_READ_FUNC(p_bmc050->dev_addr,
			BMC050_DIG_Z2_LSB, a_data_u8r, 2);
	p_bmc050->dig_z2 = (BMC050_S16)((((BMC050_S16)(
						(signed char)a_data_u8r[1])) <<
				SHIFT_LEFT_8_POSITION) | a_data_u8r[0]);

	comres |= p_bmc050->BMC050_BUS_READ_FUNC(p_bmc050->dev_addr,
			BMC050_DIG_Z3_LSB, a_data_u8r, 2);
	p_bmc050->dig_z3 = (BMC050_S16)((((BMC050_S16)(
						(signed char)a_data_u8r[1])) <<
				SHIFT_LEFT_8_POSITION) | a_data_u8r[0]);

	comres |= p_bmc050->BMC050_BUS_READ_FUNC(p_bmc050->dev_addr,
			BMC050_DIG_Z4_LSB, a_data_u8r, 2);
	p_bmc050->dig_z4 = (BMC050_S16)((((BMC050_S16)(
						(signed char)a_data_u8r[1])) <<
				SHIFT_LEFT_8_POSITION) | a_data_u8r[0]);

	comres |= p_bmc050->BMC050_BUS_READ_FUNC(p_bmc050->dev_addr,
			BMC050_DIG_XYZ1_LSB, a_data_u8r, 2);
	a_data_u8r[1] = BMC050_GET_BITSLICE(a_data_u8r[1], BMC050_DIG_XYZ1_MSB);
	p_bmc050->dig_xyz1 = (BMC050_U16)((((BMC050_U16)
					((unsigned char)a_data_u8r[1])) <<
				SHIFT_LEFT_8_POSITION) | a_data_u8r[0]);
	return comres;
}


BMC050_RETURN_FUNCTION_TYPE bmc050_get_datarate(unsigned char *data_rate)
{
	BMC050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmc050 == BMC050_NULL) {
		comres = E_BMC050_NULL_PTR;
	} else {
		comres = p_bmc050->BMC050_BUS_READ_FUNC(
				p_bmc050->dev_addr,
				BMC050_CNTL_DR__REG,
				&v_data_u8r, 1);
		*data_rate = BMC050_GET_BITSLICE(v_data_u8r,
				BMC050_CNTL_DR);
	}
	return comres;
}


BMC050_RETURN_FUNCTION_TYPE bmc050_set_datarate(unsigned char data_rate)
{
	BMC050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmc050 == BMC050_NULL) {
		comres = E_BMC050_NULL_PTR;
	} else {
		comres = p_bmc050->BMC050_BUS_READ_FUNC(
				p_bmc050->dev_addr,
				BMC050_CNTL_DR__REG,
				&v_data1_u8r, 1);
		v_data1_u8r = BMC050_SET_BITSLICE(v_data1_u8r,
				BMC050_CNTL_DR, data_rate);
		comres |= p_bmc050->BMC050_BUS_WRITE_FUNC(
				p_bmc050->dev_addr,
				BMC050_CNTL_DR__REG,
				&v_data1_u8r, 1);
	}
	return comres;
}


BMC050_RETURN_FUNCTION_TYPE bmc050_get_powermode(unsigned char *mode)
{
	BMC050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmc050 == BMC050_NULL) {
		comres = E_BMC050_NULL_PTR;
	} else {
		comres = p_bmc050->BMC050_BUS_READ_FUNC(
				p_bmc050->dev_addr,
				BMC050_POWER_CNTL_PCB__REG,
				&v_data_u8r, 1);
		*mode = BMC050_GET_BITSLICE(v_data_u8r,
				BMC050_POWER_CNTL_PCB);
	}
	return comres;
}

BMC050_RETURN_FUNCTION_TYPE bmc050_set_powermode(unsigned char mode)
{
	BMC050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmc050 == BMC050_NULL) {
		comres = E_BMC050_NULL_PTR;
	} else {
		comres = p_bmc050->BMC050_BUS_READ_FUNC(
				p_bmc050->dev_addr,
				BMC050_POWER_CNTL_PCB__REG,
				&v_data_u8r, 1);
		v_data_u8r = BMC050_SET_BITSLICE(v_data_u8r,
				BMC050_POWER_CNTL_PCB, mode);
		comres |= p_bmc050->BMC050_BUS_WRITE_FUNC(
				p_bmc050->dev_addr,
				BMC050_POWER_CNTL_PCB__REG,
				&v_data_u8r, 1);
	}
	return comres;
}

BMC050_RETURN_FUNCTION_TYPE bmc050_get_functional_state(
		unsigned char *functional_state)
{
	BMC050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmc050 == BMC050_NULL) {
		comres = E_BMC050_NULL_PTR;
	} else {
		comres = p_bmc050->BMC050_BUS_READ_FUNC(
				p_bmc050->dev_addr,
				BMC050_CNTL_OPMODE__REG,
				&v_data_u8r, 1);
		*functional_state = BMC050_GET_BITSLICE(
				v_data_u8r, BMC050_CNTL_OPMODE);
	}
	return comres;
}


BMC050_RETURN_FUNCTION_TYPE bmc050_set_functional_state(
		unsigned char functional_state)
{
	BMC050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmc050 == BMC050_NULL) {
		comres = E_BMC050_NULL_PTR;
	} else {
		switch (functional_state) {
		case BMC050_NORMAL_MODE:
			comres = bmc050_get_powermode(&v_data1_u8r);
			if (v_data1_u8r == BMC050_OFF) {
				comres = bmc050_set_powermode(BMC050_ON);
				p_bmc050->delay_msec(
						BMC050_DELAY_SUSPEND_SLEEP);
			}
			{
				comres |= p_bmc050->BMC050_BUS_READ_FUNC(
						p_bmc050->dev_addr,
						BMC050_CNTL_OPMODE__REG,
						&v_data1_u8r, 1);
				v_data1_u8r = BMC050_SET_BITSLICE(
						v_data1_u8r,
						BMC050_CNTL_OPMODE,
						BMC050_NORMAL_MODE);
				comres |= p_bmc050->BMC050_BUS_WRITE_FUNC(
						p_bmc050->dev_addr,
						BMC050_CNTL_OPMODE__REG,
						&v_data1_u8r, 1);
			}
			break;
		case BMC050_SUSPEND_MODE:
			comres = bmc050_set_powermode(BMC050_OFF);
			break;
		case BMC050_FORCED_MODE:
			comres = bmc050_get_powermode(&v_data1_u8r);
			if (v_data1_u8r == BMC050_OFF) {
				comres = bmc050_set_powermode(BMC050_ON);
				p_bmc050->delay_msec(
						BMC050_DELAY_SUSPEND_SLEEP);
			}
			comres |= p_bmc050->BMC050_BUS_READ_FUNC(
					p_bmc050->dev_addr,
					BMC050_CNTL_OPMODE__REG,
					&v_data1_u8r, 1);
			v_data1_u8r = BMC050_SET_BITSLICE(
					v_data1_u8r,
					BMC050_CNTL_OPMODE, BMC050_ON);
			comres |= p_bmc050->BMC050_BUS_WRITE_FUNC(
					p_bmc050->dev_addr,
					BMC050_CNTL_OPMODE__REG,
					&v_data1_u8r, 1);
			break;
		case BMC050_SLEEP_MODE:
			bmc050_get_powermode(&v_data1_u8r);
			if (v_data1_u8r == BMC050_OFF) {
				comres = bmc050_set_powermode(BMC050_ON);
				p_bmc050->delay_msec(
						BMC050_DELAY_SUSPEND_SLEEP);
			}
			comres |= p_bmc050->BMC050_BUS_READ_FUNC(
					p_bmc050->dev_addr,
					BMC050_CNTL_OPMODE__REG,
					&v_data1_u8r, 1);
			v_data1_u8r = BMC050_SET_BITSLICE(
					v_data1_u8r,
					BMC050_CNTL_OPMODE,
					BMC050_SLEEP_MODE);
			comres |= p_bmc050->BMC050_BUS_WRITE_FUNC(
					p_bmc050->dev_addr,
					BMC050_CNTL_OPMODE__REG,
					&v_data1_u8r, 1);
			break;
		default:
			comres = E_BMC050_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}

BMC050_RETURN_FUNCTION_TYPE bmc050_get_repetitions_XY(
		unsigned char *no_repetitions_xy)
{
	BMC050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmc050 == BMC050_NULL) {
		comres = E_BMC050_NULL_PTR;
	} else {
		comres = p_bmc050->BMC050_BUS_READ_FUNC(
				p_bmc050->dev_addr,
				BMC050_NO_REPETITIONS_XY,
				&v_data_u8r, 1);
		*no_repetitions_xy = v_data_u8r;
	}
	return comres;
}

BMC050_RETURN_FUNCTION_TYPE bmc050_set_repetitions_XY(
		unsigned char no_repetitions_xy)
{
	BMC050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmc050 == BMC050_NULL) {
		comres = E_BMC050_NULL_PTR;
	} else {
		v_data_u8r = no_repetitions_xy;
		comres = p_bmc050->BMC050_BUS_WRITE_FUNC(
				p_bmc050->dev_addr,
				BMC050_NO_REPETITIONS_XY,
				&v_data_u8r, 1);
	}
	return comres;
}

BMC050_RETURN_FUNCTION_TYPE bmc050_get_repetitions_Z(
		unsigned char *no_repetitions_z)
{
	BMC050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmc050 == BMC050_NULL) {
		comres = E_BMC050_NULL_PTR;
	} else {
		comres = p_bmc050->BMC050_BUS_READ_FUNC(
				p_bmc050->dev_addr,
				BMC050_NO_REPETITIONS_Z,
				&v_data_u8r, 1);
		*no_repetitions_z = v_data_u8r;
	}
	return comres;
}

BMC050_RETURN_FUNCTION_TYPE bmc050_set_repetitions_Z(
		unsigned char no_repetitions_z)
{
	BMC050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmc050 == BMC050_NULL) {
		comres = E_BMC050_NULL_PTR;
	} else {
		v_data_u8r = no_repetitions_z;
		comres = p_bmc050->BMC050_BUS_WRITE_FUNC(p_bmc050->dev_addr,
				BMC050_NO_REPETITIONS_Z, &v_data_u8r, 1);
	}
	return comres;
}


BMC050_RETURN_FUNCTION_TYPE bmc050_get_raw_xyz(struct bmc050_mdata *mdata)
{
	BMC050_RETURN_FUNCTION_TYPE comres;
	unsigned char a_data_u8r[6];
	if (p_bmc050 == BMC050_NULL) {
		comres = E_BMC050_NULL_PTR;
	} else {
		comres = p_bmc050->BMC050_BUS_READ_FUNC(p_bmc050->dev_addr,
				BMC050_DATAX_LSB, a_data_u8r, 6);

		a_data_u8r[0] = BMC050_GET_BITSLICE(a_data_u8r[0],
				BMC050_DATAX_LSB_VALUEX);
		mdata->datax = (BMC050_S16)((((BMC050_S16)
						((signed char)a_data_u8r[1]))
					<< SHIFT_LEFT_5_POSITION)
				| a_data_u8r[0]);

		a_data_u8r[2] = BMC050_GET_BITSLICE(a_data_u8r[2],
				BMC050_DATAY_LSB_VALUEY);
		mdata->datay = (BMC050_S16)((((BMC050_S16)
						((signed char)a_data_u8r[3]))
					<< SHIFT_LEFT_5_POSITION)
				| a_data_u8r[2]);

		a_data_u8r[4] = BMC050_GET_BITSLICE(a_data_u8r[4],
				BMC050_DATAZ_LSB_VALUEZ);
		mdata->dataz = (BMC050_S16)((((BMC050_S16)
						((signed char)a_data_u8r[5]))
					<< SHIFT_LEFT_7_POSITION)
				| a_data_u8r[4]);
	}
	return comres;
}


BMC050_RETURN_FUNCTION_TYPE bmc050_read_mdataXYZ(struct bmc050_mdata *mdata)
{
	BMC050_RETURN_FUNCTION_TYPE comres;

	unsigned char a_data_u8r[8];

	struct {
		BMC050_S16 raw_dataX;
		BMC050_S16 raw_dataY;
		BMC050_S16 raw_dataZ;
		BMC050_U16 raw_dataR;
	} raw_dataXYZ;

	if (p_bmc050 == BMC050_NULL) {
		comres = E_BMC050_NULL_PTR;
	} else {
		comres = p_bmc050->BMC050_BUS_READ_FUNC(p_bmc050->dev_addr,
				BMC050_DATAX_LSB, a_data_u8r, 8);

		/* Reading data for X axis */
		a_data_u8r[0] = BMC050_GET_BITSLICE(a_data_u8r[0],
				BMC050_DATAX_LSB_VALUEX);
		raw_dataXYZ.raw_dataX = (BMC050_S16)((((BMC050_S16)
						((signed char)a_data_u8r[1])) <<
					SHIFT_LEFT_5_POSITION) | a_data_u8r[0]);

		/* Reading data for Y axis */
		a_data_u8r[2] = BMC050_GET_BITSLICE(a_data_u8r[2],
				BMC050_DATAY_LSB_VALUEY);
		raw_dataXYZ.raw_dataY = (BMC050_S16)((((BMC050_S16)
						((signed char)a_data_u8r[3])) <<
					SHIFT_LEFT_5_POSITION) | a_data_u8r[2]);

		/* Reading data for Z axis */
		a_data_u8r[4] = BMC050_GET_BITSLICE(a_data_u8r[4],
				BMC050_DATAZ_LSB_VALUEZ);
		raw_dataXYZ.raw_dataZ = (BMC050_S16)((((BMC050_S16)
						((signed char)a_data_u8r[5])) <<
					SHIFT_LEFT_7_POSITION) | a_data_u8r[4]);

		/* Reading data for Resistance*/
		a_data_u8r[6] = BMC050_GET_BITSLICE(a_data_u8r[6],
				BMC050_R_LSB_VALUE);
		raw_dataXYZ.raw_dataR = (BMC050_U16)((((BMC050_U16)
						a_data_u8r[7]) <<
					SHIFT_LEFT_6_POSITION) | a_data_u8r[6]);

		/* Compensation for X axis */
		mdata->datax = bmc050_compensate_X(raw_dataXYZ.raw_dataX,
				raw_dataXYZ.raw_dataR);

		/* Compensation for Y axis */
		mdata->datay = bmc050_compensate_Y(raw_dataXYZ.raw_dataY,
				raw_dataXYZ.raw_dataR);

		/* Compensation for Z axis */
		mdata->dataz = bmc050_compensate_Z(raw_dataXYZ.raw_dataZ,
				raw_dataXYZ.raw_dataR);
	}
	return comres;
}

BMC050_S16 bmc050_compensate_X(BMC050_S16 mdata_x, BMC050_U16 data_R)
{
	BMC050_S16 inter_retval;
	if (mdata_x != BMC050_FLIP_OVERFLOW_ADCVAL  /* no overflow */
	   ) {
		inter_retval = ((BMC050_S16)(((BMC050_U16)
				((((BMC050_S32)p_bmc050->dig_xyz1) << 14) /
				 (data_R != 0 ? data_R : p_bmc050->dig_xyz1))) -
				((BMC050_U16)0x4000)));
		inter_retval = ((BMC050_S16)((((BMC050_S32)mdata_x) *
				((((((((BMC050_S32)p_bmc050->dig_xy2) *
			      ((((BMC050_S32)inter_retval) *
				((BMC050_S32)inter_retval)) >> 7)) +
			     (((BMC050_S32)inter_retval) *
			      ((BMC050_S32)(((BMC050_S16)p_bmc050->dig_xy1)
			      << 7)))) >> 9) +
			   ((BMC050_S32)0x100000)) *
			  ((BMC050_S32)(((BMC050_S16)p_bmc050->dig_x2) +
			  ((BMC050_S16)0xA0)))) >> 12)) >> 13)) +
			(((BMC050_S16)p_bmc050->dig_x1) << 3);
	} else {
		/* overflow */
		inter_retval = BMC050_OVERFLOW_OUTPUT;
	}
	return inter_retval;
}

BMC050_S16 bmc050_compensate_Y(BMC050_S16 mdata_y, BMC050_U16 data_R)
{
	BMC050_S16 inter_retval;
	if (mdata_y != BMC050_FLIP_OVERFLOW_ADCVAL  /* no overflow */
	   ) {
		inter_retval = ((BMC050_S16)(((BMC050_U16)(((
			(BMC050_S32)p_bmc050->dig_xyz1) << 14) /
			(data_R != 0 ?
			 data_R : p_bmc050->dig_xyz1))) -
			((BMC050_U16)0x4000)));
		inter_retval = ((BMC050_S16)((((BMC050_S32)mdata_y) *
				((((((((BMC050_S32)
				       p_bmc050->dig_xy2) *
				      ((((BMC050_S32) inter_retval) *
					((BMC050_S32)inter_retval)) >> 7)) +
				     (((BMC050_S32)inter_retval) *
				      ((BMC050_S32)(((BMC050_S16)
				      p_bmc050->dig_xy1) << 7)))) >> 9) +
				   ((BMC050_S32)0x100000)) *
				  ((BMC050_S32)(((BMC050_S16)p_bmc050->dig_y2)
					  + ((BMC050_S16)0xA0))))
				 >> 12)) >> 13)) +
			(((BMC050_S16)p_bmc050->dig_y1) << 3);
	} else {
		/* overflow */
		inter_retval = BMC050_OVERFLOW_OUTPUT;
	}
	return inter_retval;
}

BMC050_S16 bmc050_compensate_Z(BMC050_S16 mdata_z, BMC050_U16 data_R)
{
	BMC050_S32 retval;
	if ((mdata_z != BMC050_HALL_OVERFLOW_ADCVAL)	/* no overflow */
	   ) {
		retval = (((((BMC050_S32)(mdata_z - p_bmc050->dig_z4)) << 15) -
					((((BMC050_S32)p_bmc050->dig_z3) *
					  ((BMC050_S32)(((BMC050_S16)data_R) -
						  ((BMC050_S16)
						   p_bmc050->dig_xyz1))))>>2)) /
				(p_bmc050->dig_z2 +
				 ((BMC050_S16)(((((BMC050_S32)
					 p_bmc050->dig_z1) *
					 ((((BMC050_S16)data_R) << 1)))+
						 (1<<15))>>16))));
		/* saturate result to +/- 2 mT */
		if (retval > BMC050_POSITIVE_SATURATION_Z) {
			retval =  BMC050_POSITIVE_SATURATION_Z;
		} else {
			if (retval < BMC050_NEGATIVE_SATURATION_Z)
				retval = BMC050_NEGATIVE_SATURATION_Z;
		}
	} else {
		/* overflow */
		retval = BMC050_OVERFLOW_OUTPUT;
	}
	return (BMC050_S16)retval;
}

BMC050_RETURN_FUNCTION_TYPE bmc050_soft_reset(void)
{
	BMC050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmc050 == BMC050_NULL) {
		comres = E_BMC050_NULL_PTR;
	} else {
		v_data_u8r = BMC050_ON;

		comres = p_bmc050->BMC050_BUS_READ_FUNC(
				p_bmc050->dev_addr,
				BMC050_POWER_CNTL_SRST7__REG,
				&v_data_u8r, 1);
		v_data_u8r = BMC050_SET_BITSLICE(v_data_u8r,
				BMC050_POWER_CNTL_SRST7,
				BMC050_SOFT_RESET7_ON);
		comres |= p_bmc050->BMC050_BUS_WRITE_FUNC(
				p_bmc050->dev_addr,
				BMC050_POWER_CNTL_SRST7__REG, &v_data_u8r, 1);

		comres |= p_bmc050->BMC050_BUS_READ_FUNC(
				p_bmc050->dev_addr,
				BMC050_POWER_CNTL_SRST1__REG,
				&v_data_u8r, 1);
		v_data_u8r = BMC050_SET_BITSLICE(v_data_u8r,
				BMC050_POWER_CNTL_SRST1,
				BMC050_SOFT_RESET1_ON);
		comres |= p_bmc050->BMC050_BUS_WRITE_FUNC(
				p_bmc050->dev_addr,
				BMC050_POWER_CNTL_SRST1__REG,
				&v_data_u8r, 1);

		p_bmc050->delay_msec(BMC050_DELAY_SOFTRESET);
	}
	return comres;
}


BMC050_RETURN_FUNCTION_TYPE bmc050_get_self_test_XYZ(
		unsigned char *self_testxyz)
{
	BMC050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char a_data_u8r[5], v_result_u8r = 0x00;
	if (p_bmc050 == BMC050_NULL) {
		comres = E_BMC050_NULL_PTR;
	} else {
		comres = p_bmc050->BMC050_BUS_READ_FUNC(
				p_bmc050->dev_addr, BMC050_DATAX_LSB_TESTX__REG,
				a_data_u8r, 5);

		v_result_u8r = BMC050_GET_BITSLICE(a_data_u8r[4],
				BMC050_DATAZ_LSB_TESTZ);

		v_result_u8r = (v_result_u8r << 1);
		v_result_u8r = (v_result_u8r | BMC050_GET_BITSLICE(
					a_data_u8r[2], BMC050_DATAY_LSB_TESTY));

		v_result_u8r = (v_result_u8r << 1);
		v_result_u8r = (v_result_u8r | BMC050_GET_BITSLICE(
					a_data_u8r[0], BMC050_DATAX_LSB_TESTX));

		*self_testxyz = v_result_u8r;
	}
	return comres;
}

BMC050_RETURN_FUNCTION_TYPE bmc050_set_selftest(unsigned char selftest)
{
	BMC050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmc050 == BMC050_NULL) {
		comres = E_BMC050_NULL_PTR;
	} else {
		comres = p_bmc050->BMC050_BUS_READ_FUNC(
				p_bmc050->dev_addr, BMC050_CNTL_S_TEST__REG,
				&v_data1_u8r, 1);
		v_data1_u8r = BMC050_SET_BITSLICE(
				v_data1_u8r, BMC050_CNTL_S_TEST, selftest);
		comres |= p_bmc050->BMC050_BUS_WRITE_FUNC(
				p_bmc050->dev_addr, BMC050_CNTL_S_TEST__REG,
				&v_data1_u8r, 1);
	}
	return comres;
}

BMC050_RETURN_FUNCTION_TYPE bmc050_perform_advanced_selftest(
		BMC050_S16 *diff_z)
{
	BMC050_RETURN_FUNCTION_TYPE comres;
	BMC050_S16 result_positive, result_negative;
	if (p_bmc050 == BMC050_NULL) {
		comres = E_BMC050_NULL_PTR;
	} else {
		/* set sleep mode to prepare for forced measurement.
		 * If sensor is off, this will turn it on
		 * and respect needed delays. */
		comres = bmc050_set_functional_state(BMC050_SLEEP_MODE);

		/* set normal accuracy mode */
		comres |= bmc050_set_repetitions_Z(BMC050_LOWPOWER_REPZ);
		/* 14 repetitions Z in normal accuracy mode */

		/* disable X, Y channel */
		comres |= bmc050_set_control_measurement_x(
				BMC050_CHANNEL_DISABLE);
		comres |= bmc050_set_control_measurement_y(
				BMC050_CHANNEL_DISABLE);

		/* enable positive current and force a
		 * measurement with positive field */
		comres |= bmc050_set_adv_selftest(
				BMC050_ADVANCED_SELFTEST_POSITIVE);
		comres |= bmc050_set_functional_state(BMC050_FORCED_MODE);
		/* wait for measurement to complete */
		p_bmc050->delay_msec(4);

		/* read result from positive field measurement */
		comres |= bmc050_read_mdataZ(&result_positive);

		/* enable negative current and force a
		 * measurement with negative field */
		comres |= bmc050_set_adv_selftest(
				BMC050_ADVANCED_SELFTEST_NEGATIVE);
		comres |= bmc050_set_functional_state(BMC050_FORCED_MODE);
		p_bmc050->delay_msec(4); /* wait for measurement to complete */

		/* read result from negative field measurement */
		comres |= bmc050_read_mdataZ(&result_negative);

		/* turn off self test current */
		comres |= bmc050_set_adv_selftest(
				BMC050_ADVANCED_SELFTEST_OFF);

		/* enable X, Y channel */
		comres |= bmc050_set_control_measurement_x(
				BMC050_CHANNEL_ENABLE);
		comres |= bmc050_set_control_measurement_y(
				BMC050_CHANNEL_ENABLE);

		/* write out difference in positive and negative field.
		 * This should be ~ 200 mT = 3200 LSB */
		*diff_z = (result_positive - result_negative);
	}
	return comres;
}

BMC050_RETURN_FUNCTION_TYPE bmc050_set_control_measurement_x(
		unsigned char enable_disable)
{
	BMC050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmc050 == BMC050_NULL) {
		comres = E_BMC050_NULL_PTR;
	} else {
		comres = p_bmc050->BMC050_BUS_READ_FUNC(
				p_bmc050->dev_addr,
				BMC050_SENS_CNTL_CHANNELX__REG,
				&v_data1_u8r, 1);
		v_data1_u8r = BMC050_SET_BITSLICE(v_data1_u8r,
				BMC050_SENS_CNTL_CHANNELX,
				enable_disable);
		comres |= p_bmc050->BMC050_BUS_WRITE_FUNC(
				p_bmc050->dev_addr,
				BMC050_SENS_CNTL_CHANNELX__REG,
				&v_data1_u8r, 1);
	}
	return comres;
}

BMC050_RETURN_FUNCTION_TYPE bmc050_set_control_measurement_y(
		unsigned char enable_disable)
{
	BMC050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmc050 == BMC050_NULL) {
		comres = E_BMC050_NULL_PTR;
	} else {
		comres = p_bmc050->BMC050_BUS_READ_FUNC(
				p_bmc050->dev_addr,
				BMC050_SENS_CNTL_CHANNELY__REG,
				&v_data1_u8r, 1);
		v_data1_u8r = BMC050_SET_BITSLICE(
				v_data1_u8r,
				BMC050_SENS_CNTL_CHANNELY,
				enable_disable);
		comres |= p_bmc050->BMC050_BUS_WRITE_FUNC(
				p_bmc050->dev_addr,
				BMC050_SENS_CNTL_CHANNELY__REG,
				&v_data1_u8r, 1);
	}
	return comres;
}

BMC050_RETURN_FUNCTION_TYPE bmc050_set_adv_selftest(unsigned char adv_selftest)
{
	BMC050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmc050 == BMC050_NULL) {
		comres = E_BMC050_NULL_PTR;
	} else {
		switch (adv_selftest) {
		case BMC050_ADVANCED_SELFTEST_OFF:
			comres = p_bmc050->BMC050_BUS_READ_FUNC(
					p_bmc050->dev_addr,
					BMC050_CNTL_ADV_ST__REG,
					&v_data1_u8r, 1);
			v_data1_u8r = BMC050_SET_BITSLICE(
					v_data1_u8r,
					BMC050_CNTL_ADV_ST,
					BMC050_ADVANCED_SELFTEST_OFF);
			comres |= p_bmc050->BMC050_BUS_WRITE_FUNC(
					p_bmc050->dev_addr,
					BMC050_CNTL_ADV_ST__REG,
					&v_data1_u8r, 1);
			break;
		case BMC050_ADVANCED_SELFTEST_POSITIVE:
			comres = p_bmc050->BMC050_BUS_READ_FUNC(
					p_bmc050->dev_addr,
					BMC050_CNTL_ADV_ST__REG,
					&v_data1_u8r, 1);
			v_data1_u8r = BMC050_SET_BITSLICE(
					v_data1_u8r,
					BMC050_CNTL_ADV_ST,
					BMC050_ADVANCED_SELFTEST_POSITIVE);
			comres |= p_bmc050->BMC050_BUS_WRITE_FUNC(
					p_bmc050->dev_addr,
					BMC050_CNTL_ADV_ST__REG,
					&v_data1_u8r, 1);
			break;
		case BMC050_ADVANCED_SELFTEST_NEGATIVE:
			comres = p_bmc050->BMC050_BUS_READ_FUNC(
					p_bmc050->dev_addr,
					BMC050_CNTL_ADV_ST__REG,
					&v_data1_u8r, 1);
			v_data1_u8r = BMC050_SET_BITSLICE(
					v_data1_u8r,
					BMC050_CNTL_ADV_ST,
					BMC050_ADVANCED_SELFTEST_NEGATIVE);
			comres |= p_bmc050->BMC050_BUS_WRITE_FUNC(
					p_bmc050->dev_addr,
					BMC050_CNTL_ADV_ST__REG,
					&v_data1_u8r, 1);
			break;
		default:
			break;
		}
	}
	return comres;
}

BMC050_RETURN_FUNCTION_TYPE bmc050_read_mdataZ(BMC050_S16 *mdata_z)
{
	BMC050_RETURN_FUNCTION_TYPE comres;
	BMC050_S16 raw_dataZ;
	BMC050_U16 raw_dataR;
	if (p_bmc050 == BMC050_NULL) {
		comres = E_BMC050_NULL_PTR;
	} else {
		comres = bmc050_get_halldataZ(&raw_dataZ);
		comres |= bmc050_get_mdataResistance(&raw_dataR);

		/* Compensation for Z axis */
		*mdata_z = bmc050_compensate_Z(raw_dataZ, raw_dataR);
	}
	return comres;
}

BMC050_RETURN_FUNCTION_TYPE bmc050_get_halldataZ(BMC050_S16 *mdata_z)
{
	BMC050_RETURN_FUNCTION_TYPE comres;
	unsigned char a_data_u8r[2];
	if (p_bmc050 == BMC050_NULL) {
		comres = E_BMC050_NULL_PTR;
	} else {
		comres = p_bmc050->BMC050_BUS_READ_FUNC(p_bmc050->dev_addr,
				BMC050_DATAZ_LSB, a_data_u8r, 2);
		a_data_u8r[0] = BMC050_GET_BITSLICE(a_data_u8r[0],
				BMC050_DATAZ_LSB_VALUEZ);
		*mdata_z = (BMC050_S16)((((BMC050_S16)
						((signed char)a_data_u8r[1]))
					<< SHIFT_LEFT_7_POSITION)
				| a_data_u8r[0]);
	}
	return comres;
}

BMC050_RETURN_FUNCTION_TYPE bmc050_get_mdataResistance(
		BMC050_U16 *mdata_resistance)
{
	BMC050_RETURN_FUNCTION_TYPE comres;
	unsigned char a_data_u8r[2];
	if (p_bmc050 == BMC050_NULL) {
		comres = E_BMC050_NULL_PTR;
	} else {
		comres = p_bmc050->BMC050_BUS_READ_FUNC(
				p_bmc050->dev_addr,
				BMC050_R_LSB_VALUE__REG,
				a_data_u8r, 2);
		a_data_u8r[0] = BMC050_GET_BITSLICE(a_data_u8r[0],
				BMC050_R_LSB_VALUE);
		*mdata_resistance = (BMC050_U16)
			((((BMC050_U16)a_data_u8r[1])
			  << SHIFT_LEFT_6_POSITION)
			 | a_data_u8r[0]);
	}
	return comres;
}

