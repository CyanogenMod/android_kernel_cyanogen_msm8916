/*
* drivers/misc/lps25.h
*
* STMicroelectronics LPS25H Pressure / Temperature Sensor module driver
*
* Copyright (C) 2013 STMicroelectronics- AMS/High-End Sensors and Analog Division
* Environmental Sensor Application Development Team
* Lorenzo Sarchi (lorenzo.sarchi@st.com)
* Version: 1.0.1, Date: 2013/Oct/18
*
* Based on the original work from Matteo Dameno and Carmine Iascone
* (upated to revision 1.0.0 2012/Nov/21):
*
* Please refer to Raffaele Di Vaio (raffaele.di-vaio@st.com) as contact and update point
*
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
* 02110-1301 USA
*
*/
/******************************************************************************
 Revision 0.0.1 2013/Oct/14: 1st draft
	moved to input/misc
******************************************************************************/

#ifndef	__LPS25_H__
#define	__LPS25_H__

#define LPS25_PRS_MIN_POLL_PERIOD_MS	1

#define	LPS25_SAD0L				0x00
#define	LPS25_SAD0H				0x01
#define	LPS25_PRS_I2C_SADROOT	0x2E
#define	LPS25_PRS_I2C_SAD_L		((LPS25_I2C_SADROOT<<1)|SAD0L)
#define	LPS25_PRS_I2C_SAD_H		((LPS25_I2C_SADROOT<<1)|SAD0H)
#define	LPS25_PRS_DEV_NAME		"lps25h"

/* input define mappings */
#define LPS25_ABS_PR		ABS_PRESSURE
#define LPS25_ABS_TEMP	ABS_GAS
#define LPS25_ABS_DLTPR	ABS_MISC

/* Barometer and Termometer output data rate ODR */
#define	LPS25_PRS_ODR_ONESH	0x00	/* one shot both		*/
#define	LPS25_PRS_ODR_1_1	0x10	/*  1  Hz baro,  1  Hz term ODR	*/
#define	LPS25_PRS_ODR_7_7	0x50	/*  7  Hz baro,  7  Hz term ODR	*/
#define	LPS25_PRS_ODR_12_12	0x60	/* 12.5Hz baro, 12.5Hz term ODR	*/
#define	LPS25_PRS_ODR_25_25	0x70	/* 25  Hz baro, 25  Hz term ODR	*/

/*	Pressure section defines		*/
/*	Pressure Sensor Operating Mode		*/
#define	LPS25_PRS_ENABLE		0x01
#define	LPS25_PRS_DISABLE		0x00

/*	Output conversion factors		*/
#define	LPS25_SENSITIVITY_T		480	/* =	480 LSB/degrC	*/
#define	LPS25_SENSITIVITY_P		4096	/* =	LSB/mbar	*/
#define	LPS25_SENSITIVITY_P_SHIFT	12	/* =	4096 LSB/mbar	*/
#define	LPS25_TEMPERATURE_OFFSET	42.5f	/* =	42.5 degrC	*/

#ifdef __KERNEL__
struct lps25_prs_platform_data {
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	unsigned int poll_interval;
	unsigned int min_interval;
};

#endif /* __KERNEL__ */

#endif  /* __LPS25_H__ */
