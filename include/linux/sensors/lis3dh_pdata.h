/*
 *  Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *  COPYRIGHT (C) 2010, STMicroelectronics
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 and
 *  only version 2 as published by the Free Software Foundation.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */
/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
*
* File Name          : lis3dh_misc.h
* Authors            : MH - C&I BU - Application Team
*		     : Matteo Dameno (matteo.dameno@st.com)
*		     : Carmine Iascone (carmine.iascone@st.com)
* Version            : V 1.0.8
* Date               : 2011/Apr/01
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*
*******************************************************************************/
/*******************************************************************************
Version History.

Revision 1-0-0 05/11/2009
	First Release
Revision 1-0-1 26/01/2010
	Linux K&R Compliant Release
Revision 1-0-5 16/08/2010
	Interrupt Management
Revision 1-0-6 10/11/2010
	ioclt no more supported
	sysfs support
Revision 1-0-7 26/11/2010
	moved to input/misc
	manages use/non-use of interrupts
 Revision 1.0.8 2010/Apr/01
  corrects a bug in interrupt pin management in 1.0.7 acc side

*******************************************************************************/

#ifndef	__LIS3DH_H__
#define	__LIS3DH_H__

#define SAD0L			0x00
#define SAD0H			0x01
#define LIS3DH_ACC_I2C_SADROOT	0x0C
#define LIS3DH_ACC_I2C_SAD_L	((LIS3DH_ACC_I2C_SADROOT<<1)|SAD0L)
#define LIS3DH_ACC_I2C_SAD_H	((LIS3DH_ACC_I2C_SADROOT<<1)|SAD0H)

#define LIS3DH_ACC_DEV_NAME		"lis3dh"

//longjiang add platform code start 20130520
#define GSENSOR_DEVICE_NAME		"yl_acc_sensor"
#define GSENSOR_INPUT_NAME		"yl_acc_input"

struct acc_offset {
    signed short  key;	    //CalStatus 0:without calibration 1:success 2:fail
    signed short  x;	    //xoffset
    signed short  y;	    //yoffset
    signed short  z;	    //zoffset
};
//add end

/************************************************/
/* 	Accelerometer defines section	 	*/
/************************************************/

/* Accelerometer Sensor Full Scale */
#define LIS3DH_ACC_FS_MASK		0x30
#define LIS3DH_ACC_G_2G 		0x00
#define LIS3DH_ACC_G_4G 		0x10
#define LIS3DH_ACC_G_8G 		0x20
#define LIS3DH_ACC_G_16G		0x30
//vincent

#define GSENSORIO 0x1D
#define GSENSOR_IOCTL_APP_SET_AFLAG		_IOW(GSENSORIO, 1, short)
#define GSENSOR_IOCTL_APP_SET_DELAY		_IOW(GSENSORIO, 2, short)
#define GSENSOR_IOCTL_APP_CALIBRATE		_IOR(GSENSORIO, 3, short)
#define GSENSOR_IOCTL_APP_OFFSET		_IOR(GSENSORIO, 4, short)

typedef enum
{
    MMA8452_STANDBY,
    MMA8452_MEASUREMENT,
    MMA8452_LEVEL_DETECTION,
    MMA8452_PULSE_DETECTION
}mma8452_mode_type;

//vinent
#ifdef	__KERNEL__
struct lis3dh_acc_platform_data {
	int poll_interval;
	int min_interval;

	u8 g_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
	//longjiang add platform code start 20130520
	int (*suspend)(void);
	int (*resume)(void);
	//add end

	/* set gpio_int[1,2] either to the choosen gpio pin number or to -EINVAL
	 * if leaved unconnected
	 */
	int gpio_int1;
	int gpio_int2;
};
#endif	/* __KERNEL__ */

#endif	/* __LIS3DH_H__ */

