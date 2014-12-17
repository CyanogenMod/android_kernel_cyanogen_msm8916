/* include/linux/cm32131.h
 *
 * Copyright (C) 2013 Capella Microsystems Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_CM32131_H
#define __LINUX_CM32131_H

#define CM32131_I2C_NAME "cm32131"

/* Define Slave Address*/
#define	CM32131_ALS_ADDR	0x90>>1
#define	CM32131_ARA_ADDR	0x19>>1

#define ALS_CALIBRATED	0x6F17

/*cm32131*/
/*Define ALS Command Code*/
#define	ALS_CMD		0x00
#define	ALS_HW  	0x01
#define	ALS_LW	    0x02
#define	ALS_DATA    0x04

/*for ALS command*/
#define CM32131_ALS_SM_1		(0 << 11)
#define CM32131_ALS_SM_2		(1 << 11)
#define CM32131_ALS_SM_HALF		(2 << 11)
#define CM32131_ALS_IT_125MS	(0 << 6)
#define CM32131_ALS_IT_250MS	(1 << 6)
#define CM32131_ALS_IT_500MS	(2 << 6)
#define CM32131_ALS_IT_1000MS	(3 << 6)
#define CM32131_ALS_PERS_1		(0 << 4)
#define CM32131_ALS_PERS_2		(1 << 4)
#define CM32131_ALS_PERS_4		(2 << 4)
#define CM32131_ALS_PERS_8		(3 << 4)
#define CM32131_ALS_INT_EN		(1 << 1)
#define CM32131_ALS_SD			(1 << 0)/*enable/disable ALS func, 1:disable , 0: enable*/
#define CM32131_ALS_DEFAULT		0xA704

#define LS_PWR_ON				(1 << 0)

struct cm32131_platform_data {
	int intr;
	uint16_t levels[10];
	uint16_t golden_adc;
	int (*power)(int, uint8_t); /* power to the chip */
	uint8_t ALS_slave_address;  	
	uint8_t check_interrupt_add;
	uint16_t is_cmd;
};

#endif
