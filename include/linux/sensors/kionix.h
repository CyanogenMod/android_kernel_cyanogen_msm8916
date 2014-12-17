/* include/linux/input/kionix_accel.h - Kionix accelerometer driver
 *
 * Copyright (C) 2012 Kionix, Inc.
 * Written by Kuching Tan <kuchingtan@kionix.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __KIONIX_ACCEL_H__
#define __KIONIX_ACCEL_H__

#include "accel_common.h"

#define KIONIX_ACCEL_I2C_ADDR		0x0F
#define KIONIX_ACCEL_NAME			"kionix_accel"
#define KIONIX_ACCEL_IRQ			"kionix-irq"

//longjiang add start 20130930
#define KIONIX_DEVICE_NAME			"yl_acc_sensor"
#define KIONIX_INPUT_NAME			"kionix"

#define KIONIX_IOCTL_APP_SET_AFLAG	ACCEL_IOCTL_ACTIVE
#define KIONIX_IOCTL_APP_SET_DELAY	ACCEL_IOCTL_SET_DELAY
#define KIONIX_IOCTL_APP_CALIBRATE	ACCEL_IOCTL_CALIBRATE
#define KIONIX_IOCTL_APP_OFFSET	ACCEL_IOCTL_SETOFFSET//modified by wudongxing for factory pattern.2013.11.07


//add end

struct kionix_accel_platform_data {
	/* Although the accelerometer can perform at high ODR,
	 * there is a need to keep the maximum ODR to a lower
	 * value due to power consumption or other concern.
	 * Use this variable to set the minimum allowable
	 * interval for data to be reported from the
	 * accelerometer. Unit is measured in milli-
	 * seconds. Recommended value is 5ms. */
	unsigned int min_interval;
	/* Use this variable to set the default interval for
	 * data to be reported from the accelerometer. This
	 * value will be used during driver setup process,
	 * but can be changed by the system during runtime via
	 * sysfs control. Recommended value is 200ms.*/
	unsigned int poll_interval;

        unsigned char map_x,map_y,map_z;
        unsigned char neg_x,neg_y,neg_z;

	/* This variable controls the corresponding direction
	 * of the accelerometer that is mounted on the board
	 * of the device. Refer to the porting guide for
	 * details. Valid value is 1 to 8. */
	unsigned char accel_direction;

	/* Use this variable to choose whether or not to use
	 * DRDY hardware interrupt mode to trigger a data
	 * report event instead of using software polling.
	 * Note that for those accelerometer model that does
	 * not support DRDY hardware interrupt, the driver
	 * will revert to software polling mode automatically.
	 * Valid value is 0 or 1.*/
	bool accel_irq_use_drdy;

	/* Use this variable to control the number of
	 * effective bits of the accelerometer output.
	 * Use the macro definition to select the desired
	 * number of effective bits. */
	#define KIONIX_ACCEL_RES_12BIT	0
	#define KIONIX_ACCEL_RES_8BIT	1
	#define KIONIX_ACCEL_RES_6BIT	2
	unsigned char accel_res;

	/* Use this variable to control the G range of
	 * the accelerometer output. Use the macro definition
	 * to select the desired G range.*/
	#define KIONIX_ACCEL_G_2G		0
	#define KIONIX_ACCEL_G_4G		1
	#define KIONIX_ACCEL_G_6G		2
	#define KIONIX_ACCEL_G_8G		3
	unsigned char accel_g_range;

	/* Optional callback functions that can be implemented
	 * on per product basis. If these callbacks are defined,
	 * they will be called by the driver. */
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
	//longjiang add platform code start 20130520
	int gpio_init;
	int (*suspend)(void);
	int (*resume)(void);
	//add end
};
#endif  /* __KIONIX_ACCEL_H__ */
