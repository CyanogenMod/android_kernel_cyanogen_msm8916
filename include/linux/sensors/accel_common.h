/*
 *  accel_common.h - definitions for all Accelerator sensors
 *
 *  Copyright (C) 2010 Yulong Tech. Co., Ltd.
 *  Jay.HF <huangfujie@yulong.com>
 *  Copyright (c) 2010-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ACCEL_COMMON_H__
#define __ACCEL_COMMON_H__

#include <linux/ioctl.h>

#define ACCEL_DEVICE_NAME          "yl_acc_sensor"
#define ACCEL_DRIVER_NAME          "yl_acc_sensor"
#define ACCEL_INPUT_NAME           "yl_acc_input"

#define ACCEL_MAGIC                 0x1D
#define ACCEL_IOCTL_ACTIVE          _IOW(ACCEL_MAGIC, 0x01, short)
#define ACCEL_IOCTL_SET_DELAY       _IOW(ACCEL_MAGIC, 0x02, short)
#define ACCEL_IOCTL_GET_DELAY       _IOR(ACCEL_MAGIC, 0x03, short)
#define ACCEL_IOCTL_CALIBRATE       _IOW(ACCEL_MAGIC, 0x04, short)
#define ACCEL_IOCTL_SETOFFSET       _IOW(ACCEL_MAGIC, 0x05, short)

struct acc_offset{
	signed short key;	//calibrate status
	signed short x;		//x offset
	signed short y;		//y offset
	signed short z;		//z offset
};

#endif

