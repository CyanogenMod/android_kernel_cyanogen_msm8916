/*
 *  mma845x.h - Linux kernel modules for 3-Axis Orientation/Motion
 *  Detection Sensor MMA8451/MMA8452/MMA8453
 *
 *  Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#ifndef	__MMA845x_H__
#define	__MMA845x_H__

#include <linux/sensors/accel_common.h>

#define MMA845x_I2C_ADDR             0x1D
#define MMA8451_ID                   0x1A
#define MMA8452_ID                   0x2A
#define MMA8453_ID                   0x3A

#define MMA845x_DRV_NAME             "mma845x"
#define MMA845x_DEVICE_NAME          ACCEL_DRIVER_NAME
#define MMA845x_INPUT_NAME           ACCEL_INPUT_NAME

/* The sensitivity is represented in counts/g. In 2g mode the
sensitivity is 1024 counts/g. In 4g mode the sensitivity is 512
counts/g and in 8g mode the sensitivity is 256 counts/g.
 */
enum {
    MODE_2G = 0,
    MODE_4G,
    MODE_8G,
};

enum {
    MMA_STANDBY = 0,
    MMA_ACTIVED,
};

/*
 * IOCTLs for MMA845x library
 */
#define MMA845x_IOCTL_ACTIVE         ACCEL_IOCTL_ACTIVE
#define MMA845x_IOCTL_SET_DELAY      ACCEL_IOCTL_SET_DELAY
#define MMA845x_IOCTL_GET_DELAY      ACCEL_IOCTL_GET_DELAY
#define MMA845x_IOCTL_CALIBRATE      ACCEL_IOCTL_CALIBRATE

#ifdef	__KERNEL__
struct mma845x_platform_data {
    int poll_interval;
    u8 g_range;
    u8 position;

    int  (*init)(void);
    void (*exit)(void);
    int  (*power_on)(void);
    int  (*power_off)(void);
    int  (*suspend)(void);
    int  (*resume)(void);

    /* set gpio_int[1,2] either to the choosen gpio pin number or to -EINVAL
     * if leaved unconnected
     */
    int gpio_int1;
    int gpio_int2;
};
#endif	/* __KERNEL__ */

#endif	/* __MMA845x_H__ */

