/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_ATMEL_MXT_TS_H
#define __LINUX_ATMEL_MXT_TS_H

#include <linux/types.h>

#define ATMEL_CONFIG_FILE_NAME "maxtouch.cfg"

#define MXT_MAGIC                 0x5A
#define MXT_IOCTL_CALIBRATE       _IOW(MXT_MAGIC, 0x01, short)
#define MXT_IOCTL_SETOFFSET       _IOW(MXT_MAGIC, 0x02, short)
#define MXT_IOCTL_GETSTATUS       _IOR(MXT_MAGIC, 0x03, unsigned int)

enum maxfusion_reportid {
    RAWACCEL  = 0,
    RAWGYRO   = 1,
    RAWMAG    = 2,
    RAWTEMP   = 3,
    ACCEL     = 4,
    LIN_ACCEL = 5,
    GYRO      = 6,
    MAG       = 7,
    TILT      = 8,
    ROT       = 9,
    TEMP      = 10,
    MOTSTATE  = 11,
    STEP      = 12,
    TAP       = 13,
    SHAKE     = 14,
    GRAVITY,
};

/* The platform data for the Atmel maXTouch touchscreen driver */
struct mxt_platform_data {
    unsigned long irqflags;
    unsigned char t19_num_keys;
    const unsigned int *t19_keymap;
    int t15_num_keys;
    const unsigned int *t15_keymap;
    int gpio_irq;
    int gpio_reset;
    int gpio_pwr;
    int gpio_mode;
    int gyro_irq;
    int meg_irq;
    const char *cfg_name;
};

#endif /* __LINUX_ATMEL_MXT_TS_H */
