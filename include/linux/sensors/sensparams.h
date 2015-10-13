/*
 *  sensparams.h - definitions for all ALS&Prox sensors
 *
 *  Copyright (C) 2010 Yulong Tech. Co., Ltd.
 *  Jay.HF <huangfujie@yulong.com>
 *  Copyright (c) 2010-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __SENSOR_PARAMS_H__
#define __SENSOR_PARAMS_H__

#include <linux/yl_params.h>

enum {
    SENSPARAMS_TYPE_ACCEL = 0,
    SENSPARAMS_TYPE_PROX,
    SENSPARAMS_TYPE_PRESS,
    SENSPARAMS_TYPE_RESV1,
    SENSPARAMS_TYPE_RESV2,
    SENSPARAMS_TYPE_RESV3,
};

int sensparams_read_from_flash(int type, unsigned char *out, int len);
int sensparams_write_to_flash(int type, unsigned char *in, int len);

#endif

