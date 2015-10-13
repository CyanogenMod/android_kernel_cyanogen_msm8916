/*
 *  sensparams.c - definitions for storage all sensors parameters
 *
 *  Copyright (C) 2010 Yulong Tech. Co., Ltd.
 *  Jay.HF <huangfujie@yulong.com>
 *  Copyright (c) 2010-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <asm/errno.h>
#include <asm/delay.h>
#include <linux/poll.h> 
#include <linux/wakelock.h>
#include <linux/sensors/sensparams.h>
#include <linux/yl_params.h>

int sensparams_write_to_flash(int type, unsigned char *in, int len) {
    int ret = 0;
    struct ProductlineInfo *ptr = NULL;
    unsigned char param[512] ="PRODUCTLINE";
    int size = sizeof(param);

    if (in == NULL || len <= 0) {
        printk(KERN_ERR"%s: error, in = %s, len = %d\n", __func__, in, len);
        return -1;
    }

    ret = yl_params_kernel_read(param, size);//, 1);
    if (ret != size) {
        printk(KERN_ERR"%s: write %d type sensor param failed\n", __func__, type);
        return -1;
    }

    ptr = (struct ProductlineInfo *)param;
    switch(type) {
        case SENSPARAMS_TYPE_ACCEL:
            memcpy(&ptr->AccInfo, in, len);
            break;
            
        case SENSPARAMS_TYPE_PROX:
            memcpy(&ptr->LightProxInfo, in, len);
            break;
            
        case SENSPARAMS_TYPE_PRESS:
            memcpy(&ptr->PressInfo, in, len);
            break;
            
        case SENSPARAMS_TYPE_RESV1:
            memcpy(&ptr->SensorReserved1, in, len);
            break;
            
        case SENSPARAMS_TYPE_RESV2:
            memcpy(&ptr->SensorReserved2, in, len);
            break;
            
        case SENSPARAMS_TYPE_RESV3:
            memcpy(&ptr->SensorReserved3, in, len);
            break;
            
        default:
            printk(KERN_ERR"%s: %d type sensor is not support\n", __func__, type);
            return -1;
    }

    ret = yl_params_kernel_write(param, size);//, 1);
	if (ret != size) {
        printk(KERN_ERR"%s: read %d type sensor param failed\n", __func__, type);
        return -1;
    }

	return len;
}
EXPORT_SYMBOL_GPL(sensparams_write_to_flash);

int sensparams_read_from_flash(int type, unsigned char *out, int len) {
    int ret = 0;
    struct ProductlineInfo *ptr = NULL;
    unsigned char param[512] ="PRODUCTLINE";
    int size = sizeof(param);

    if (out == NULL || len <= 0) {
        printk(KERN_ERR"%s: error, out = %s, len = %d\n", __func__, out, len);
        return -1;
    }

    ret = yl_params_kernel_read(param, size);//, 1);
    if (ret != size) {
        printk(KERN_ERR"%s: read %d type sensor param failed\n", __func__, type);
        return -1;
    }

    ptr = (struct ProductlineInfo *)param;
    switch(type) {
        case SENSPARAMS_TYPE_ACCEL:
            memcpy(out, &ptr->AccInfo, len);
            break;
            
        case SENSPARAMS_TYPE_PROX:
            memcpy(out, &ptr->LightProxInfo, len);
            break;
            
        case SENSPARAMS_TYPE_PRESS:
            memcpy(out, &ptr->PressInfo, len);
            break;
            
        case SENSPARAMS_TYPE_RESV1:
            memcpy(out, &ptr->SensorReserved1, len);
            break;
            
        case SENSPARAMS_TYPE_RESV2:
            memcpy(out, &ptr->SensorReserved2, len);
            break;
            
        case SENSPARAMS_TYPE_RESV3:
            memcpy(out, &ptr->SensorReserved3, len);
            break;
            
        default:
            printk(KERN_ERR"%s: %d type sensor is not support\n", __func__, type);
            return -1;
    }
    return len;
}
EXPORT_SYMBOL_GPL(sensparams_read_from_flash);

