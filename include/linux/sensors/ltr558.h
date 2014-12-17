/***********************************************************************
 **
 **  Copyright (C), 2013-2015, Yulong Tech. Co., Ltd.
 **  FileName:		ltr558.h
 **  Description:	Linux device driver for ltr558 ambient light and proximity sensors 
 **  Author:		longjiang
 **  Version:		1.00
 **  Date:             	2013-09-25                                              
 **
 ***********************************************************************/


#ifndef  __LTR558_H__
#define  __LTR558_H__

#include <linux/sensors/alsprox_common.h>

#define LTR558_DRIVER_NAME                "ltr558"
#define LTR558_DEVICE_NAME                ALSPROX_DEVICE_NAME
#define LTR558_INPUT_NAME_L               "ltr558_light"
#define LTR558_INPUT_NAME_P               "ltr558_proximity"

/* ioctl numbers*/
#define LTR558_IOCTL_ALS_ON               ALSPROX_IOCTL_ALS_ON
#define LTR558_IOCTL_ALS_OFF              ALSPROX_IOCTL_ALS_OFF
#define LTR558_IOCTL_PROX_ON              ALSPROX_IOCTL_PROX_ON
#define LTR558_IOCTL_PROX_OFF             ALSPROX_IOCTL_PROX_OFF
#define LTR558_IOCTL_PROX_OFFSET          ALSPROX_IOCTL_PROX_OFFSET
#define LTR558_IOCTL_PROX_CALIBRATE       ALSPROX_IOCTL_PROX_CALIBRATE

/* LTR-558 Registers */
#define LTR558_ALS_CONTR	0x80
#define LTR558_PS_CONTR		0x81
#define LTR558_PS_LED		0x82
#define LTR558_PS_N_PULSES	0x83
#define LTR558_PS_MEAS_RATE	0x84
#define LTR558_ALS_MEAS_RATE	0x85
#define LTR558_MANUFACTURER_ID	0x87

#define LTR558_INTERRUPT	0x8F
#define LTR558_PS_THRES_UP_0	0x90
#define LTR558_PS_THRES_UP_1	0x91
#define LTR558_PS_THRES_LOW_0	0x92
#define LTR558_PS_THRES_LOW_1	0x93

#define LTR558_ALS_THRES_UP_0	0x97
#define LTR558_ALS_THRES_UP_1	0x98
#define LTR558_ALS_THRES_LOW_0	0x99
#define LTR558_ALS_THRES_LOW_1	0x9A

#define LTR558_INTERRUPT_PERSIST 0x9E

/* 558's Read Only Registers */
#define LTR558_ALS_DATA_CH1_0	0x88
#define LTR558_ALS_DATA_CH1_1	0x89
#define LTR558_ALS_DATA_CH0_0	0x8A
#define LTR558_ALS_DATA_CH0_1	0x8B
#define LTR558_ALS_PS_STATUS	0x8C
#define LTR558_PS_DATA_0	0x8D
#define LTR558_PS_DATA_1	0x8E


/* Basic Operating Modes */
#define LTR558_MODE_ALS_ON_Range1	0x0b //0x3B
#define LTR558_MODE_ALS_ON_Range2	0x03 //0x33
#define LTR558_MODE_ALS_StdBy		0x00

#define LTR558_MODE_PS_ON_Gain1	0x03
#define LTR558_MODE_PS_ON_Gain4	0x07
#define LTR558_MODE_PS_ON_Gain8	0x0B
#define LTR558_MODE_PS_ON_Gain16	0x0f  //0x0C
#define LTR558_MODE_PS_StdBy		0x00

#define LTR558_PS_RANGE1 	1
#define LTR558_PS_RANGE2	2
#define LTR558_PS_RANGE4 	4
#define LTR558_PS_RANGE8	8

#define LTR558_ALS_RANGE1_320	1
#define LTR558_ALS_RANGE2_64K 	2

/* Power On response time in ms */
#define LTR558_PON_DELAY	600
#define LTR558_WAKEUP_DELAY	10

struct ltr558_platform_data {
    int gpio_int;
	unsigned int irq_gpio_flags;
    int als_coff1;
    int als_coff2;

    int  (*init)(void);
    void (*exit)(void);
    int  (*power_on)(void);
    int  (*power_off)(void);
    int  (*suspend)(void);
    int  (*resume)(void);
};

#endif
