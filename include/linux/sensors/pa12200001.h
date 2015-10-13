/*
 *  pa12200001.h - Linux kernel modules for ambient light + proximity sensor
 *
 *  Copyright (c) 2013, All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 and
 *  only version 2 as published by the Free Software Foundation.

 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __PA12200001_H__
#define __PA12200001_H__

#include <linux/types.h>
#include <linux/ioctl.h>

#include <linux/sensors/alsprox_common.h>
/*pa12200001 als/ps Default*/

#define PA12_I2C_ADDRESS        0x1E    //7 bit Address

#define PA12200001_INPUT_NAME_P  "PA12200001_proximity"
#define PA12200001_INPUT_NAME_L  "PA12200001_light"

/*pa12200001 als/ps sensor register map*/
#define REG_CFG0            0X00    // ALS_GAIN(D5-4),PS_ON(D1) ALS_ON(D0)
#define REG_CFG1            0X01    // LED_CURR(D5-4),PS_PRST(D3-2),ALS_PRST(D1-0)
#define REG_CFG2            0X02    // PS_MODE(D7-6),CLEAR(D4),INT_SET(D3-2),PS_INT(D1),ALS_INT(D0)
#define REG_CFG3            0X03    // INT_TYPE(D6),PS_PERIOD(D5-3),ALS_PERIOD(D2-0)
#define REG_ALS_TL_LSB      0X04    // ALS Threshold Low LSB
#define REG_ALS_TL_MSB      0X05    // ALS Threshold Low MSB
#define REG_ALS_TH_LSB      0X06    // ALS Threshold high LSB
#define REG_ALS_TH_MSB      0X07    // ALS Threshold high MSB
#define REG_PS_TL           0X08    // PS Threshold Low
#define REG_PS_TH           0X0A    // PS Threshold High
#define REG_ALS_DATA_LSB    0X0B    // ALS DATA
#define REG_ALS_DATA_MSB    0X0C    // ALS DATA
#define REG_PS_DATA     0X0E    // PS DATA
#define REG_PS_OFFSET       0X10    // TBD
#define REG_PS_SET          0X11    // 0x03

#define ALS_ACTIVE          0x01
#define PS_ACTIVE               0x02

#define ALS_INT_ACTIVE      0x01
#define PS_INT_ACTIVE           0x02

/*i2c re-try settin*/
#define I2C_RETRY                   5
#define I2C_RETRY_DELAY      5

/*Use fast calibration ?*/
#define PA12_FAST_PS_CAL      1
#define PA12_CAL_MAX_ALS_COUNT      19000   //TBD
#define PA12_FAST_CAL_TOLERANCE     30      //TBD
#define PA12_PS_OFFSET_DEFAULT  20      /* for X-talk cannceling */
#define PA12_PS_OFFSET_EXTRA        4
#define PA12_PS_OFFSET_MAX      100

struct pa12200001_platform_data {
    int (*power_onoff)(int onoff);
    int                 irq;  /* proximity/light-sensor- external irq*/
    int                       gpio_int;
    unsigned int          irq_gpio_flags;
    unsigned char       pa12_ps_th_high;//40
    unsigned char       pa12_ps_th_low;//25
    unsigned char       pa12_ps_th_max;//255
    unsigned char       pa12_ps_th_min;//0
    unsigned char       pa12_ps_offset_default;// for X-talk cannceling//5
    unsigned char       pa12_als_gain;// 0:125lux 1:1000lux 2:2000lux 3:10000lux//1
    unsigned char       pa12_led_curr;// 0:150mA 1:100mA 2:50mA 3:25mA//0
    unsigned char       pa12_ps_prst;// 0:1point 1:2points 2:4points 3:8points (for INT)//2
    unsigned char       pa12_als_prst;// 0:1point 1:2points 2:4points 3:8points (for INT)//0
    unsigned char       pa12_int_set;// 0:ALS interrupt only 1:PS interrupt only 3:BOTH interrupt//1
    unsigned char       pa12_ps_mode;// 0:OFFSET MEASURE 3:NORMAL//3
    unsigned char       pa12_int_type;// 0:Window type 1:Hysteresis type for Auto Clear flag//1
    unsigned char       pa12_ps_period;// 2:25 ms 3:50 ms sleep time//1
    unsigned char       pa12_als_period;// 0 ms//0
    unsigned char       pa12_ps_fast_run_cal;
    unsigned char       pa12_fast_cal_tolerance;
    unsigned char       pa12_ps_offset_max;
};

#endif
