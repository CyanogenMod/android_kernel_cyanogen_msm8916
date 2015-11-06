/* drivers/input/touchscreen/gslX68X.h
 * 
 * 2010 - 2013 SLIEAD Technology.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be a reference 
 * to you, when you are integrating the SLIEAD's CTP IC into your system, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * General Public License for more details.
 * 
 */

//#include <mach/ldo.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/sensors.h>
#include <linux/wakelock.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

//#define GSL_DEBUG		//调试信息开关，打开则输出调试信息
#define GSL_TIMER		//定时器开关
#define TPD_PROC_DEBUG	//proc文件系统接口红开关
#define GSL9XX_VDDIO_1800  1
//#define GSL_REPORT_POINT_SLOT
//#define GSL_PROXIMITY_SENSOR //Proximity sensor replaced by touch panel
#define GSL_GESTURE			 //Resuming the circcity by touch panel
#define GSL_GESTURE_WAKELOCK_DUR msecs_to_jiffies(200)

/*define i2c addr and device name*/
#define GSL_TS_ADDR 				0x40
#define GSL_TS_NAME				"GSL_TP"

/*define irq and reset gpio num*/
#define GSL_RST_GPIO_NUM	914
#define GSL_IRQ_GPIO_NUM	915
#define GSL_IRQ_NUM			gpio_to_irq(GSL_IRQ_GPIO_NUM)
//#define GSL_IRQ_NUM			sprd_alloc_gpio_irq(GSL_IRQ_GPIO_NUM)
#define GSL_IRQ_NAME		"gsl_irq"
#define GSL_RST_NAME		"gsl_reset"


#if 0
/*vdd power*/
#define GSL_POWER_ON()	do{\
			}while(0)
#endif		
//#define GSL_TIMER_CHECK_CIRCLE		200
#define GSL_PRESSURE				50

/*debug of time*/
#define TPD_DEBUG_TIME				0x20150128

/*define screen of resolution ratio*/
#define GSL_MAX_X		720
#define GSL_MAX_Y		1280 

/*i2c of adapter number*/

/*virtual keys*/
//#define TOUCH_VIRTUAL_KEYS

/*button of key*/
#define GSL_HAVE_TOUCH_KEY 			1
#if GSL_HAVE_TOUCH_KEY
	struct key_data{
		u16 key;
		u32 x_min;
		u32 x_max;
		u32 y_min;
		u32 y_max;
	};
	#define GSL_KEY_NUM	 3	 
	struct key_data gsl_key_data[GSL_KEY_NUM] = {
		{KEY_MENU,100,140,880,920},
		{KEY_HOMEPAGE,220,260,880,920},
		{KEY_BACK,340,380,880,920},
	};
#endif

struct gsl_touch_info{
	int x[10];
	int y[10];
	int id[10];
	int finger_num;
};

struct gsl_ts_platform_data {
	int irq_gpio_number;
	u32 irq_gpio_flags;
	int reset_gpio_number;	
	u32 reset_gpio_flags;
};

struct gsl_ts_data{
	struct i2c_client		*client;
	struct input_dev		*idev;
#ifdef GSL_PROXIMITY_SENSOR
	struct input_dev 		*input_dev_ps;
	struct sensors_classdev ps_cdev;
#endif
	#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
	#endif

	struct gsl_touch_info		*cinfo;
	u32 gsl_halt_flag;			//0 normal;1 the machine is suspend;
	u32 gsl_sw_flag;			//0 normal;1 download the firmware;
	u32 gsl_up_flag;			//0 normal;1 have one up event;
	u32 gsl_point_state;		//the point down and up of state	
#ifdef GSL_TIMER
	struct delayed_work		timer_work;
	struct workqueue_struct	*timer_wq;
	volatile int gsl_timer_flag;	//0:first test	1:second test 2:doing gsl_load_fw
	unsigned int gsl_timer_data;		
	u32 watchdog_counter;
#endif
#if GSL_HAVE_TOUCH_KEY
	int gsl_key_state;
#endif
	#if 0
	struct gsl_ts_platform_data *pdata;
	#endif

	struct mutex hw_lock;
#ifdef GSL_GESTURE
	struct wake_lock gesture_wake_lock;
#endif
};

#ifdef GSL_PROXIMITY_SENSOR
static struct sensors_classdev sensors_proximity_cdev = {
	.name = "proximity",
	.vendor = "proximity",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
#endif

#define GSL_ALG_ID
#ifdef GSL_ALG_ID
extern unsigned int gsl_version_id(void);
extern void gsl_alg_id_main(struct gsl_touch_info *cinfo);
extern void gsl_DataInit(int *ret);
extern unsigned int gsl_mask_tiaoping(void);
extern int gsl_obtain_gesture(void);
extern void gsl_FunIICRead(unsigned int (*fun) (unsigned int *,unsigned int,unsigned int));
extern void gsl_GestureExternInt(unsigned int *model, int len);

#endif

struct fw_data
{
    u32 offset : 8;
    u32 : 0;
    u32 val;
};

#if 1
#include "gsl_ts_8074_doubleclick.h"//new version with character gesture and double click.
#else
#include "gsl_ts_8074.h"
#endif

static u8 gsl_cfg_index = 0;
struct fw_config_type
{
	const struct fw_data *fw;
	unsigned int fw_size;
#ifdef GSL_ALG_ID
	unsigned int *data_id;
	unsigned int data_size;
#endif
};
static struct fw_config_type gsl_cfg_table[9] = {
/*0*/{GSLX68X_FW,(sizeof(GSLX68X_FW)/sizeof(struct fw_data)),
	gsl_config_data_id,(sizeof(gsl_config_data_id)/4)},
/*1*/{NULL,0,NULL,0}, //GSL_TS_CFG_CHAOSHENG_4_2_OGS_1680,
/*2*/{NULL,0,NULL,0}, //GSL_TS_CFG_BAOMING_4_5_G_G_2682,
/*3*/{NULL,0,NULL,0}, //GSL_TS_CFG_LIYOU_4_5_P_G_1688,
/*4*/{NULL,0,NULL,0},
/*5*/{NULL,0,NULL,0},
/*6*/{NULL,0,NULL,0},
/*7*/{NULL,0,NULL,0},
/*8*/{NULL,0,NULL,0},
};
