/* drivers/input/touchscreen/gt813_827_828.h
 * 
 * 2010 - 2012 Goodix Technology.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be a reference 
 * to you, when you are integrating the GOODiX's CTP IC into your system, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * General Public License for more details.
 * 
 * Version:1.0
 *      V1.0:2012/08/31,first release.
 */

#ifndef _LINUX_GOODIX_TOUCH_H
#define	_LINUX_GOODIX_TOUCH_H

#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include<linux/mutex.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/input/touchscreen_yl.h>


struct goodix_ts_data {
    spinlock_t irq_lock;
    struct i2c_client *client;
    struct input_dev  *input_dev;
    struct hrtimer timer;
    struct work_struct  work;
#ifdef CONFIG_FB
	struct notifier_block fb_notif;
#endif
    s32 irq_is_disable;
    s32 use_irq;
    u16 abs_x_max;
    u16 abs_y_max;
    u8  max_touch_num;
    u8  int_trigger_type;
    u8  green_wake_mode;
    u8  chip_type;
    u8  enter_update;
    u8  gtp_is_suspend;
    u8  gtp_rawdiff_mode;
    u8  gtp_cfg_len;
    u8  fixed_cfg;
    spinlock_t esd_lock;
    u8  esd_running;
    s32 clk_tick_cnt;
    u8  fw_error;
    struct mutex reset_mutex;
    struct mutex doze_mutex;
    struct tw_platform_data *pdata;		
    struct pinctrl *ts_pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
};

extern u16 show_len;
extern u16 total_len;

//***************************PART3:OTHER define*********************************
#define GTP_DRIVER_VERSION    "V2.1.2<2013/12/31>"
#define GTP_I2C_NAME          "Goodix-TS"
#define GTP_POLL_TIME         10
#define GTP_ADDR_LENGTH       2
#define GTP_CONFIG_MIN_LENGTH 186
#define GTP_CONFIG_MAX_LENGTH 240
#define FAIL                  0
#define SUCCESS               1
#define SWITCH_OFF            0
#define SWITCH_ON             1

#ifdef CONFIG_MACH_CP8675
#define TW_GLOVE_SWITCH  1
#else
#define TW_GLOVE_SWITCH  0
#endif
/* begin to add esd protect switch and set it open by liushilong@yulong.com on 2014-11-7 18:00*/
#define GTP_ESD_PROTECT       1
/* end to add esd protect switch and set it open by liushilong@yulong.com on 2014-11-7 18:00*/
//Register define
#define GTP_READ_COOR_ADDR    0x814E
#define GTP_REG_SLEEP         0x8040
#define GTP_REG_SENSOR_ID     0x814A
#define GTP_REG_CONFIG_DATA   0x8047
#define GTP_REG_VERSION       0x8140
#define GTP_I2C_RETRY_3		3
#define GTP_I2C_RETRY_5		5
#define GTP_REG_REPORT_RATE   0x8056

#define RESOLUTION_LOC        3
#define TRIGGER_LOC           8

extern unsigned char GTP_DEBUG_ON;

#define GTP_DEBUG_ARRAY_ON    0
#define GTP_DEBUG_FUNC_ON     0


//Log define
#define GTP_INFO(fmt,arg...)           printk("<<-GTP-INFO->> "fmt"\n",##arg)
#define GTP_ERROR(fmt,arg...)          printk("<<-GTP-ERROR->> "fmt"\n",##arg)
#define GTP_DEBUG(fmt,arg...)          do{\
                                         if(GTP_DEBUG_ON)\
                                         printk("<<-GTP-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                       }while(0)
#define GTP_DEBUG_ARRAY(array, num)    do{\
                                         s32 i;\
                                         u8* a = array;\
                                         if(GTP_DEBUG_ARRAY_ON)\
                                         {\
                                            printk("<<-GTP-DEBUG-ARRAY->>\n");\
                                            for (i = 0; i < (num); i++)\
                                            {\
                                                printk("%02x   ", (a)[i]);\
                                                if ((i + 1 ) %10 == 0)\
                                                {\
                                                    printk("\n");\
                                                }\
                                            }\
                                            printk("\n");\
                                        }\
                                       }while(0)
#define GTP_DEBUG_FUNC()               do{\
                                         if(GTP_DEBUG_FUNC_ON)\
                                         printk("<<-GTP-FUNC->> Func:%s@Line:%d\n",__func__,__LINE__);\
                                       }while(0)
#define GTP_SWAP(x, y)                 do{\
                                         typeof(x) z = x;\
                                         x = y;\
                                         y = z;\
                                       }while (0)

//****************************PART4:UPDATE define*******************************
//Error no
#define ERROR_NO_FILE           2   //ENOENT
#define ERROR_FILE_READ         23  //ENFILE
#define ERROR_FILE_TYPE         21  //EISDIR
#define ERROR_GPIO_REQUEST      4   //EINTR
#define ERROR_I2C_TRANSFER      5   //EIO
#define ERROR_NO_RESPONSE       16  //EBUSY
#define ERROR_TIMEOUT           110 //ETIMEDOUT
//--------------------------------------------------------
/* GTP CM_HEAD RW flags */
#define GTP_RW_READ			0
#define GTP_RW_WRITE			1
#define GTP_RW_READ_IC_TYPE		2
#define GTP_RW_WRITE_IC_TYPE		3
#define GTP_RW_FILL_INFO		4
#define GTP_RW_NO_WRITE			5
#define GTP_RW_READ_ERROR		6
#define GTP_RW_DISABLE_IRQ		7
#define GTP_RW_READ_VERSION		8
#define GTP_RW_ENABLE_IRQ		9
#define GTP_RW_ENTER_UPDATE_MODE	11
#define GTP_RW_LEAVE_UPDATE_MODE	13
#define GTP_RW_UPDATE_FW		15
#define GTP_RW_CHECK_RAWDIFF_MODE	17

/* GTP need flag or interrupt */
#define GTP_NO_NEED			0
#define GTP_NEED_FLAG			1
#define GTP_NEED_INTERRUPT		2
//--------------------------------------------------------
//*****************************End of Part III********************************
int gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr,
					u8 *rxbuf, int len);
s32 gtp_send_cfg(struct i2c_client *client);
void gtp_reset_guitar(struct i2c_client *client, s32 ms);
void gtp_irq_disable(struct goodix_ts_data *ts);
void gtp_irq_enable(struct goodix_ts_data *ts);

s32 init_wr_node(struct i2c_client *client);
void uninit_wr_node(void);
u8 gup_init_update_proc(struct goodix_ts_data *ts);
s32 gup_enter_update_mode(struct i2c_client *client);
void gup_leave_update_mode(void);
s32 gup_update_proc(void *dir);
void gtp_esd_switch(struct i2c_client *, s32);
#endif /* _LINUX_GOODIX_TOUCH_H */
