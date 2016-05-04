/* drivers/input/touchscreen/gt813_827_828.c
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
 * Version: 1.9
 * Authors: andrew@goodix.com, meta@goodix.com
 * Release Date: 2013/04/25
 * Revision record:
 *      V1.0:
 *          first Release. By Andrew, 2012/08/31
 *      V1.2:
 *          modify gtp_reset_guitar,slot report,tracking_id & 0x0F. By Andrew, 2012/10/15
 *      V1.4:
 *          modify gt9xx_update.c. By Andrew, 2012/12/12
 *      V1.6:
 *          1. new heartbeat/esd_protect mechanism(add external watchdog)
 *          2. doze mode, sliding wakeup
 *          3. 3 more cfg_group(GT9 Sensor_ID: 0~5)
 *          3. config length verification
 *          4. names & comments
 *                  By Meta, 2013/03/11
 *      V1.8:
 *          1. pen/stylus identification
 *          2. read double check & fixed config support
 *          2. new esd & slide wakeup optimization
 *                  By Meta, 2013/06/08
 *		 V2.1.2:
 *			  1. optimized gtp_init_panel
 *			  2. auto update before init_panel
 *			  3. auto update 9158 flash check
 *						2013/12/31
 */

#include <linux/irq.h>
#include <linux/input/touchscreen_yl.h>

#ifdef CONFIG_OF
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#include "gt9xx.h"
#include "gt9xx_config.h"

#if GTP_ICS_SLOT_REPORT
    #include <linux/input/mt.h>
#endif

unsigned char GTP_DEBUG_ON=0;
//litao3 add 140102 for close TW_DEBUG
//#define YL_TW_DEBUG
//add end
static unsigned char tp_index;
s16 gtp_fw_version = 0;
u8 gtp_cfg_version = 0;
u8 tp_supported = 0;
#if GTP_COB
u8 cfg_version_bak = 0;
#endif

static const char *goodix_ts_name = "goodix";
static struct workqueue_struct *goodix_wq;
struct i2c_client * i2c_connect_client = NULL;
static u8 config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH]
                = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
struct tw_platform_data *gtpdata=NULL;

#if GTP_HAVE_TOUCH_KEY
	static const u16 touch_key_array[] = GTP_KEY_TAB;
	#define GTP_MAX_KEY_NUM	 (sizeof(touch_key_array)/sizeof(touch_key_array[0]))
#endif
static unsigned int GTP_PWR_EN=1;
#define GTP_SLEEP_PWR_EN 0
u8 glove_mode=0;
u8 glove_switch=0;
atomic_t gt_keypad_enable;

#ifdef CONFIG_TOUCHSCREEN_GT9XX_YL_COVER_WINDOW_SIZE

struct cover_window_info {
     unsigned int win_x_min;
     unsigned int win_x_max;

     unsigned int win_y_min;
     unsigned int win_y_max;

     atomic_t windows_switch;     //on/off
};

struct cover_window_info gt9xx_cover_window;

#endif

static s8 gtp_i2c_test(struct i2c_client *client);
void gtp_reset_guitar(struct i2c_client *client, s32 ms);

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data);
#endif
static int goodix_ts_disable(struct input_dev *in_dev);
static int goodix_ts_enable(struct input_dev *in_dev);

#if GTP_CREATE_WR_NODE
extern s32 init_wr_node(struct i2c_client*);
extern void uninit_wr_node(void);
#endif

extern u8 gup_init_update_proc(struct goodix_ts_data *);

#ifdef HALL_SWITCH_WAKEUP
void tw_hall_wakeup(int in_hall);
#endif
#if GTP_ESD_PROTECT
static struct delayed_work gtp_esd_check_work;
static struct workqueue_struct * gtp_esd_check_workqueue = NULL;
static void gtp_esd_check_func(struct work_struct *);
/* begin to transplant TW GTP_ESD_PROTECT code from 8675_C00, add declaration of watchdog and esd switch function by liushilong@yulong.com on 2014-11-6 16:57*/
static s32 gtp_init_ext_watchdog(struct i2c_client *client);
void gtp_esd_switch(struct i2c_client *, s32);
/* end to transplant TW GTP_ESD_PROTECT code from 8675_C00, add declaration of watchdog and esd switch function by liushilong@yulong.com on 2014-11-6 16:57*/
#endif
#ifdef YL_TW_DEBUG
char *key_name[3] ={
	"Menu",
	"Home",
	"Back",
};
#endif

#pragma pack(1)
typedef struct
{
    u8  hw_info[4];          //hardware info//
    u8  pid[8];              //product id   //
    u16 vid;                 //version id   //
}st_fw_head;
#pragma pack()

//extern u8 gup_init_update_proc(void *dir);
extern u8 gup_check_update_file(struct i2c_client *client, st_fw_head* fw_head, u8* path);
extern u8 gup_get_ic_fw_msg(struct i2c_client *client);

#if GTP_SLIDE_WAKEUP
typedef enum
{
    DOZE_DISABLED = 0,
    DOZE_ENABLED = 1,
    DOZE_WAKEUP = 2,
}DOZE_T;
static DOZE_T doze_status = DOZE_DISABLED;
static s8 gtp_enter_doze(struct goodix_ts_data *ts);

enum support_gesture_e {
	TW_SUPPORT_NONE_SLIDE_WAKEUP = 0x0,
	TW_SUPPORT_UP_SLIDE_WAKEUP = 0x1,
	TW_SUPPORT_DOWN_SLIDE_WAKEUP = 0x2,
	TW_SUPPORT_LEFT_SLIDE_WAKEUP = 0x4,
	TW_SUPPORT_RIGHT_SLIDE_WAKEUP = 0x8,
	TW_SUPPORT_E_SLIDE_WAKEUP = 0x10,
	TW_SUPPORT_O_SLIDE_WAKEUP = 0x20,
	TW_SUPPORT_W_SLIDE_WAKEUP = 0x40,
	TW_SUPPORT_C_SLIDE_WAKEUP = 0x80,
	TW_SUPPORT_M_SLIDE_WAKEUP = 0x100,
	TW_SUPPORT_DOUBLE_CLICK_WAKEUP = 0x200,

	TW_SUPPORT_GESTURE_IN_ALL = (TW_SUPPORT_UP_SLIDE_WAKEUP | TW_SUPPORT_DOWN_SLIDE_WAKEUP |
								TW_SUPPORT_LEFT_SLIDE_WAKEUP | TW_SUPPORT_RIGHT_SLIDE_WAKEUP | TW_SUPPORT_E_SLIDE_WAKEUP |
								TW_SUPPORT_O_SLIDE_WAKEUP |TW_SUPPORT_W_SLIDE_WAKEUP |TW_SUPPORT_C_SLIDE_WAKEUP |
								TW_SUPPORT_M_SLIDE_WAKEUP | TW_SUPPORT_DOUBLE_CLICK_WAKEUP)
};

u32 support_gesture = TW_SUPPORT_DOUBLE_CLICK_WAKEUP |
		TW_SUPPORT_DOWN_SLIDE_WAKEUP |
		TW_SUPPORT_LEFT_SLIDE_WAKEUP |
		TW_SUPPORT_RIGHT_SLIDE_WAKEUP |
		TW_SUPPORT_O_SLIDE_WAKEUP |
		TW_SUPPORT_C_SLIDE_WAKEUP;
char wakeup_slide[32];

#endif


/*******************************************************
Function:
	Read data from the i2c slave device.

Input:
	client:	i2c device.
	buf[0]:operate address.
	buf[1]~buf[len]:read data buffer.
	len:operate length.

Output:
	numbers of i2c_msgs to transfer
*********************************************************/
s32 gtp_i2c_read(struct i2c_client *client, u8 *buf, s32 len)
{
    struct i2c_msg msgs[2];
    s32 ret=-1;
    s32 retries = 0;
    struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

    GTP_DEBUG_FUNC();

    msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = client->addr;
    msgs[0].len   = GTP_ADDR_LENGTH;
    msgs[0].buf   = &buf[0];

    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = client->addr;
    msgs[1].len   = len - GTP_ADDR_LENGTH;
    msgs[1].buf   = &buf[GTP_ADDR_LENGTH];

    while(retries < 5)
    {
        mutex_lock(&ts->reset_mutex);
        ret = i2c_transfer(client->adapter, msgs, 2);
		mutex_unlock(&ts->reset_mutex);

	if(ret == 2)
		break;
        retries++;
#if GTP_SLIDE_WAKEUP
             if (DOZE_ENABLED == doze_status)
                {
                   mdelay(10);  //wait for i2c resume
                }
#endif
    }
    if(retries >= 5)
    {
    #if GTP_SLIDE_WAKEUP
        // reset chip would quit doze mode
        if (DOZE_ENABLED == doze_status)
        {
            return ret;
        }
    #endif
        GTP_DEBUG("I2C retry timeout, reset chip.");
		//qysh qu xiao zhushi
        //gtp_reset_guitar(client, 10);
    }
    return ret;
}

/*******************************************************
Function:
	write data to the i2c slave device.

Input:
	client:	i2c device.
	buf[0]:operate address.
	buf[1]~buf[len]:write data buffer.
	len:operate length.

Output:
	numbers of i2c_msgs to transfer.
*********************************************************/
s32 gtp_i2c_write(struct i2c_client *client,u8 *buf,s32 len)
{
    struct i2c_msg msg;
    s32 ret=-1;
    s32 retries = 0;
    struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

    GTP_DEBUG_FUNC();

    msg.flags = !I2C_M_RD;
    msg.addr  = client->addr;
    msg.len   = len;
    msg.buf   = buf;

    while(retries < 5)
    {
	mutex_lock(&ts->reset_mutex);
        ret = i2c_transfer(client->adapter, &msg, 1);
        mutex_unlock(&ts->reset_mutex);

        if (ret == 1)
		break;
        retries++;
    }
    if(retries >= 5)
    {
    #if GTP_SLIDE_WAKEUP
        if (DOZE_ENABLED == doze_status)
        {
            return ret;
        }
    #endif
        GTP_DEBUG("I2C retry timeout, reset chip.");
        //gtp_reset_guitar(client, 10);
    }
    return ret;
}
#ifdef HALL_SWITCH_WAKEUP
void tw_hall_wakeup(int in_hall)
{
    struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
    if(in_hall)     //hall detect leave-state
    {
		if(ts->gtp_is_suspend)
	        {
	        // report powerkey event for PowerManagerService to wakeup or sleep system
		    input_report_key(ts->input_dev, KEY_POWER, 1);
		    input_sync(ts->input_dev);
		    input_report_key(ts->input_dev, KEY_POWER, 0);
		    input_sync(ts->input_dev);
		}
    }
    else    // hall detect near-state
    {
		if(0 ==ts->gtp_is_suspend)
		{
		    input_report_key(ts->input_dev, KEY_POWER, 1);
		    input_sync(ts->input_dev);
		    input_report_key(ts->input_dev, KEY_POWER, 0);
		    input_sync(ts->input_dev);
		}
    }
}
EXPORT_SYMBOL_GPL(tw_hall_wakeup);

#endif
#if GTP_SLIDE_WAKEUP
void gtp_enable_irq_wake(struct goodix_ts_data *ts, u8 enable)
{
	static u8 irq_wake_status = 0;

	if (irq_wake_status != enable) {
		if (enable) {
			enable_irq_wake(ts->client->irq);
		} else {
			disable_irq_wake(ts->client->irq);
		}
		irq_wake_status = enable;
	}
}
#endif
/*******************************************************
Function:
    i2c read twice, compare the results
Input:
    client:  i2c device
    addr:    operate address
    rxbuf:   read data to store, if compare successful
    len:     bytes to read
Output:
    FAIL:    read failed
    SUCCESS: read successful
*********************************************************/
s32 gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
    u8 buf[16] = {0};
    u8 confirm_buf[16] = {0};
    u8 retry = 0;

    while (retry++ < 3)
    {
        memset(buf, 0xAA, 16);
        buf[0] = (u8)(addr >> 8);
        buf[1] = (u8)(addr & 0xFF);
        gtp_i2c_read(client, buf, len + 2);

        memset(confirm_buf, 0xAB, 16);
        confirm_buf[0] = (u8)(addr >> 8);
        confirm_buf[1] = (u8)(addr & 0xFF);
        gtp_i2c_read(client, confirm_buf, len + 2);

        if (!memcmp(buf, confirm_buf, len+2))
        {
            break;
        }
    }
    if (retry < 3)
    {
        memcpy(rxbuf, confirm_buf+2, len);
        return SUCCESS;
    }
    else
    {
        GTP_ERROR("i2c read 0x%04X, %d bytes, double check failed!", addr, len);
        return FAIL;
    }
}

/*******************************************************
Function:
	Send config Function.

Input:
	client:	i2c client.

Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
s32 gtp_send_cfg(struct i2c_client *client)
{
    s32 ret = 0;


#if GTP_DRIVER_SEND_CFG
    s32 retry = 0;

    for (retry = 0; retry < 5; retry++)
    {
        ret = gtp_i2c_write(client, config , GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
        if (ret > 0)
        {
            break;
        }
    }

	GTP_DEBUG_FUNC();

#endif

    return ret;
}

/*******************************************************
Function:
	Enable IRQ Function.

Input:
	ts:	i2c client private struct.

Output:
	None.
*******************************************************/
void gtp_irq_disable(struct goodix_ts_data *ts)
{
    unsigned long irqflags;

    GTP_DEBUG_FUNC();

    spin_lock_irqsave(&ts->irq_lock, irqflags);
    if (!ts->irq_is_disable)
    {
        ts->irq_is_disable = 1;
        disable_irq_nosync(ts->client->irq);
    }
    spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
	Disable IRQ Function.

Input:
	ts:	i2c client private struct.

Output:
	None.
*******************************************************/
void gtp_irq_enable(struct goodix_ts_data *ts)
{
    unsigned long irqflags = 0;

    GTP_DEBUG_FUNC();

    spin_lock_irqsave(&ts->irq_lock, irqflags);
    if (ts->irq_is_disable)
    {
        enable_irq(ts->client->irq);
        ts->irq_is_disable = 0;
    }
    spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
	Touch down report function.

Input:
	ts:private data.
	id:tracking id.
	x:input x.
	y:input y.
	w:input weight.

Output:
	None.
*******************************************************/
static void gtp_touch_down(struct goodix_ts_data* ts,s32 id,s32 x,s32 y,s32 w)
{
#if GTP_CHANGE_X2Y
    GTP_SWAP(x, y);
#endif

#ifdef CONFIG_TOUCHSCREEN_GT9XX_YL_COVER_WINDOW_SIZE
if(atomic_read(&gt9xx_cover_window.windows_switch) && !(x > gt9xx_cover_window.win_x_min
                                                   && x < gt9xx_cover_window.win_x_max
                                                   && y > gt9xx_cover_window.win_y_min
                                                   && y < gt9xx_cover_window.win_y_max));
else
#endif
{
#if GTP_ICS_SLOT_REPORT
    input_mt_slot(ts->input_dev, id);
    input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
    input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
    input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
#else
	input_report_key(ts->input_dev, BTN_TOUCH, 1);//for fastmmi
    input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
    input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
    input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
    input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
    input_mt_sync(ts->input_dev);
#endif

    GTP_DEBUG("ID:%d, X:%d, Y:%d, W:%d", id, x, y, w);
}
}

/*******************************************************
Function:
	Touch up report function.

Input:
	ts:private data.

Output:
	None.
*******************************************************/
static void gtp_touch_up(struct goodix_ts_data* ts, s32 id)
{
#if GTP_ICS_SLOT_REPORT
    input_mt_slot(ts->input_dev, id);
    input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
    GTP_DEBUG("Touch id[%2d] release!", id);
#else
#if 0	//for Android4.0, use MT-A protocol
    input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
    input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_X, 0);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, 0);
#endif
	input_report_key(ts->input_dev, BTN_TOUCH, 0);//for fastmmi
    input_mt_sync(ts->input_dev);
#endif
}
extern struct device *touchscreen_get_dev(void);

/*******************************************************
Function:
	Goodix touchscreen work function.

Input:
	work:	work_struct of goodix_wq.

Output:
	None.
*******************************************************/
static void goodix_ts_work_func(struct work_struct *work)
{
    u8  end_cmd[3] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF, 0};
    u8  point_data[2 + 1 + 8 * GTP_MAX_TOUCH + 1]={GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF};
    u8  touch_num = 0;
    u8  finger = 0;
    static u16 pre_touch = 0;
    static u8 pre_key = 0;
    u8  key_value = 0;
    u8* coor_data = NULL;
    s32 input_x = 0;
    s32 input_y = 0;
    s32 input_w = 0;
    s32 id = 0;
    s32 i  = 0;
    s32 ret = -1;
    struct goodix_ts_data *ts = NULL;
#ifdef YL_TW_DEBUG
	static u8 pre_touch_status = 0;
	u8 cur_touch_status = 0;
#endif

#if GTP_SLIDE_WAKEUP
 //added by yewenliang for test
    char *none_wakeup[2] = {"GESTURE=NONE",NULL};
    char *left_wakeup[2] = {"GESTURE=LEFT",NULL};
    char *right_wakeup[2] = {"GESTURE=RIGHT",NULL};
    char *up_wakeup[2] = {"GESTURE=UP",NULL};
    char *down_wakeup[2] = {"GESTURE=DOWN",NULL};
    char *doubleClick_wakeup[2] = {"GESTURE=DOUBLE_CLICK",NULL};
    char *c_wakeup[2] = {"GESTURE=C",NULL};
    char *e_wakeup[2] = {"GESTURE=E",NULL};
    char *m_wakeup[2] = {"GESTURE=M",NULL};
    char *o_wakeup[2] = {"GESTURE=O",NULL};
    char *w_wakeup[2] = {"GESTURE=W",NULL};
    char **envp;

    u8 doze_buf[3] = {0x81, 0x4B};
    uint16_t gesture_key = KEY_UNKNOWN;
#endif
    GTP_DEBUG_FUNC();

    ts = container_of(work, struct goodix_ts_data, work);
    if (ts->enter_update)
    {
        return;
    }
#if GTP_SLIDE_WAKEUP
/* begin to add mutex_lock for gesture awake for avoiding this mode be occupied by liushilong@yulong.com on 2014-11-6 17:07*/
    mutex_lock(&ts->doze_mutex);
/* end to add mutex_lock for gesture awake for avoiding this mode be occupied by liushilong@yulong.com on 2014-11-6 17:07*/
    if (DOZE_ENABLED == doze_status)
    {
        ret = gtp_i2c_read(i2c_connect_client, doze_buf, 3);
        GTP_DEBUG("0x814B = 0x%02X", doze_buf[2]);
        envp = none_wakeup;	//added by yewenliang for test

        if (ret > 0)
        {
            if ((doze_buf[2] == 0xAA) && (support_gesture & TW_SUPPORT_RIGHT_SLIDE_WAKEUP))
            {
                GTP_INFO("Slide(0xAA) To Light up the screen!");
                doze_status = DOZE_WAKEUP;
                sprintf(wakeup_slide,"right");
                envp = right_wakeup;	//added by yewenliang for test
                gesture_key = KEY_GESTURE_SLIDE_RIGHT;
            }
            else if ((doze_buf[2] == 0xBB) && (support_gesture & TW_SUPPORT_LEFT_SLIDE_WAKEUP))
            {
                GTP_INFO("Slide(0xBB) To Light up the screen!");
                doze_status = DOZE_WAKEUP;
                sprintf(wakeup_slide,"left");
                envp = left_wakeup;	//added by yewenliang for test
                gesture_key = KEY_GESTURE_SLIDE_LEFT;
            }
            else if ((0xC0 == (doze_buf[2] & 0xC0)) && (support_gesture & TW_SUPPORT_DOUBLE_CLICK_WAKEUP))
            {
                GTP_INFO("double click to light up the screen!");
                doze_status = DOZE_WAKEUP;
                sprintf(wakeup_slide,"double_click");
                envp = doubleClick_wakeup;	//added by yewenliang for test
                gesture_key = KEY_WAKEUP;
            }
            else if ((doze_buf[2] == 0xBA) && (support_gesture & TW_SUPPORT_UP_SLIDE_WAKEUP))
            {
                GTP_INFO("Slide(0xBA) To Light up the screen!");
                doze_status = DOZE_WAKEUP;
                sprintf(wakeup_slide,"up");
                envp = up_wakeup;	//added by yewenliang for test
            }
            else if ((doze_buf[2] == 0xAB) && (support_gesture & TW_SUPPORT_DOWN_SLIDE_WAKEUP))
            {
                GTP_INFO("Slide(0xAB) To Light up the screen!");
                doze_status = DOZE_WAKEUP;
                sprintf(wakeup_slide,"down");
                envp = down_wakeup;	//added by yewenliang for test
                gesture_key = KEY_GESTURE_SLIDE_DOWN;
            }
            else if ((doze_buf[2] == 0x63) && (support_gesture & TW_SUPPORT_C_SLIDE_WAKEUP))
            {
                GTP_INFO("Slide(0x63) To Light up the screen!");
                doze_status = DOZE_WAKEUP;
                sprintf(wakeup_slide,"c");
                envp = c_wakeup;	//added by yewenliang for test
                gesture_key = KEY_GESTURE_SLIDE_C;
            }
            else if ((doze_buf[2] == 0x65) && (support_gesture & TW_SUPPORT_E_SLIDE_WAKEUP))
            {
                GTP_INFO("Slide(0x65) To Light up the screen!");
                doze_status = DOZE_WAKEUP;
                sprintf(wakeup_slide,"e");
                envp = e_wakeup;	//added by yewenliang for test
            }
            else if ((doze_buf[2] == 0x6D) && (support_gesture & TW_SUPPORT_M_SLIDE_WAKEUP))
            {
                GTP_INFO("Slide(0x6D) To Light up the screen!");
                doze_status = DOZE_WAKEUP;
                sprintf(wakeup_slide,"m");
                envp = m_wakeup;	//added by yewenliang for test
            }
            else if ((doze_buf[2] == 0x6F) && (support_gesture & TW_SUPPORT_O_SLIDE_WAKEUP))
            {
                GTP_INFO("Slide(0x6F) To Light up the screen!");
                doze_status = DOZE_WAKEUP;
                sprintf(wakeup_slide,"o");
                envp = o_wakeup;	//added by yewenliang for test
                gesture_key = KEY_GESTURE_SLIDE_O;
            }
            else if ((doze_buf[2] == 0x77) && (support_gesture & TW_SUPPORT_W_SLIDE_WAKEUP))
            {
                GTP_INFO("Slide(0x77) To Light up the screen!");
                doze_status = DOZE_WAKEUP;
                sprintf(wakeup_slide,"w");
                envp = w_wakeup;	//added by yewenliang for test
            }
            else
            {
			    //added by yewenliang
                // clear 0x814B
                doze_buf[2] = 0x00;
                gtp_i2c_write(i2c_connect_client, doze_buf, 3);
                gtp_enter_doze(ts);
            }
        }

        if (doze_status == DOZE_WAKEUP && gesture_key != KEY_UNKNOWN) {
             input_report_key(ts->input_dev, gesture_key, 1);
             input_report_key(ts->input_dev, gesture_key, 0);
             input_sync(ts->input_dev);

             // clear 0x814B
             doze_buf[2] = 0x00;
             gtp_i2c_write(i2c_connect_client, doze_buf, 3);
             gtp_enter_doze(ts);
        }

        if (ts->use_irq)
        {
            gtp_irq_enable(ts);
        }

/* begin to add mutex_lock for gesture awake elease this mode by liushilong@yulong.com on 2014-11-6 17:07*/
		mutex_unlock(&ts->doze_mutex);
/* end to add mutex_lock for gesture awake elease this mode by liushilong@yulong.com on 2014-11-6 17:07*/

        return;
    }

/* begin to add mutex_unlock for gesture awake release this mode by liushilong@yulong.com on 2014-11-6 17:07*/
    mutex_unlock(&ts->doze_mutex);
/* end to add mutex_unlock for gesture awake release this mode by liushilong@yulong.com on 2014-11-6 17:07*/
#endif

    ret = gtp_i2c_read(ts->client, point_data, 12);
    if (ret < 0)
    {
        GTP_ERROR("I2C transfer error. errno:%d\n ", ret);
        goto exit_work_func;
    }

    finger = point_data[GTP_ADDR_LENGTH];
    if((finger & 0x80) == 0)
    {
        goto exit_work_func;
    }

    touch_num = finger & 0x0f;
    if (touch_num > GTP_MAX_TOUCH)
    {
        goto exit_work_func;
    }

    if (touch_num > 1)
    {
        u8 buf[8 * GTP_MAX_TOUCH] = {(GTP_READ_COOR_ADDR + 10) >> 8, (GTP_READ_COOR_ADDR + 10) & 0xff};

        ret = gtp_i2c_read(ts->client, buf, 2 + 8 * (touch_num - 1));
        memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));
    }

#if GTP_HAVE_TOUCH_KEY
    key_value = point_data[3 + 8 * touch_num];
//	input_report_key(ts->input_dev, BTN_TOUCH, (touch_num || key_value));

    if (atomic_read(&gt_keypad_enable) && (key_value || pre_key))
    {
        for (i = 0; i < GTP_MAX_KEY_NUM; i++)
        {
            input_report_key(ts->input_dev, touch_key_array[i], key_value & (0x01<<i));
			GTP_DEBUG("input_report_key:%d, touch_key_arr:%d\n", i, touch_key_array[i]);
				#ifdef YL_TW_DEBUG
					if (key_value & (0x01<<i))
					GTP_INFO("%s KEY press.", key_name[i]);
				#endif
        }
        touch_num = 0;
        pre_touch = 0;
    }
#endif
    pre_key = key_value;

    GTP_DEBUG("pre_touch:%02x, finger:%02x.", pre_touch, finger);

#if GTP_ICS_SLOT_REPORT
    if (pre_touch || touch_num)
    {
        s32 pos = 0;
        u16 touch_index = 0;

        coor_data = &point_data[3];
        if(touch_num)
        {
            id = coor_data[pos] & 0x0F;
            touch_index |= (0x01<<id);
        }

        for (i = 0; i < GTP_MAX_TOUCH; i++)
        {
            if (touch_index & (0x01<<i))
            {
                input_x  = coor_data[pos + 1] | coor_data[pos + 2] << 8;
                input_y  = coor_data[pos + 3] | coor_data[pos + 4] << 8;
                input_w  = coor_data[pos + 5] | coor_data[pos + 6] << 8;

                gtp_touch_down(ts, id, input_x, input_y, input_w);
                pre_touch |= 0x01 << i;

                pos += 8;
                id = coor_data[pos] & 0x0F;
                touch_index |= (0x01<<id);
            }
            else// if (pre_touch & (0x01 << i))
            {
                gtp_touch_up(ts, i);
                pre_touch &= ~(0x01 << i);
            }
        }
    }

#else
    if (touch_num )
    {
        for (i = 0; i < touch_num; i++)
        {
            coor_data = &point_data[i * 8 + 3];

            id = coor_data[0] & 0x0F;
            input_x  = coor_data[1] | coor_data[2] << 8;
            input_y  = coor_data[3] | coor_data[4] << 8;
            input_w  = coor_data[5] | coor_data[6] << 8;

            gtp_touch_down(ts, id, input_x, input_y, input_w);
#ifdef YL_TW_DEBUG
			cur_touch_status |= (0x01<<id);
			if (pre_touch < touch_num) {
				if (pre_touch_status ^ (0x01<<id)) {
		    			GTP_INFO("Point[%d] Touch Down! x=%d, y=%d.", id, input_x, input_y);
				}
			}
#endif
        }
#ifdef YL_TW_DEBUG
    	if(pre_touch > touch_num) {
    		for (i = 0; i < GTP_MAX_TOUCH; i++) {
    			if ((pre_touch_status^cur_touch_status)&(0x01<<i)) {
		    		GTP_INFO("Point[%d] Touch Up!", i);
		    	}
    		}
    	}
#endif
    }
    else if (pre_touch)
    {
    	#ifdef YL_TW_DEBUG
        GTP_INFO("All Touch Release!");
#else
        GTP_DEBUG("Touch Release!");
        #endif
        gtp_touch_up(ts, 0);
    }
#ifdef YL_TW_DEBUG
    pre_touch_status = cur_touch_status;
#endif
#if 0	//for Android4.0, use MT-A protocol
    input_report_key(ts->input_dev, BTN_TOUCH, (touch_num || key_value));
#endif
    pre_touch = touch_num;
#endif

    input_sync(ts->input_dev);

exit_work_func:
    if(!ts->gtp_rawdiff_mode)
    {
        ret = gtp_i2c_write(ts->client, end_cmd, 3);
        if (ret < 0)
        {
            GTP_INFO("I2C write end_cmd  error!");
        }
    }

    if (ts->use_irq)
    {
        gtp_irq_enable(ts);
    }
}

/*******************************************************
Function:
	Timer interrupt service routine.

Input:
	timer:	timer struct pointer.

Output:
	Timer work mode. HRTIMER_NORESTART---not restart mode
*******************************************************/
static enum hrtimer_restart goodix_ts_timer_handler(struct hrtimer *timer)
{
    struct goodix_ts_data *ts = container_of(timer, struct goodix_ts_data, timer);

    GTP_DEBUG_FUNC();

    queue_work(goodix_wq, &ts->work);
    hrtimer_start(&ts->timer, ktime_set(0, (GTP_POLL_TIME+6)*1000000), HRTIMER_MODE_REL);
    return HRTIMER_NORESTART;
}

/*******************************************************
Function:
	External interrupt service routine.

Input:
	irq:	interrupt number.
	dev_id: private data pointer.

Output:
	irq execute status.
*******************************************************/
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
    struct goodix_ts_data *ts = dev_id;

    GTP_DEBUG_FUNC();

    gtp_irq_disable(ts);
    queue_work(goodix_wq, &ts->work);

    return IRQ_HANDLED;
}


/*******************************************************
Function:
	Reset chip Function.

Input:
	ms:reset time.

Output:
	None.
*******************************************************/
#if 1
void gtp_reset_guitar(struct i2c_client *client, s32 ms)
{
	struct goodix_ts_data * ts = i2c_get_clientdata(client);

    	GTP_DEBUG_FUNC();

	if(ts->pdata->reset)
		ts->pdata->reset(ms);

}
#endif
#if GTP_SLIDE_WAKEUP
/*******************************************************
Function:
    Enter doze mode for sliding wakeup.
Input:
    ts: goodix tp private data
Output:
    1: succeed, otherwise failed
*******************************************************/
static s8 gtp_enter_doze(struct goodix_ts_data *ts)
{
    s8 ret = -1;
    s8 retry = 0;
    u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 8};

    GTP_DEBUG_FUNC();

#if GTP_DBL_CLK_WAKEUP
    i2c_control_buf[2] = 0x09;
#endif

    gtp_irq_disable(ts);
    sprintf(wakeup_slide,"none");
    GTP_DEBUG("entering doze mode...");
    while(retry++ < 5)
    {

        i2c_control_buf[0] = 0x80;
        i2c_control_buf[1] = 0x46;
        ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
        if (ret < 0)
        {
            GTP_DEBUG("failed to set doze flag into 0x8046, %d", retry);
            continue;
        }

        i2c_control_buf[0] = 0x80;
        i2c_control_buf[1] = 0x40;
        ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
        if (ret > 0)
        {
            doze_status = DOZE_ENABLED;
			GTP_INFO("[GTP]:Enter in doze mode successed!");
            gtp_enable_irq_wake(ts, 1);
            gtp_irq_enable(ts);
            return ret;
        }
        msleep(10);
    }
    GTP_ERROR("GTP send doze cmd failed.");
    gtp_irq_enable(ts);
    return ret;
}
#endif
/*******************************************************
Function:
	Eter sleep function.

Input:
	ts:private data.

Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
static s8 gtp_enter_sleep(struct goodix_ts_data * ts)
{
    s8 ret = 0;
    u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 5};

#if GTP_POWER_CTRL_SLEEP

    gtp_i2c_write(ts->client, i2c_control_buf, 3);

    if(ts->pdata->power)
        ts->pdata->power(0);

#else
    s8 retry = 0;
    //irq output 0
    gpio_direction_output(ts->pdata->gpio_irq, 0);
	//gpio_direction_output(ts->pdata->gpio_reset, 0);//qysh for power sonsume11.02
    msleep(5);
    while(retry++ < 5)
    {
        ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
        if (ret > 0)
        {
            GTP_DEBUG("GTP enter sleep!");
	     return ret;
        }
        msleep(10);
    }
    GTP_ERROR("GTP send sleep cmd failed.");

#endif
    return ret;
}

/*******************************************************
Function:
	Wakeup from sleep mode Function.

Input:
	ts:	private data.

Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
static s8 gtp_wakeup_sleep(struct goodix_ts_data * ts)
{
    u8 retry = 0;
    s8 ret = -1;

    GTP_DEBUG_FUNC();

#if GTP_POWER_CTRL_SLEEP
    if(ts->pdata->power)
        ts->pdata->power(1);
#endif

    while(retry++ < 5)
    {
        gtp_reset_guitar(ts->client, 20);

        ret = gtp_send_cfg(ts->client);
        if (ret < 0)
        {
            GTP_INFO("Wakeup sleep send config failed!");
            continue;
        }
        GTP_INFO("GTP wakeup sleep");
        return 1;
    }

    GTP_ERROR("GTP wakeup sleep failed.");
    return ret;
}

/*******************************************************
Function:
	Early suspend function.

Input:
	h:early_suspend struct.

Output:
	None.
*******************************************************/
s32 gtp_init_panel(struct goodix_ts_data *ts)
{
    s32 ret = -1;
    s32 i;
    u8 check_sum = 0;
    u8 opr_buf[16];
    u8 sensor_id = 0;

    ret = gtp_i2c_read_dbl_check(ts->client, 0x41E4, opr_buf, 1);
    if (SUCCESS == ret)
    {
        if (opr_buf[0] != 0xBE)
        {
            ts->fw_error = 1;
            printk("Firmware error, no config sent!");
            return -1;
        }
    }

// module id
    ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_SENSOR_ID, &sensor_id, 1);
    if (SUCCESS == ret)
    {
        if (sensor_id >= 0x06)
        {
            GTP_ERROR("Invalid sensor_id(0x%02X), No Config Sent!", sensor_id);
            return -1;
        }
    }
    else
    {
        GTP_ERROR("Failed to get sensor_id, No config sent!");
        return -1;
    }

    tp_supported = 0;
    for(i=0; i<ARRAY_SIZE(yl_cfg); i++)
    {
		if(sensor_id == yl_cfg[i].tp_id)
		{
			tp_index = i;
			tp_supported = 1;
			ts->gtp_cfg_len = yl_cfg[tp_index].firmware_size;
			break;
		}
    }
	if(tp_supported != 1)
	{
		GTP_ERROR("Invalid sensor_id(0x%02X), No Config Sent!", sensor_id);
		return -1;
	}

    if (ts->gtp_cfg_len < GTP_CONFIG_MIN_LENGTH)
    {
        GTP_ERROR("Sensor_ID(%d) matches with NULL or INVALID CONFIG GROUP! NO Config Sent! You need to check you header file CFG_GROUP section!", sensor_id);
        return -1;
    }

//config
    ts->gtp_cfg_len = yl_cfg[tp_index].firmware_size;
    memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
    memcpy(&config[GTP_ADDR_LENGTH], yl_cfg[tp_index].firmware, ts->gtp_cfg_len);

#if GTP_COB
    ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_CONFIG_DATA, &opr_buf[0], 1);
    if (ret == SUCCESS)
    {
		if ( (opr_buf[0] < 90) && (opr_buf[0] != config[GTP_ADDR_LENGTH]) )   // change the condition by that not equal
        {
           cfg_version_bak = config[GTP_ADDR_LENGTH];
           	config[GTP_ADDR_LENGTH] = 0x00;
        }
    }
#endif

#if GTP_CUSTOM_CFG
//max size
    config[RESOLUTION_LOC]     = ts->pdata->screen_x&0xff;
    config[RESOLUTION_LOC + 1] = (ts->pdata->screen_x>>8)&0xff;
    config[RESOLUTION_LOC + 2] = ts->pdata->screen_y&0xff;
    config[RESOLUTION_LOC + 3] = (ts->pdata->screen_y>>8)&0xff;

#endif  //endif GTP_CUSTOM_CFG

//trigger type
    if (ts->pdata->irqflag == IRQF_TRIGGER_RISING)  //RISING
    {
        config[TRIGGER_LOC] &= 0xfc;
    }
    else if (ts->pdata->irqflag == IRQF_TRIGGER_FALLING)  //FALLING
    {
        config[TRIGGER_LOC] &= 0xfc;
        config[TRIGGER_LOC] |= 0x01;
    }

//chechsum
    check_sum = 0;
    for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++)
    {
        check_sum += config[i];
    }
    config[ts->gtp_cfg_len] = (~check_sum) + 1;

    GTP_DEBUG_FUNC();
    ts->abs_x_max = (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
    ts->abs_y_max = (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC + 2];
    ts->int_trigger_type = (config[TRIGGER_LOC]) & 0x03;
    if ((!ts->abs_x_max)||(!ts->abs_y_max))
    {
        GTP_ERROR("GTP resolution & max_touch_num invalid, use default value!");
        ts->abs_x_max = ts->pdata->screen_x;
        ts->abs_y_max = ts->pdata->screen_y;
    }

//send config to gtxxx
    ret = gtp_send_cfg(ts->client);
    if (ret < 0)
    {
        GTP_ERROR("Send config error.");
    }
    msleep(10);

#if GTP_COB
	if(cfg_version_bak)
	{
    	config[GTP_ADDR_LENGTH] = cfg_version_bak;
		cfg_version_bak = 0;

		//chechsum
	    check_sum = 0;
	    for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++)
	    {
	        check_sum += config[i];
	    }
	    config[ts->gtp_cfg_len] = (~check_sum) + 1;

	    msleep(10);
		ret = gtp_send_cfg(ts->client);
	    msleep(500);   // should change MSEC to 500 for update completely
	}
#endif

    return 0;
}

/*******************************************************
Function:
	glove write config

Input:
	ts:	i2c client private struct.

Output:
	Executive outcomes.0---succeed.
*******************************************************/
static s32 gtp_write_config(struct goodix_ts_data *ts)
{
    s32 ret = -1;
    s32 i;
    u8 check_sum = 0;
	u8 glove_gtp_cfg_len=0;

//config
	if(glove_mode==1)
		{
	glove_gtp_cfg_len = yl_cfg_glove[tp_index].firmware_size;
    memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
    memcpy(&config[GTP_ADDR_LENGTH], yl_cfg_glove[tp_index].firmware, glove_gtp_cfg_len);
		}
	else if(glove_mode==0)
		{
	glove_gtp_cfg_len = yl_cfg[tp_index].firmware_size;
    memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
    memcpy(&config[GTP_ADDR_LENGTH], yl_cfg[tp_index].firmware, glove_gtp_cfg_len);
		}
#if GTP_CUSTOM_CFG
//max size
    config[RESOLUTION_LOC]     = ts->pdata->screen_x&0xff;
    config[RESOLUTION_LOC + 1] = (ts->pdata->screen_x>>8)&0xff;
    config[RESOLUTION_LOC + 2] = ts->pdata->screen_y&0xff;
    config[RESOLUTION_LOC + 3] = (ts->pdata->screen_y>>8)&0xff;

#endif  //endif GTP_CUSTOM_CFG

//trigger type
    if (ts->pdata->irqflag == IRQF_TRIGGER_RISING)  //RISING
    {
        config[TRIGGER_LOC] &= 0xfc;
    }
    else if (ts->pdata->irqflag == IRQF_TRIGGER_FALLING)  //FALLING
    {
        config[TRIGGER_LOC] &= 0xfc;
        config[TRIGGER_LOC] |= 0x01;
    }

//chechsum
    check_sum = 0;
    for (i = GTP_ADDR_LENGTH; i < glove_gtp_cfg_len; i++)
    {
        check_sum += config[i];
    }
    config[glove_gtp_cfg_len] = (~check_sum) + 1;

    GTP_DEBUG_FUNC();
//
    ts->abs_x_max = (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
    ts->abs_y_max = (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC + 2];
    ts->int_trigger_type = (config[TRIGGER_LOC]) & 0x03;
    if ((!ts->abs_x_max)||(!ts->abs_y_max))
    {
        GTP_ERROR("GTP resolution & max_touch_num invalid, use default value!");
        ts->abs_x_max = ts->pdata->screen_x;
        ts->abs_y_max = ts->pdata->screen_y;
    }

//send config to gtxxx
    ret = gtp_send_cfg(ts->client);
    if (ret < 0)
    {
        GTP_ERROR("Send config error.");
    }

    GTP_DEBUG("X_MAX = %d,Y_MAX = %d,TRIGGER = 0x%02x",
             ts->abs_x_max,ts->abs_y_max,ts->int_trigger_type);

    msleep(10);

    return 0;
}


/*******************************************************
Function:
	Read goodix touchscreen version function.

Input:
	client:	i2c client struct.
	version:address to store version info

Output:
	Executive outcomes.0---succeed.
*******************************************************/
s32 gtp_read_version(struct i2c_client *client, u16* version)
{
    s32 ret = -1;
    u8 buf[8] = {GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff};

    GTP_DEBUG_FUNC();

    ret = gtp_i2c_read(client, buf, sizeof(buf));
    if (ret < 0)
    {
        GTP_ERROR("GTP read version failed");
        return ret;
    }

    if (version)
    {
        *version = (buf[7] << 8) | buf[6];
    }

    if (buf[5] == 0x00)
    {
        printk("IC Version: %c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[7], buf[6]);
    }
    else
    {
        printk("IC Version: %c%c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
    }
    return ret;

}
s32 gtp_read_report_rate(struct i2c_client *client, u16* rate)
{
    s32 ret = -1;
    u8 buf[3] = {GTP_REG_REPORT_RATE>> 8, GTP_REG_REPORT_RATE & 0xff};

    GTP_DEBUG_FUNC();

    ret = gtp_i2c_read(client, buf, sizeof(buf));
    if (ret < 0)
    {
        GTP_ERROR("GTP read report rate failed");
        return ret;
    }

    if (rate)
    {
        *rate = ((buf[2]&0x0f)+5);
    }


    GTP_INFO("report_rate,*rate=%d ms one point",*rate);

    return ret;
}

s32 gtp_read_fw_cfg_version(void)
{
	s32 ret = -1;
	u8 cfg[3] = {0};
	ret = gtp_read_version(i2c_connect_client, &gtp_fw_version);
  if (ret < 0)
   {
        	GTP_ERROR("GTP read version failed");
        	return ret;
   }
  cfg[0] = GTP_REG_CONFIG_DATA >> 8;
	cfg[1] = GTP_REG_CONFIG_DATA & 0xff;
	ret = gtp_i2c_read(i2c_connect_client, cfg, 3);
	if( ret < 0 )
	{
		GTP_ERROR("Read config version failed.\n");
		return ret;
	}
//	if(gtp_cfg_version < cfg[2])
	gtp_cfg_version =cfg[2];

	return 0;
}

/*******************************************************
Function:
	I2c test Function.

Input:
	client:i2c client.

Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
static s8 gtp_i2c_test(struct i2c_client *client)
{
    u8 test[3] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
    u8 retry = 0;
    s8 ret = -1;

    GTP_DEBUG_FUNC();

    while(retry++ < 5)
    {
        ret = gtp_i2c_read(client, test, 3);
        if (ret > 0)
        {
            return ret;
        }
        GTP_ERROR("GTP i2c test failed time %d.",retry);
        msleep(10);
    }
    return ret;
}


////////////////////////////////////////////////////////////////////////////
static int goodix_ts_suspend(struct device *dev)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
    GTP_DEBUG_FUNC();
	return goodix_ts_disable(ts->input_dev);
}

static int goodix_ts_resume(struct device *dev)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
    GTP_DEBUG_FUNC();
	return goodix_ts_enable(ts->input_dev);
}


#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct goodix_ts_data *ts_data =
		container_of(self, struct goodix_ts_data, fb_notif);
    GTP_DEBUG_FUNC();

	if (evdata && evdata->data && event == FB_EVENT_BLANK && ts_data &&
			ts_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			goodix_ts_resume(&ts_data->client->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			goodix_ts_suspend(&ts_data->client->dev);
	}

	return 0;
}
#elif defined(CONFIG_PM)
static const struct dev_pm_ops goodix_pm_ops = {
	.suspend	= mxt_suspend,
	.resume	= mxt_resume,
};
#endif

//////////////////////////////////////////////////////////////////////////////////////
/*******************************************************
Function:
	Request irq Function.

Input:
	ts:private data.

Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
static s8 gtp_request_irq(struct goodix_ts_data *ts)
{
    s32 ret = -1;

    GTP_DEBUG("INT trigger type:%x", ts->int_trigger_type);
    GTP_DEBUG("GTP ts->client->irq = %d",ts->client->irq);

    ret  = request_irq(ts->client->irq,
                       goodix_ts_irq_handler,
                       ts->pdata->irqflag,
                       ts->client->name,
                       ts);
    if (ret)
    {
        GTP_ERROR("Request IRQ failed!ERRNO:%d.", ret);
        hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        ts->timer.function = goodix_ts_timer_handler;
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
        return -1;
    }
    else
    {
        ts->use_irq = 1;
        gtp_irq_disable(ts);
        return 0;
    }
}

/*******************************************************
Function:
	Request input device Function.

Input:
	ts:private data.

Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
static s8 gtp_request_input_dev(struct goodix_ts_data *ts)
{
    s8 ret = -1;
    s8 phys[32];
#if GTP_HAVE_TOUCH_KEY
    u8 index = 0;
#endif

    GTP_DEBUG_FUNC();

    ts->input_dev = input_allocate_device();
    if (ts->input_dev == NULL)
    {
        GTP_ERROR("Failed to allocate input device.");
        return -ENOMEM;
    }



#if GTP_ICS_SLOT_REPORT
    input_mt_init_slots(ts->input_dev, 10);     // in case of "out of memory"
#else
//for Android4.0, use MT-A protocol
   ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif

#if GTP_HAVE_TOUCH_KEY
	atomic_set(&gt_keypad_enable, 1);
    for (index = 0; index < GTP_MAX_KEY_NUM; index++)
    {
        input_set_capability(ts->input_dev,EV_KEY,touch_key_array[index]);
    }
#endif

#if GTP_SLIDE_WAKEUP
    set_bit(EV_KEY, ts->input_dev->evbit);
    set_bit(KEY_WAKEUP, ts->input_dev->keybit);
    set_bit(KEY_GESTURE_SLIDE_DOWN, ts->input_dev->keybit);
    set_bit(KEY_GESTURE_SLIDE_LEFT, ts->input_dev->keybit);
    set_bit(KEY_GESTURE_SLIDE_RIGHT, ts->input_dev->keybit);
    set_bit(KEY_GESTURE_SLIDE_C, ts->input_dev->keybit);
    set_bit(KEY_GESTURE_SLIDE_O, ts->input_dev->keybit);
#endif 
#if GTP_CHANGE_X2Y
    GTP_SWAP(ts->abs_x_max, ts->abs_y_max);
#endif

    input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->pdata->screen_x, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->pdata->screen_y, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, GTP_MAX_TOUCH, 0, 0);

    set_bit(EV_ABS, ts->input_dev->evbit);
    set_bit(EV_SYN, ts->input_dev->evbit);
    set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

    sprintf(phys, "input/ts");
    ts->input_dev->name = goodix_ts_name;
    ts->input_dev->phys = phys;
    ts->input_dev->id.bustype = BUS_I2C;
    ts->input_dev->id.vendor = 0xDEAD;
    ts->input_dev->id.product = 0xBEEF;
    ts->input_dev->id.version = 10427;

#ifdef CONFIG_FB
	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if (ret)
		GTP_ERROR("Unable to register fb_notifier: %d\n",ret);
#endif
    input_set_drvdata(ts->input_dev, ts);

    ret = input_register_device(ts->input_dev);
    if (ret)
    {
        GTP_ERROR("Register %s input device failed", ts->input_dev->name);
        return -ENODEV;
    }



    return 0;
}
/***********************************************************************************************
Name	:

Input	:


Output	:

function	:

***********************************************************************************************/

/*******************************************************
   ********  1--active  0--not active***********
*******************************************************/
int goodix_active(void)
{
   GTP_DEBUG("enter %s\n",__FUNCTION__);
   return 1;
}
/*******************************************************
***********check firmware if need update***********
*******************************************************/
int goodix_firmware_need_update(void)
{
    st_fw_head fw_head;
    int ret = -1;

    GTP_INFO(" check if need firmware update...\n");
    GTP_DEBUG("enter %s\n",__FUNCTION__);

    ret = gup_get_ic_fw_msg(i2c_connect_client);
    if(FAIL == ret)
    {
        GTP_ERROR("[update_proc]get ic message fail.");
        return FAIL;
    }
    ret = gup_check_update_file(i2c_connect_client, &fw_head, 0);
    return ret;
 }

/*******************************************************
*********************do firmware update ***************
*******************************************************/
int goodix_firmware_do_update(void)
{
     int ret = -1;

    GTP_DEBUG("enter %s\n",__FUNCTION__);
   ret = goodix_firmware_need_update();
   if(ret)
   {
      struct goodix_ts_data *ts = NULL;
      GTP_DEBUG_FUNC();
      ts = i2c_get_clientdata(i2c_connect_client);

      if(0 == gup_init_update_proc(ts))
	  	return -1;
      else
	{
		gtp_read_fw_cfg_version();
		return 0;
      	}
   }

   return ret;
}

/*******************************************************
*******************check if need calibrate***********
*******************************************************/

int goodix_need_calibrate(void)
{
   GTP_DEBUG("enter %s\n",__FUNCTION__);
   return 1;
}

/*******************************************************
 ******************system write "calibrate"************
*******************************************************/

int goodix_calibrate(void)
{
 GTP_DEBUG("enter %s\n",__FUNCTION__);
		  return 0;
}

/*******************************************************
 ******************get firmware version **************
*******************************************************/
int goodix_get_firmware_version(char * version )
{
    gtp_read_fw_cfg_version();

    if(tp_supported)
 	return sprintf(version, "%s:%s:0x%4X:0x%2X", yl_cfg[tp_index].tp_name, TW_IC_PREFIX_NAME, gtp_fw_version, gtp_cfg_version);
    else
 	return sprintf(version, "%s:%s:0x%4X:0x%2X", "----", TW_IC_PREFIX_NAME, gtp_fw_version, gtp_cfg_version);
}

/*******************************************************
  ******************system write "reset"***************
*******************************************************/
int goodix_reset_touchscreen(void)
{
   GTP_DEBUG("enter %s\n",__FUNCTION__);
	return 1;
}

//added by taokai
u8 gt9158_has_resumed = 0;
#if  TW_GLOVE_SWITCH

u32 yl_get_charger_status(void);

void yl_chg_status_changed(void)
{
	u32 chg_status;
	int count = 0;
	u8 write_val;
	s32 ret = -1;
	u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 0x0a};
	u8 i2c_command_buf[3] = {0x80,0x46};

	chg_status = yl_get_charger_status();
	pr_err("tw get charger status is: %d\n", chg_status);
	//GTP_DEBUG("[GTP]:%s glove mode  charge status has change, chg_status = %d.\n", __func__, chg_status);

	if (glove_switch == MODE_GLOVE)
	{
		if(gt9158_has_resumed == 1)
		{
			if (chg_status == 0)
			{
			write_val = 0x0b;
			} else
			{
			write_val = 0x0a;
			}
            i2c_command_buf[2]=write_val;
			if(i2c_connect_client != NULL)
        		ret = gtp_i2c_write(i2c_connect_client, i2c_command_buf, 3);
			if (ret < 0)
				{
					printk("failed to set finger mode flag into 0x8046\n");
				}

			i2c_control_buf[0] = 0x80;
      i2c_control_buf[1] = 0x40;

			for (count =0; count<5; count++)
			{
			i2c_control_buf[2] = write_val;
			if(i2c_connect_client != NULL)
        		ret = gtp_i2c_write(i2c_connect_client, i2c_control_buf, 3);
			if(ret > 0)
			{
				GTP_DEBUG("[GTS]:%s write 0x8040 register success ,value = %d.\n", __func__, write_val);
				break;
			} else
			{
				printk(KERN_ERR "%s: glove mode  write 0x8040 error, count = %d, write_val = %d.\n",
					__func__, count, write_val);
			}
			}
		}
	}
}
EXPORT_SYMBOL(yl_chg_status_changed);
touch_mode_type goodix_get_mode(void)
{
	return glove_switch;//sprintf(buf, "%d",glove_switch);
}

int goodix_set_mode(touch_mode_type glove)
{
	u8 ret=0;
	struct goodix_ts_data *ts;

	if (i2c_connect_client == NULL)
		return -1;

	ts = i2c_get_clientdata(i2c_connect_client);
	glove_switch=glove;

	if (ts->use_irq)
	{
	    gtp_irq_disable(ts);
	}
	else
	{
	    hrtimer_cancel(&ts->timer);
	}

#ifdef CONFIG_TOUCHSCREEN_GT9XX_YL_COVER_WINDOW_CFG
	if(glove_switch!=MODE_GLOVE && glove_switch!=MODE_WINDOW )
#else
	if(glove_switch!=MODE_GLOVE)
#endif
	{
	    printk(KERN_ERR"[GTP]:switch to normal mode.\n");
	    ts->gtp_cfg_len = yl_cfg[tp_index].firmware_size;
	    memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
	    memcpy(&config[GTP_ADDR_LENGTH], yl_cfg[tp_index].firmware, ts->gtp_cfg_len);
	}
#ifdef CONFIG_TOUCHSCREEN_GT9XX_YL_COVER_WINDOW_CFG
	else if( glove_switch == MODE_WINDOW )
	{
	    printk(KERN_ERR"[GTP]:switch to window mode.\n");
	    ts->gtp_cfg_len = yl_cfg_window[tp_index].firmware_size;
	    memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
	    memcpy(&config[GTP_ADDR_LENGTH], yl_cfg_window[tp_index].firmware, ts->gtp_cfg_len);

	}
#endif
	else
	{
	    printk(KERN_ERR"[GTP]:switch to glove mode.\n");
	    ts->gtp_cfg_len = yl_cfg_glove[tp_index].firmware_size;
	    memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
	    memcpy(&config[GTP_ADDR_LENGTH], yl_cfg_glove[tp_index].firmware, ts->gtp_cfg_len);
	}
	gtp_reset_guitar(ts->client, 20);
	ret = gtp_send_cfg(ts->client);
	if (ret >= 0)
	{
	    	msleep(400);
 		GTP_INFO("send config successed!");
	}

	if (glove_switch == MODE_GLOVE) {
	    yl_chg_status_changed();
	}
	if(!ts->gtp_is_suspend){
		if (ts->use_irq)
		{
		    gtp_irq_enable(ts);
		}
		else
		{
		    hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		}
	}
	printk(KERN_ERR"%s: mode  = %d\n", __func__,glove_switch);
	return 1;
}
#else
void yl_chg_status_changed(void)
{
    GTP_DEBUG("enter %s\n",__FUNCTION__);
}
touch_mode_type goodix_get_mode(void)
{
    GTP_DEBUG("enter %s\n",__FUNCTION__);
   return 1;
}

int goodix_set_mode(touch_mode_type work_mode)
{
	GTP_DEBUG("enter %s\n",__FUNCTION__);
    return 1;
}
#endif

/* begin to add interface function for TW_GLOVE_SWITCH by liushilong@yulong.com on 2014-11-6 17:15*/
#ifdef CONFIG_TOUCHSCREEN_GT9XX_YL_TW_GLOVE_SWITCHy
void window_mode(bool on)
{
	if(on){
		int tmp = glove_switch;
		goodix_set_mode(MODE_WINDOW);
		glove_switch = tmp;
	}else{
		goodix_set_mode(glove_switch);
	}
}
int glove_windows_switch(int in_hall)
{
	if(in_hall)
	{
		window_mode(true);
	} else {
		window_mode(false);
	}
	return 0;
}
#else
void window_mode(bool on)
{
    GTP_DEBUG("Not support window mode.\n");
}

int glove_windows_switch(int in_hall)
{
    GTP_DEBUG("Not support window switch.\n");
	return 0;
}
#endif
EXPORT_SYMBOL_GPL(glove_windows_switch);
/* end to add interface function for TW_GLOVE_SWITCH by liushilong@yulong.com on 2014-11-6 17:15*/


/*******************************************************
  ****************get "oreitation:X" ************
*******************************************************/
touch_oreitation_type goodix_get_oreitation(void)
{
   GTP_DEBUG("enter %s\n",__FUNCTION__);
   return 1;
}


int goodix_set_oreitation(touch_oreitation_type oreitate)
{
   GTP_DEBUG("enter %s\n",__FUNCTION__);
   return 1;
}

/*******************************************************
  **********read buf[256]: ef:ab [???????]?šºo??š¬  ********
*******************************************************/
int goodix_read_regs(char * buf)
{
    int ret = -1;
    u8 cfg[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH] = { 0 };

    GTP_DEBUG_FUNC();

    cfg[0] = GTP_REG_CONFIG_DATA >> 8;
    cfg[1] = GTP_REG_CONFIG_DATA & 0xff;
    ret = gtp_i2c_read(i2c_connect_client, cfg, GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
    if( ret < 0 )
    {
        GTP_ERROR("Read config version failed.\n");
        return ret;
    }

    for(ret=0; ret<GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH; ret++)
    {
	printk("reg%d = 0x%x\n", ret, cfg[ret]);
    }

    return 1;
}

/*******************************************************
  **********write buf[256]: ef:ab [???????]?šºo??š¬ ********
*******************************************************/
int goodix_write_regs(const char * buf)
{
	GTP_DEBUG("enter %s\n",__FUNCTION__);
	return 1;
}

/*******************************************************
  ***************tw debug on or off *************
*******************************************************/
int goodix_debug(int val)
{
	GTP_DEBUG("enter %s\n",__FUNCTION__);
	if(val)
		GTP_DEBUG_ON=1;
	else
		GTP_DEBUG_ON=0;
	return 1;
}

int goodix_vendor(char*  vendor)
{
    	return sprintf(vendor, "%s", "goodix");
}
#if GTP_SLIDE_WAKEUP
int goodix_get_wakeup_gesture(char*  gesture)
{
    	return sprintf(gesture, "%s", (char *)wakeup_slide);
}

int goodix_get_gesture_ctrl(char *gesture_ctrl)
{
    return sprintf(gesture_ctrl, "0x%08x", support_gesture);
}

int goodix_gesture_ctrl(const char*  gesture_buf)
{
	char *gesture;
	int buf_len;
	char *gesture_p;
	char *temp_p;
	/*longjiang add start 20140604*/
	s8 ret = -1;
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);
	/*add end*/

	buf_len = strlen(gesture_buf);

	GTP_DEBUG("%s buf_len = %d.\n", __func__, buf_len);
    GTP_DEBUG("%s gesture_buf:%s.\n", __func__, gesture_buf);

	gesture_p = kzalloc(buf_len + 1, GFP_KERNEL);
	if(gesture_p == NULL) {
		GTP_ERROR("%s: alloc mem error.\n", __func__);
		return -1;
	}
	temp_p = gesture_p;
	strlcpy(gesture_p, gesture_buf, buf_len + 1);

	while(gesture_p) {
		gesture = strsep(&gesture_p, ",");
		GTP_DEBUG("%s gesture:%s.\n", __func__, gesture);

		if (!strncmp(gesture, "up=",3)) {
			if(!strncmp(gesture+3, "true", 4)) {
				GTP_DEBUG("%s: enable up slide wakeup func.\n", __func__);
				support_gesture |= TW_SUPPORT_UP_SLIDE_WAKEUP;
			} else if(!strncmp(gesture+3, "false", 5)) {
				GTP_DEBUG("%s: disable up slide wakeup func.\n", __func__);
				support_gesture &= ~TW_SUPPORT_UP_SLIDE_WAKEUP;
			}
			continue;
		}

		if (!strncmp(gesture, "down=",5)) {
			if(!strncmp(gesture+5, "true", 4)) {
				GTP_DEBUG("%s: enable down slide wakeup func.\n", __func__);
				support_gesture |= TW_SUPPORT_DOWN_SLIDE_WAKEUP;
			} else if(!strncmp(gesture+5, "false", 5)) {
				GTP_DEBUG("%s: disable down slide wakeup func.\n", __func__);
				support_gesture &= ~TW_SUPPORT_DOWN_SLIDE_WAKEUP;
			}
			continue;
		}

		if (!strncmp(gesture, "left=",5)) {
			if(!strncmp(gesture+5, "true", 4)) {
				GTP_DEBUG("%s: enable left slide wakeup func.\n", __func__);
				support_gesture |= TW_SUPPORT_LEFT_SLIDE_WAKEUP;
			} else if(!strncmp(gesture+5, "false", 5)) {
				GTP_DEBUG("%s: disable left slide wakeup func.\n", __func__);
				support_gesture &= ~TW_SUPPORT_LEFT_SLIDE_WAKEUP;
			}
			continue;
		}

		if (!strncmp(gesture, "right=",6)) {
			if(!strncmp(gesture+6, "true", 4)) {
				GTP_DEBUG("%s: enable right slide wakeup func.\n", __func__);
				support_gesture |= TW_SUPPORT_RIGHT_SLIDE_WAKEUP;
			} else if(!strncmp(gesture+6, "false", 5)) {
				GTP_DEBUG("%s: disable right slide wakeup func.\n", __func__);
				support_gesture &= ~TW_SUPPORT_RIGHT_SLIDE_WAKEUP;
			}
			continue;
		}

		if (!strncmp(gesture, "double_click=",13)) {
			if(!strncmp(gesture+13, "true", 4)) {
				GTP_DEBUG("%s: enable double click wakeup func.\n", __func__);
				support_gesture |= TW_SUPPORT_DOUBLE_CLICK_WAKEUP;
			} else if(!strncmp(gesture+13, "false", 5)) {
				GTP_DEBUG("%s: disable double click wakeup func.\n", __func__);
				support_gesture &= ~TW_SUPPORT_DOUBLE_CLICK_WAKEUP;
			}
			continue;
		}
		if (!strncmp(gesture, "e=",2)) {
			if(!strncmp(gesture+2, "true", 4)) {
				GTP_DEBUG("%s: enable e click wakeup func.\n", __func__);
				support_gesture |= TW_SUPPORT_E_SLIDE_WAKEUP;
			} else if(!strncmp(gesture+2, "false", 5)) {
				GTP_DEBUG("%s: disable e click wakeup func.\n", __func__);
				support_gesture &= ~TW_SUPPORT_E_SLIDE_WAKEUP;
			}
			continue;
		}
		if (!strncmp(gesture, "o=",2)) {
			if(!strncmp(gesture+2, "true", 4)) {
				GTP_DEBUG("%s: enable o click wakeup func.\n", __func__);
				support_gesture |= TW_SUPPORT_O_SLIDE_WAKEUP;
			} else if(!strncmp(gesture+2, "false", 5)) {
				GTP_DEBUG("%s: disable o click wakeup func.\n", __func__);
				support_gesture &= ~TW_SUPPORT_O_SLIDE_WAKEUP;
			}
			continue;
		}
		if (!strncmp(gesture, "w=",2)) {
			if(!strncmp(gesture+2, "true", 4)) {
				GTP_DEBUG("%s: enable w click wakeup func.\n", __func__);
				support_gesture |= TW_SUPPORT_W_SLIDE_WAKEUP;
			} else if(!strncmp(gesture+2, "false", 5)) {
				GTP_DEBUG("%s: disable w click wakeup func.\n", __func__);
				support_gesture &= ~TW_SUPPORT_W_SLIDE_WAKEUP;
			}
			continue;
		}
		if (!strncmp(gesture, "c=",2)) {
			if(!strncmp(gesture+2, "true", 4)) {
				GTP_DEBUG("%s: enable c click wakeup func.\n", __func__);
				support_gesture |= TW_SUPPORT_C_SLIDE_WAKEUP;
			} else if(!strncmp(gesture+2, "false", 5)) {
				GTP_DEBUG("%s: disable c click wakeup func.\n", __func__);
				support_gesture &= ~TW_SUPPORT_C_SLIDE_WAKEUP;
			}
			continue;
		}
		if (!strncmp(gesture, "m=",2)) {
			if(!strncmp(gesture+2, "true", 4)) {
				GTP_DEBUG("%s: enable m click wakeup func.\n", __func__);
				support_gesture |= TW_SUPPORT_M_SLIDE_WAKEUP;
			} else if(!strncmp(gesture+2, "false", 5)) {
				GTP_DEBUG("%s: disable m click wakeup func.\n", __func__);
				support_gesture &= ~TW_SUPPORT_M_SLIDE_WAKEUP;
			}
			continue;
		}
	}
	/*longjiang add start 20140604*/
	GTP_DEBUG("%s: gt9158_has_resumed=%d\n", __func__, gt9158_has_resumed);
	if (gt9158_has_resumed == 0) {
		gtp_reset_guitar(ts->client, 20);
		if(support_gesture & TW_SUPPORT_GESTURE_IN_ALL)
		{

/* begin to judge doze status for entering doze mode by liushilong@yulong.com on 2014-11-6 17:15*/
			mutex_lock(&ts->doze_mutex);
			if( doze_status != DOZE_DISABLED ){
				ret = gtp_enter_doze(ts);
			}
			mutex_unlock(&ts->doze_mutex);
/* end to judge doze status for entering doze mode by liushilong@yulong.com on 2014-11-6 17:15*/

			if (ret < 0) {
				printk("%s: gtp enter doze failed, ret=%d\n", __func__, ret);
			} else {
				printk("%s: gtp enter doze success\n", __func__);
			}
		} else {
			if (ts->use_irq)
			{
				gtp_irq_disable(ts);
			} else {
				hrtimer_cancel(&ts->timer);
			}

			ret = gtp_enter_sleep(ts);
			if (ret < 0) {
				printk("%s: gtp enter sleep failed, ret=%d\n", __func__, ret);
			} else {
				printk("%s: gtp enter sleep success\n", __func__);
			}
			gtp_enable_irq_wake(ts, 0);

			if(ts->pdata->suspend)
				ts->pdata->suspend();
		}
	}
	/*add end*/

	kfree(temp_p);
	return 1;
}
#endif
touchscreen_ops_tpye goodix_ops=
{
	.touch_id					= 0,
	.touch_type			  = 1,
	.active						= goodix_active,
	.firmware_need_update		= goodix_firmware_need_update,
	.firmware_do_update			= goodix_firmware_do_update,
	.need_calibrate				= goodix_need_calibrate,
	.calibrate					= goodix_calibrate,
	.get_firmware_version		= goodix_get_firmware_version,
	.reset_touchscreen			= goodix_reset_touchscreen,
	.get_mode					= goodix_get_mode,
	.set_mode					= goodix_set_mode,
	.get_oreitation				= goodix_get_oreitation,
	.set_oreitation				= goodix_set_oreitation,
	.read_regs					= goodix_read_regs,
	.write_regs					= goodix_write_regs,
	.debug						= goodix_debug,
	.get_vendor				= goodix_vendor,
#if GTP_SLIDE_WAKEUP
	.get_wakeup_gesture			= goodix_get_wakeup_gesture,
	.get_gesture_ctrl			= goodix_get_gesture_ctrl,
	.gesture_ctrl				=goodix_gesture_ctrl,
#endif
};

#ifdef CONFIG_OF

static struct regulator *ts_vdd;
static struct regulator *ts_vcc_i2c;
/*
static const char * GTP_VDD;
static const char * GTP_VCC_IO;
static unsigned int GTP_GPIO_RESET;
static unsigned int GTP_GPIO_IRQ;
*/


#define GTP_VDD "vdd_ana"
#define GTP_VCC_IO "vcc_i2c"
//change qysh
//#define GTP_GPIO_RESET 16
//#define GTP_GPIO_IRQ  17
//end
#define REGULATOR_DEV	&i2c_connect_client->dev

static int goodix_pinctrl_init(struct goodix_ts_data *ts)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	ts->ts_pinctrl = devm_pinctrl_get(&(ts->client->dev));
	if (IS_ERR_OR_NULL(ts->ts_pinctrl)) {
		dev_dbg(&ts->client->dev,
			"Target does not use pinctrl\n");
		retval = PTR_ERR(ts->ts_pinctrl);
		ts->ts_pinctrl = NULL;
		return retval;
	}

	ts->gpio_state_active
		= pinctrl_lookup_state(ts->ts_pinctrl, "pmx_ts_active");
	if (IS_ERR_OR_NULL(ts->gpio_state_active)) {
		dev_dbg(&ts->client->dev,
			"Can not get ts default pinstate\n");
		retval = PTR_ERR(ts->gpio_state_active);
		ts->ts_pinctrl = NULL;
		return retval;
	}

	ts->gpio_state_suspend
		= pinctrl_lookup_state(ts->ts_pinctrl, "pmx_ts_suspend");
	if (IS_ERR_OR_NULL(ts->gpio_state_suspend)) {
		dev_dbg(&ts->client->dev,
			"Can not get ts sleep pinstate\n");
		retval = PTR_ERR(ts->gpio_state_suspend);
		ts->ts_pinctrl = NULL;
		return retval;
	}

	return 0;
}

static int goodix_pinctrl_select(struct goodix_ts_data *ts,
						bool on)
{
	struct pinctrl_state *pins_state;
	int ret;

	pins_state = on ? ts->gpio_state_active
		: ts->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(ts->ts_pinctrl, pins_state);
		if (ret) {
			dev_err(&ts->client->dev,
				"can not set %s pins\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
			return ret;
		}
	} else
		dev_err(&ts->client->dev,
			"not a valid '%s' pinstate\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");

	return 0;
}

static int goodix_power_init(bool on)
{
	int rc;

	if(!GTP_PWR_EN)
		return 0;

	if (!on)
		goto gtp_pwr_deinit;

	ts_vdd = regulator_get(REGULATOR_DEV, GTP_VDD);
	if (IS_ERR(ts_vdd)) {
		rc = PTR_ERR(ts_vdd);
		pr_err("Touchscreen Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(ts_vdd) > 0) {
		rc = regulator_set_voltage(ts_vdd, 2600000,3300000);
		if (rc) {
			pr_err("Touchscreen Regulator set failed vdd rc=%d\n", rc);
			goto gtp_reg_vdd_put;
		}
	}

	ts_vcc_i2c = regulator_get(REGULATOR_DEV, GTP_VCC_IO);
	if (IS_ERR(ts_vcc_i2c)) {
		rc = PTR_ERR(ts_vcc_i2c);
		pr_err("Touchscreen Regulator get failed vcc_i2c rc=%d\n", rc);
		goto gtp_reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(ts_vcc_i2c) > 0) {
		rc = regulator_set_voltage(ts_vcc_i2c, 1800000,1800000);
		if (rc) {
			pr_err("Touchscreen Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto gtp_reg_vcc_i2c_put;
		}
	}

	return 0;

gtp_reg_vcc_i2c_put:
	regulator_put(ts_vcc_i2c);
gtp_reg_vdd_set_vtg:
	if (regulator_count_voltages(ts_vdd) > 0)
		regulator_set_voltage(ts_vdd, 0, 3300000);
gtp_reg_vdd_put:
	regulator_put(ts_vdd);
	return rc;

gtp_pwr_deinit:
	if (regulator_count_voltages(ts_vdd) > 0)
		regulator_set_voltage(ts_vdd, 0, 3300000);

	regulator_put(ts_vdd);

	if (regulator_count_voltages(ts_vcc_i2c) > 0)
		regulator_set_voltage(ts_vcc_i2c, 0, 1800000);

	regulator_put(ts_vcc_i2c);

	return 0;
}

static int goodix_ts_power(int on)
{
	int rc;
	printk("[gt]:goodix_ts_power.\n");

	if(!GTP_PWR_EN)
		return 0;

	if (!on)
		goto goodix_power_off;

	rc = regulator_enable(ts_vdd);
	if (rc) {
		pr_err("Touchscreen Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(ts_vcc_i2c);
	if (rc) {
		pr_err("Touchscreen Regulator vdd_i2c enable failed rc=%d\n", rc);
		regulator_disable(ts_vdd);
	}

	return rc;

goodix_power_off:
	rc = regulator_disable(ts_vdd);
	if (rc) {
		pr_err("Touchscreen Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(ts_vcc_i2c);
	if (rc) {
		pr_err("Touchscreen Regulator vdd_i2c disable failed rc=%d\n", rc);
		rc = regulator_enable(ts_vdd);
	}

	return rc;
}


static int goodix_ts_gpio_init(bool on)
{
	int ret = 0;
	printk("[gt]:goodix_ts_gpio_init.\n");

	if(!on)
		goto gtp_gpio_deinit;

	/* touchscreen reset gpio request and init */
	if (gpio_is_valid(gtpdata->gpio_reset)) {
		ret = gpio_request(gtpdata->gpio_reset, "goodix-ts-reset");
		if (ret){
			pr_err("TOUCH:%s:Failed to request GPIO %d\n",__func__, gtpdata->gpio_reset);
			return ret;
		}
		gpio_direction_output(gtpdata->gpio_reset, 1);
	} else {
		pr_err("TOUCH:%s:irq gpio not provided\n",__func__);
		return -1;
	}

	if (gpio_is_valid(gtpdata->gpio_irq)) {
		ret = gpio_request(gtpdata->gpio_irq, "goodix-ts-irq");
		if (ret){
			pr_err("TOUCH:%s: Failed to request GPIO %d\n",__func__, gtpdata->gpio_irq);
			gpio_free(gtpdata->gpio_irq);
			return ret;
		}
		gpio_direction_input(gtpdata->gpio_irq);
	} else {
		pr_err("TOUCH:%s: reset gpio not provided\n",__func__);
		gpio_free(gtpdata->gpio_reset);
		return -1;
	}

	return 0;


gtp_gpio_deinit:
	gpio_free(gtpdata->gpio_reset);
	gpio_free(gtpdata->gpio_irq);

	return 0;
}

static int goodix_ts_hw_init(void)

{
	int ret = 0;

	ret = goodix_ts_gpio_init(1);
	if(ret)
	{
		pr_err("Touchscreen GPIO init failed!\n");
		return ret;
	}

	ret = goodix_power_init(1);
	if(ret)
	{
		pr_err("Touchscreen Power init failed!\n");
		goodix_ts_gpio_init(0);
		return ret;
	}

	pr_debug("TOUCH: requst and init resource complete\n");

	return 0;
}

static void goodix_ts_release(void)
{
	int ret = 0;

	ret = goodix_ts_gpio_init(0);
	if(ret)
	{
		pr_err("Touchscreen GPIO release failed!\n");
	}

	ret = goodix_power_init(0);
	if(ret)
	{
		pr_err("Touchscreen Power release failed!\n");
	}

}

static int goodix_ts_reset(int ms)
{
    struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

    mutex_lock(&ts->reset_mutex);
    gpio_direction_output(gtpdata->gpio_irq, 0);
    gpio_direction_output(gtpdata->gpio_reset, 0);
    msleep(ms);

    //step2:select I2C slave addr,INT:0--0x5d;1--0x14.
    gpio_direction_output(gtpdata->gpio_irq, 0);
    mdelay(2);

    //end select I2C slave addr
    gpio_direction_output(gtpdata->gpio_reset, 1);
    mdelay(6);                          //must > 3ms

    gpio_direction_input(gtpdata->gpio_reset);
    gpio_direction_output(gtpdata->gpio_irq, 0);
    msleep(50);
    gpio_direction_input(gtpdata->gpio_irq);
    mutex_unlock(&ts->reset_mutex);

/* begin to transplant TW GTP_ESD_PROTECT code from 8675_C00, reset init watchdog by liushilong@yulong.com on 2014-11-6 17:24*/
#if GTP_ESD_PROTECT
    gtp_init_ext_watchdog(i2c_connect_client);
#endif
/* end to transplant TW GTP_ESD_PROTECT code from 8675_C00, reset init watchdog by liushilong@yulong.com on 2014-11-6 17:24*/
    return 0;
}


static int goodix_ts_sleep(void)
{
	//irq output 0
//	gpio_direction_output(gtpdata->gpio_irq, 0);

	if( GTP_PWR_EN && GTP_SLEEP_PWR_EN)
		goodix_ts_power(0);

	mdelay(5);
	return 0;
}


static int goodix_ts_wakeup(void)
{

	if( GTP_PWR_EN && GTP_SLEEP_PWR_EN)
		goodix_ts_power(1);

	goodix_ts_reset(20);

	return 0;
}

static int panel_xres = 0;

static int __init goodix_parse_xres(char *str)
{
	if (!str)
		return -EINVAL;

	panel_xres = simple_strtoul(str, NULL, 0);

	return 0;
}

early_param("panel.xres", goodix_parse_xres);

static int goodix_ts_parse_dt(struct device *dev, struct tw_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	int ret;
	enum of_gpio_flags flags;

	/* reset, irq gpio info */
	pdata->gpio_reset = of_get_named_gpio_flags(np, "goodix,reset-gpio",
				0, &flags);
	printk("[gt]:reset-gpio = %d\n", pdata->gpio_reset);

	pdata->gpio_irq = of_get_named_gpio_flags(np, "goodix,irq-gpio",
				0, &flags);
	printk("[gt]:irq-gpio = %d\n", pdata->gpio_irq);

	ret = of_property_read_u32(np, "goodix,irq_flags", (unsigned int *)&pdata->irqflag);
	if (ret) {
		dev_err(dev, "Looking up %s property in node %s failed",
			"goodix,irq_flags", np->full_name);
		return -ENODEV;
	}
	printk("[gt]:irq-flag = %d\n", (int)pdata->irqflag);
	pdata->irqflag = 2;

	if (panel_xres <= 0) {
		ret = of_property_read_u32(np, "goodix,screen_x", &pdata->screen_x);
		if (ret) {
			dev_err(dev, "Looking up %s property in node %s failed",
					"goodix,screen_x", np->full_name);
			return -ENODEV;
		}

		ret = of_property_read_u32(np, "goodix,screen_y", &pdata->screen_y);
		if (ret) {
			dev_err(dev, "Looking up %s property in node %s failed",
					"goodix,screen_y", np->full_name);
			return -ENODEV;
		}
	} else {
		pdata->screen_x = panel_xres;
		pdata->screen_y = ( (panel_xres * 16 ) / 9);
	}

	printk("[gt]:screen x,y = %d, %d\n", pdata->screen_x, pdata->screen_y);


	ret = of_property_read_u32(np, "goodix,pwr_en", &pdata->pwr_en);
	ret = of_property_read_u32(np, "goodix,sleep_pwr_en", &pdata->sleep_pwr_en);
	ret = of_property_read_string(np, "goodix,vcc_i2c_supply", &pdata->ts_vcc_i2c);
	ret = of_property_read_string(np, "goodix,vdd_supply", &pdata->ts_vdd);

	printk("[gt]:pwr_en=%d, sleep_pwr_en=%d, vcc_i2c=%s, vdd=%s\n", pdata->pwr_en,
				pdata->sleep_pwr_en, pdata->ts_vcc_i2c, pdata->ts_vdd);


	pdata->init = goodix_ts_hw_init;
	pdata->release = goodix_ts_release;
	pdata->reset = goodix_ts_reset;
	pdata->power = goodix_ts_power;
	pdata->suspend = goodix_ts_sleep;
	pdata->resume = goodix_ts_wakeup;

	return 0;
}
#endif
/******************glove switch **************************/

static ssize_t glove_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	u8 ret = 0;
	u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP};
    ret = gtp_i2c_read(i2c_connect_client, i2c_control_buf, 3);
    if( ret < 0 )
    {
        GTP_ERROR("Read register 0x8040 failed.\n");
        return ret;
    }
	else
		{
		printk("[gt]:register 0x8040 value =%d.\n",i2c_control_buf[2]);
		}
	ret = sprintf(buf, "%d%d",glove_switch,i2c_control_buf[2]);
	return ret;
}

static ssize_t glove_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 ret=0;
    struct goodix_ts_data *ts;

	u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 0x0A};
	//memset(ts, 0, sizeof(*ts));
   	ts = i2c_get_clientdata(i2c_connect_client);
	if(buf==NULL)
	{
		GTP_INFO("[FTS]: buf is NULL!\n");
		return -ENOMEM;
	}

	glove_switch = (unsigned char)simple_strtoul(buf, NULL, 10);
	GTP_INFO("input arguments glove_switch=%d",glove_switch);
	switch(glove_switch)
		{
		//not charging,open glove mode
		case 0:
		       glove_mode=1;
			gtp_write_config(ts);
			GTP_INFO("not charging,open glove mode,glove config write over!");
			break;
		//not charging,exit glove mode
		case 1:
			glove_mode=0;
			gtp_write_config(ts);
			GTP_INFO("not charging,exit glove mode,finger config write over!");
			break;
		//charging,open glove mode first , then  enter finger mode,and close glove mode
		case 2:
			i2c_control_buf[2] = 0x0A;
			i2c_control_buf[0] = 0x80;
			i2c_control_buf[1] = 0x46;
			ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
			if (ret < 0)
				{
					GTP_INFO("failed to set finger mode flag into 0x8046\n");
				}
			else
				{
					GTP_INFO("succeed to set finger mode flag into 0x8046\n");
				}
			i2c_control_buf[0] = 0x80;
			i2c_control_buf[1] = 0x40;
			ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
			if (ret > 0)
			      {
					GTP_INFO("charger in ,now glove had opens.GTP enter finger mode!");
			      }
			else
				{
					GTP_INFO("charger in ,now glove had open,write register 0x8040 0x0A err!");
				}
				break;
		//end charging,now glove mode had open
		case 3:
			i2c_control_buf[2] = 0x0B;

			i2c_control_buf[0] = 0x80;
			i2c_control_buf[1] = 0x46;
			ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
			if (ret < 0)
				{
					GTP_INFO("failed to set automatic switching mode flag into 0x8046\n");
				}
			else
				{
					GTP_INFO(" succeed to set automatic switching mode into 0x8046\n");
				}
			i2c_control_buf[0] = 0x80;
			i2c_control_buf[1] = 0x40;
			ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
			if (ret > 0)
				{
					GTP_INFO("end charging,now glove mode had open,GTP enter glove mode!");
				}
			else
				{
					GTP_INFO("end charging,now glove mode had open. write register 0x0B err!");
				}
				break;
		case 4:

			GTP_INFO("4 charging , glove was closed,nothing to do!");
			break;
		case 5:

			GTP_INFO("5 charging exit ,glove was closed,nothing to do!");
			break;
		//charging first,open glove,and then enter finger mode,and close glove mode
		case 6:
			glove_mode=1;
			i2c_control_buf[2] = 0x0A;
			gtp_write_config(ts);
			GTP_INFO("charging first ,glove open second,glove config write over!");
			i2c_control_buf[0] = 0x80;
			i2c_control_buf[1] = 0x46;
			ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
			if (ret < 0)
				{
				      GTP_INFO("failed to set finger mode flag into 0x8046\n");
		              }
			else
				{
					GTP_INFO(" succeed to set finger mode flag into 0x8046\n");
				}
			i2c_control_buf[0] = 0x80;
			i2c_control_buf[1] = 0x40;
			ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
			if (ret > 0)
				{
					GTP_INFO("charging first ,glove open second,glovewirte 0x8040 0A,GTP enter finger mode!");
				}
			else
				{
					GTP_INFO("charging first ,glove open second,glove, write register 0A err!");
				}
			break;
		//glove mode exit,now is charing
		case 7:
			glove_mode=0;
			gtp_write_config(ts);
			GTP_INFO("glove mode exit,now is charging,send finger_config.");

			break;

		default:

			GTP_INFO("default nothing to do!");
			break;

}
	return count;

}

/*********************test_gesture_slide_support***add mxg*********/
 static ssize_t gesture_support_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
 {
	u8 ret = 0;
 #if GTP_SLIDE_WAKEUP
	  ret = sprintf(buf, "%d",doze_status);
 #else
	  ret = sprintf(buf, "%d",0);
 #endif
	  return ret;
 }

/********************config version show*****add mxg*******/
static ssize_t gt968_send_cfg_ver_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	u8 ret = 0;
	u8 i2c_control_buf[3] = {(u8)(GTP_REG_CONFIG_DATA  >> 8), (u8)GTP_REG_CONFIG_DATA};
    ret = gtp_i2c_read(i2c_connect_client, i2c_control_buf, 3);
    if( ret < 0 )
    {
        GTP_ERROR("Read register 0x8047 failed.\n");
        return ret;
    }
	else
		{
		printk("[gt]:register 0x8047 value =%d.\n",i2c_control_buf[2]);
		}
	ret = sprintf(buf, "%02x",i2c_control_buf[2]);
	return ret;
}

/****************************config send************/
static ssize_t gt968_send_cfg_ver_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	unsigned char cfg_version = 0;
	unsigned char check_sum = 0;
	unsigned char save_cfg_version = 0;
	unsigned char save_check_sum = 0;

	u8 retry = 0;
	s32 ret = -1;
    s32 i;
   struct goodix_ts_data *goodix_ts;

   	goodix_ts = i2c_get_clientdata(i2c_connect_client);
	if(buf == NULL) {
		GTP_ERROR("buf is NULL!");
		return -ENOMEM;
	}

	cfg_version = (unsigned char)simple_strtoul(buf, NULL, 10);
	GTP_DEBUG("cfg version = %d.",cfg_version);

	if(cfg_version < 'A' || cfg_version > 'Z' ) {
		cfg_version = 0x00;
	}

	save_cfg_version = config[GTP_ADDR_LENGTH];
	save_check_sum = config[goodix_ts->gtp_cfg_len];

	config[GTP_ADDR_LENGTH] = cfg_version;
    for (i = GTP_ADDR_LENGTH; i < goodix_ts->gtp_cfg_len; i++)
    {
        check_sum += config[i];
    }
    config[goodix_ts->gtp_cfg_len] = (~check_sum) + 1;

	if (goodix_ts->use_irq) {
		gtp_irq_disable(goodix_ts);
	}
	while(retry++ < 3)
	{
		gtp_reset_guitar(goodix_ts->client, 20);
		ret = gtp_send_cfg(goodix_ts->client);
		if (ret > 0)
		{
			GTP_INFO("send new config success.");
			break;
		}
	}

	if (goodix_ts->use_irq) {
		gtp_irq_enable(goodix_ts);
	}

	config[GTP_ADDR_LENGTH] = save_cfg_version;
    config[goodix_ts->gtp_cfg_len] = save_check_sum;
	return count;
}

#ifdef CONFIG_TOUCHSCREEN_GT9XX_YL_COVER_WINDOW_SIZE
///cover for small window. linronghui&mengxiuguan. start. 2014.02.11
 static ssize_t windows_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
 {
	 ssize_t ret = 0;

	 ret = sprintf(buf, "%d\n", !!atomic_read(&gt9xx_cover_window.windows_switch));
	 return ret;
 }

 static ssize_t windows_store(struct device *dev,
					 struct device_attribute *attr,const char *buf, size_t count)
 {
	 unsigned long val;

	 if (strict_strtoul(buf, 16, &val))
		 return -EINVAL;

         if(atomic_read(&gt9xx_cover_window.windows_switch) && val == 0)
         {
             atomic_dec(&gt9xx_cover_window.windows_switch);
             printk(KERN_ERR "cover is open!\n");
         }
         else if(!atomic_read(&gt9xx_cover_window.windows_switch) && val == 1)
         {
             atomic_inc(&gt9xx_cover_window.windows_switch);
             printk(KERN_ERR "cover! small windows mode!\n");
         }
         else if(val != 0 && val != 1)
         printk(KERN_ERR "err cover window value from user space\n");

	 return count;
 }
///cover for small window. linronghui&mengxiuguan. end. 2014.02.11
#endif

static ssize_t keypad_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&gt_keypad_enable));
}

static ssize_t keypad_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val = 0;
	bool enable = 0;
	int i = 0;
	struct goodix_ts_data *ts = i2c_get_clientdata(i2c_connect_client);

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	enable = (val == 0 ? 0 : 1);
	atomic_set(&gt_keypad_enable, enable);
	if (enable) {
		for (i = 0; i < GTP_MAX_KEY_NUM; i++) {
			set_bit(touch_key_array[i], ts->input_dev->keybit);
		}
	} else {
		for (i = 0; i < GTP_MAX_KEY_NUM; i++) {
			clear_bit(touch_key_array[i], ts->input_dev->keybit);
		}
	}
	input_sync(ts->input_dev);

	return count;
}

static DEVICE_ATTR(gesture_support, S_IRUGO|S_IWUSR, gesture_support_show, NULL);
static DEVICE_ATTR(send_cfg_ver,S_IRUGO|S_IWUSR, gt968_send_cfg_ver_show, gt968_send_cfg_ver_store);

static DEVICE_ATTR(glove_switch, S_IRUGO|S_IWUSR, glove_show, glove_store);

#ifdef CONFIG_TOUCHSCREEN_GT9XX_YL_COVER_WINDOW_SIZE
static DEVICE_ATTR(windows_switch, S_IRUGO|S_IWUSR, windows_show, windows_store);
#endif

static DEVICE_ATTR(keypad_enable, S_IRUGO|S_IWUSR, keypad_enable_show, keypad_enable_store);

static struct attribute *goodix_attributes[] = {
	&dev_attr_glove_switch.attr,
#ifdef CONFIG_TOUCHSCREEN_GT9XX_YL_COVER_WINDOW_SIZE
	&dev_attr_windows_switch.attr,
#endif
	&dev_attr_send_cfg_ver.attr,
	&dev_attr_gesture_support.attr,/////add mxg
	&dev_attr_keypad_enable.attr,
	NULL
};

static struct attribute_group goodix_attribute_group = {
	.attrs = goodix_attributes
};


/******************glove switch **************************/
/*******************************************************
Function:
	Goodix touchscreen probe function.

Input:
	client:	i2c device struct.
	id:device id.

Output:
	Executive outcomes. 0---succeed.
*******************************************************/
static int  goodix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    s32 ret = -1;
    struct goodix_ts_data *ts;
    struct tw_platform_data *pdata;
    //u16 version_info;
	u16 rate;

    GTP_DEBUG_FUNC();

#ifdef CONFIG_TOUCHSCREEN_GT9XX_YL_COVER_WINDOW_SIZE

atomic_set(&gt9xx_cover_window.windows_switch, 0);   ///cover for small window. linronghui&mengxiuguan. 2014.02.11

///default value
gt9xx_cover_window.win_x_min = 0;
gt9xx_cover_window.win_y_min = 0;

gt9xx_cover_window.win_x_max = 540;
gt9xx_cover_window.win_y_max = 300;

///other porduct
#if defined(CONFIG_BOARD_CPXXXXXXX)

#endif

#endif

    //do NOT remove these output log
    GTP_INFO("GTP Driver Version:%s",GTP_DRIVER_VERSION);
    GTP_INFO("GTP Driver build@%s,%s", __TIME__,__DATE__);
    GTP_INFO("GTP I2C Address:0x%02x", client->addr);

    i2c_connect_client = client;
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        GTP_ERROR("I2C check functionality failed.");
        return -ENODEV;
    }

#ifdef CONFIG_OF
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct tw_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
		//client->dev.platform_data = pdata;

		ret = goodix_ts_parse_dt(&client->dev, pdata);
		if (ret)
		{
			dev_err(&client->dev, "Failed to parse dt\n");
			devm_kfree(&client->dev, pdata);
			return ret;
		}  
	}
	else
		pdata = client->dev.platform_data;
	#endif


    if(!pdata)
    {
        GTP_ERROR("dev platform data is null.");
        return -ENODEV;
    }

    ts = kzalloc(sizeof(*ts), GFP_KERNEL);
    if (ts == NULL)
    {
        GTP_ERROR("Alloc GFP_KERNEL memory failed.");
        return -ENOMEM;
    }

    memset(ts, 0, sizeof(*ts));
    ts->gtp_rawdiff_mode = 0;
    ts->pdata = pdata;
    ts->abs_x_max = ts->pdata->screen_x;
    ts->abs_y_max = ts->pdata->screen_y;
    ts->client = client;
    ts->client->irq = gpio_to_irq(ts->pdata->gpio_irq);
    ts->fw_error = 0;
	gtpdata=pdata;
	mutex_init(&ts->reset_mutex);

/* begin to initialize doze mode mutex by liushilong@yulong.com on 2014-11-6 17:28*/
	mutex_init(&ts->doze_mutex);
/* end to initialize doze mode mutex by liushilong@yulong.com on 2014-11-6 17:28*/

        i2c_set_clientdata(client, ts);
	 printk("[gt]:x=%d,y=%d.\n",ts->abs_x_max,ts->abs_y_max);
	if(!ts->pdata->init || ts->pdata->init() < 0)
	{
        	GTP_ERROR("GTP request IO port failed.");
        	goto exit_init_failed;
	}

	if(ts->pdata->power)
	{
		ret = ts->pdata->power(1);
		if(ret < 0)
		{
	    		GTP_ERROR("GTP power on failed.");
	        	goto exit_power_failed;
		}
	}

	if(ts->pdata->reset)
	{
		ret = ts->pdata->reset(20);
		if(ret < 0)
		{
	    		GTP_ERROR("GTP reset failed.");
	        	goto exit_reset_failed;
		}
	}

/* begin to transplant TW GTP_ESD_PROTECT code from 8675_C00, set clock click count for esd ckeck circle by liushilong@yulong.com on 2014-11-6 17:30*/
#if GTP_ESD_PROTECT
    ts->clk_tick_cnt = GTP_ESD_CHECK_CIRCLE * HZ;      // HZ: clock ticks in 1 second generated by system
    GTP_DEBUG("Clock ticks for an esd cycle: %d", ts->clk_tick_cnt);
    spin_lock_init(&ts->esd_lock);
    // ts->esd_lock = SPIN_LOCK_UNLOCKED;
#endif
/* end to transplant TW GTP_ESD_PROTECT code from 8675_C00, set clock click count for esd ckeck circle by liushilong@yulong.com on 2014-11-6 17:30*/

    ret = goodix_pinctrl_init(ts);
	if (!ret && ts->ts_pinctrl) {
		ret = goodix_pinctrl_select(ts, true);
		if (ret < 0)
			goto exit_init_panel_failed;
	}
    spin_lock_init(&ts->irq_lock);
    INIT_WORK(&ts->work, goodix_ts_work_func);

    ret = gtp_i2c_test(client);

    ret = gtp_i2c_test(client);
    if (ret < 0)
    {
        GTP_ERROR("I2C communication ERROR!");
      	goto exit_i2c_test_failed;
    }

/* begin to transplant TW GTP_ESD_PROTECT code from 8675_C00, set clock click count for esd ckeck circle by liushilong@yulong.com on 2014-11-6 17:30*/
#if GTP_ESD_PROTECT
    gtp_esd_switch(client, SWITCH_ON);
#endif
/* end to transplant TW GTP_ESD_PROTECT code from 8675_C00, set clock click count for esd ckeck circle by liushilong@yulong.com on 2014-11-6 17:30*/

#if GTP_AUTO_UPDATE
    gup_init_update_proc(ts);
#endif
    ret = gtp_init_panel(ts);
    if (ret < 0)
    {
        GTP_ERROR("GTP init panel failed.");
        ts->abs_x_max = ts->pdata->screen_x;
        ts->abs_y_max = ts->pdata->screen_y;
        ts->int_trigger_type = ts->pdata->irqflag;
    }

    ret = gtp_request_input_dev(ts);
    if (ret < 0)
    {
        GTP_ERROR("GTP request input dev failed");
      	goto exit_init_panel_failed;
    }


    GTP_DEBUG("GTP ts->client->irq = %d",ts->client->irq);
    ret = gtp_request_irq(ts);
    if (ret < 0)
    {
        GTP_INFO("GTP works in polling mode.");
    }
    else
    {
        GTP_INFO("GTP works in interrupt mode.");
    }


    ret = gtp_read_fw_cfg_version();
    if (ret < 0)
    {
        GTP_ERROR("Read version failed.");
    }

    gtp_irq_enable(ts);

#if GTP_CREATE_WR_NODE
    init_wr_node(client);
#endif
#if GTP_SLIDE_WAKEUP
    sprintf(wakeup_slide,"none");
#endif
    touchscreen_set_ops(&goodix_ops);

   gt9158_has_resumed = 1;
#if  TW_GLOVE_SWITCH

    yl_chg_status_changed();
#endif

/* begin to remove these function, these should be work on initialize period, del 5 line,liushilong@yulong.com on 2014-11-6 17:40*/
//#if GTP_ESD_PROTECT
//    INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
//    gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
//    queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, msecs_to_jiffies(GTP_ESD_CHECK_CIRCLE));
//#endif
/* end to remove these function, these should be work on initialize period, del 5 line,liushilong@yulong.com on 2014-11-6 17:40*/

	gtp_read_report_rate(client,&rate);
	ret= sysfs_create_group(&client->dev.kobj, &goodix_attribute_group);
    if (0 != ret)
    {
	dev_err(&client->dev, "%s() - ERROR: sysfs_create_group() failed: %d\n", __FUNCTION__, ret);
	sysfs_remove_group(&client->dev.kobj, &goodix_attribute_group);
    }
    else
    {
	GTP_INFO("%s sysfs_create_group() success",__func__);
    }

    printk("[gt]:gt probe over 0x%2x, GTP_ESD_PROTECT=%d\n", config[2], GTP_ESD_PROTECT);
    return 0;

exit_init_panel_failed:
exit_i2c_test_failed:
exit_reset_failed:
exit_power_failed:
	if(ts->pdata->release)
			ts->pdata->release();
exit_init_failed:
	mutex_destroy(&ts->reset_mutex);

/* begin to destory this doze_mutex, if exit_init_failed by liushilong@yulong.com on 2014-11-6 17:46*/
	mutex_destroy(&ts->doze_mutex);
/* end to destory this doze_mutex, if exit_init_failed by liushilong@yulong.com on 2014-11-6 17:46*/

	 kfree(ts);
	 if (ts->ts_pinctrl) {
		ret = goodix_pinctrl_select(ts, false);
		if (ret < 0)
			pr_err("Cannot get idle pinctrl state\n");
	}
    return ret;
}


/*******************************************************
Function:
	Goodix touchscreen driver release function.

Input:
	client:	i2c device struct.

Output:
	Executive outcomes. 0---succeed.
*******************************************************/
static int  goodix_ts_remove(struct i2c_client *client)
{
    struct goodix_ts_data *ts = i2c_get_clientdata(client);

    GTP_DEBUG_FUNC();

#if GTP_CREATE_WR_NODE
    uninit_wr_node();
#endif

#if GTP_ESD_PROTECT
    destroy_workqueue(gtp_esd_check_workqueue);
#endif

    if (ts)
    {
        if (ts->use_irq)
        {
            free_irq(client->irq, ts);
        }
        else
        {
            hrtimer_cancel(&ts->timer);
        }
    }

    GTP_INFO("GTP driver is removing...");
    i2c_set_clientdata(client, NULL);
    input_unregister_device(ts->input_dev);
    input_free_device(ts->input_dev);
    if(ts->pdata->release)
	ts->pdata->release();
    kfree(ts);

    return 0;
}
/*******************************************************
Function:
	Early suspend function.

Input:
	h:early_suspend struct.

Output:
	None.
*******************************************************/
static int goodix_ts_disable(struct input_dev *in_dev)
{
    struct goodix_ts_data *ts;
    s8 ret = -1;
    ts = i2c_get_clientdata(i2c_connect_client);

    GTP_DEBUG_FUNC();

    if(ts->gtp_is_suspend)
		return 0;

#if GTP_ESD_PROTECT
    ts->gtp_is_suspend = 1;
/* begin to the esd switch will be switch off when gtp is suspend by liushilong@yulong.com on 2014-11-6 17:47*/
    gtp_esd_switch(ts->client, SWITCH_OFF);
/* end to the esd switch will be switch off when gtp is suspend by liushilong@yulong.com on 2014-11-6 17:47*/
#endif

#if GTP_SLIDE_WAKEUP
    sprintf(wakeup_slide,"none");

    if(support_gesture & TW_SUPPORT_GESTURE_IN_ALL)
    {
        printk(KERN_ERR "zzzz: tw enter slide wakeup\n");
        ret = gtp_enter_doze(ts);
    }
    else

    {
        printk(KERN_ERR "tw enter sleep\n");
        sprintf(wakeup_slide,"none");
#endif
        if (ts->use_irq)
        {
            gtp_irq_disable(ts);
        }
        else
        {
            hrtimer_cancel(&ts->timer);
        }
        ret = gtp_enter_sleep(ts);

        if(ts->pdata->suspend)
	 ts->pdata->suspend();
#if GTP_SLIDE_WAKEUP

    }
#endif
    if (ret < 0)
    {
        GTP_ERROR("GTP early suspend failed.");
    }

    gt9158_has_resumed = 0;

    msleep(60);   //linronghui
    ts->gtp_is_suspend = 1;
    return ret;
}

/*******************************************************
Function:
	Late resume function.

Input:
	h:early_suspend struct.

Output:
	None.
*******************************************************/
static int goodix_ts_enable(struct input_dev *in_dev)
{
    struct goodix_ts_data *ts;
    s8 ret = -1;

    gt9158_has_resumed = 1;

    ts = i2c_get_clientdata(i2c_connect_client);

    GTP_DEBUG_FUNC();

    if(!ts->gtp_is_suspend)
		return 0;


    if(ts->pdata->resume)
        ts->pdata->resume();

#if GTP_SLIDE_WAKEUP
//    if(doze_status != DOZE_WAKEUP) {
//         sprintf(wakeup_slide,"none");
//    }

/* begin to add mutex for doze mode liushilong@yulong.com on 2014-11-6 17:49*/
	mutex_lock(&ts->doze_mutex);
    doze_status = DOZE_DISABLED;
    mutex_unlock(&ts->doze_mutex);
/* end to add mutex for doze mode liushilong@yulong.com on 2014-11-6 17:49*/

    gtp_enable_irq_wake(ts, 0);
#endif

    ret = gtp_wakeup_sleep(ts);
    if (ret < 0)
    {
        GTP_ERROR("GTP later resume failed.");
    }

#if  TW_GLOVE_SWITCH
    if (glove_switch == MODE_GLOVE) {
    	yl_chg_status_changed();
    	}
#endif

    if (ts->use_irq)
    {
        gtp_irq_enable(ts);
    }
    else
    {
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
    }

    ts->gtp_is_suspend = 0;

/* begin to modify delayed work instead of gtp_esd_switch, it should switch on esd protct when late resume by liushilong@yulong.com on 2014-11-6 17:49*/
#if GTP_ESD_PROTECT
    gtp_esd_switch(ts->client, SWITCH_ON);
#endif
/* end to modify delayed work instead of gtp_esd_switch, it should switch on esd protct when late resume by liushilong@yulong.com on 2014-11-6 17:49*/
    return ret;
}

/* begin to transplant TW GTP_ESD_PROTECT code from 8675_C00, when esd test is fail, it should restart the mobile phone
         add: gtp_i2c_read_no_rst() for read i2c client
              gtp_i2c_write_no_rst() for write i2c client
              gtp_esd_switch() for switch on/off esd protect
              gtp_init_ext_watchdog() for set watchdog
         modify:gtp_esd_check_func() by liushilong@yulong.com on 2014-11-7 14:00 */
#if GTP_ESD_PROTECT
s32 gtp_i2c_read_no_rst(struct i2c_client *client, u8 *buf, s32 len)
{
    struct i2c_msg msgs[2];
    s32 ret=-1;
    s32 retries = 0;
	int retriesNum = 5;

    GTP_DEBUG_FUNC();

    msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = client->addr;
    msgs[0].len   = GTP_ADDR_LENGTH;
    msgs[0].buf   = &buf[0];
    //msgs[0].scl_rate = 300 * 1000;    // for Rockchip, etc.

    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = client->addr;
    msgs[1].len   = len - GTP_ADDR_LENGTH;
    msgs[1].buf   = &buf[GTP_ADDR_LENGTH];
    //msgs[1].scl_rate = 300 * 1000;

    while(retries < retriesNum)
    {
        ret = i2c_transfer(client->adapter, msgs, 2);
        if(ret == 2)
		{
			break;
		}
        retries++;
    }
    if ((retries >= retriesNum))
    {
        GTP_ERROR("I2C Read: 0x%04X, %d bytes failed, errcode: %d!", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
    }
    return ret;
}
s32 gtp_i2c_write_no_rst(struct i2c_client *client,u8 *buf,s32 len)
{
    struct i2c_msg msg;
    s32 ret = -1;
    s32 retries = 0;
	int retriesNum = 5;

    GTP_DEBUG_FUNC();

    msg.flags = !I2C_M_RD;
    msg.addr  = client->addr;
    msg.len   = len;
    msg.buf   = buf;
    //msg.scl_rate = 300 * 1000;    // for Rockchip, etc

    while(retries < retriesNum)
    {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret == 1)break;
        retries++;
    }
    if((retries >= retriesNum))
    {
        GTP_ERROR("I2C Write: 0x%04X, %d bytes failed, errcode: %d!", (((u16)(buf[0] << 8)) | buf[1]), len-2, ret);
    }
    return ret;
}
/*******************************************************
Function:
    switch on & off esd delayed work
Input:
    client:  i2c device
    on:      SWITCH_ON / SWITCH_OFF
Output:
    void
*********************************************************/
void gtp_esd_switch(struct i2c_client *client, s32 on)
{
    struct goodix_ts_data *ts;

    ts = i2c_get_clientdata(client);
    spin_lock(&ts->esd_lock);

    if (SWITCH_ON == on)     // switch on esd
    {
        if (!ts->esd_running)
        {
            ts->esd_running = 1;
            spin_unlock(&ts->esd_lock);
            GTP_INFO("Esd started");
            queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, ts->clk_tick_cnt);
        }
        else
        {
            spin_unlock(&ts->esd_lock);
        }
    }
    else    // switch off esd
    {
        if (ts->esd_running)
        {
            ts->esd_running = 0;
            spin_unlock(&ts->esd_lock);
            GTP_INFO("Esd cancelled");
            cancel_delayed_work_sync(&gtp_esd_check_work);
        }
        else
        {
            spin_unlock(&ts->esd_lock);
        }
    }
}

/*******************************************************
Function:
    Initialize external watchdog for esd protect
Input:
    client:  i2c device.
Output:
    result of i2c write operation.
        1: succeed, otherwise: failed
*********************************************************/
static s32 gtp_init_ext_watchdog(struct i2c_client *client)
{
    u8 opr_buffer[3] = {0x80, 0x41, 0xAA};
    GTP_DEBUG("[Esd]Init external watchdog");
    return gtp_i2c_write_no_rst(client, opr_buffer, 3);
}

/*******************************************************
Function:
    Esd protect function.
    External watchdog added by meta, 2013/03/07
Input:
    work: delayed work
Output:
    None.
*******************************************************/
static void gtp_esd_check_func(struct work_struct *work)
{
    s32 i = 0;
    s32 ret = -1;
    struct goodix_ts_data *ts = NULL;
    u8 esd_buf[5] = {0x80, 0x40};
	s32 retriesNum = 3;

    GTP_DEBUG_FUNC();

    ts = i2c_get_clientdata(i2c_connect_client);


	if (ts->gtp_is_suspend || ts->enter_update)
    {
        GTP_INFO("Esd suspended or IC update firmware!");
        return;
    }


    for (i = 0; i < retriesNum; i++)
    {
        ret = gtp_i2c_read_no_rst(ts->client, esd_buf, 4);

        GTP_DEBUG("[Esd]0x8040 = 0x%02X, 0x8041 = 0x%02X", esd_buf[2], esd_buf[3]);
        if ((ret < 0))
        {
            // IIC communication problem
            continue;
        }
        else
        {
            if ((esd_buf[2] == 0xAA) || (esd_buf[3] != 0xAA))
            {
                // IC works abnormally..
                u8 chk_buf[4] = {0x80, 0x40};

                gtp_i2c_read_no_rst(ts->client, chk_buf, 4);

                GTP_DEBUG("[Check]0x8040 = 0x%02X, 0x8041 = 0x%02X", chk_buf[2], chk_buf[3]);

                if ((chk_buf[2] == 0xAA) || (chk_buf[3] != 0xAA))
                {
                    i = retriesNum;
                    break;
                }
                else
                {
                    continue;
                }
            }
            else
            {
                // IC works normally, Write 0x8040 0xAA, feed the dog
                esd_buf[2] = 0xAA;
                gtp_i2c_write_no_rst(ts->client, esd_buf, 3);
                break;
            }
        }
    }
    if (i >= retriesNum)
    {

        GTP_ERROR("IC working abnormally! Process reset guitar.");
        esd_buf[0] = 0x42;
        esd_buf[1] = 0x26;
        esd_buf[2] = 0x01;
        esd_buf[3] = 0x01;
        esd_buf[4] = 0x01;
        gtp_i2c_write_no_rst(ts->client, esd_buf, 5);
        msleep(50);
        gtp_reset_guitar(ts->client, 50);
        msleep(50);
        gtp_send_cfg(ts->client);
    }

    if(!ts->gtp_is_suspend)
    {
        queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, ts->clk_tick_cnt);
    }
    else
    {
        GTP_INFO("Esd suspended!");
    }
    return;
}
#endif
/* end to transplant TW GTP_ESD_PROTECT code from 8675_C00, when esd test is fail, it should restart the mobile phone
         add: gtp_i2c_read_no_rst() for read i2c client
              gtp_i2c_write_no_rst() for write i2c client
              gtp_esd_switch() for switch on/off esd protect
              gtp_init_ext_watchdog() for set watchdog
         modify:gtp_esd_check_func() by liushilong@yulong.com on 2014-11-7 14:00 */

static const struct i2c_device_id goodix_ts_id[] = {
    { GTP_I2C_NAME, 0 },
    { }
};

#ifdef CONFIG_OF
static struct of_device_id goodix_match_table[] = {
	{ .compatible = "Goodix,Goodix-TS",},
	{ },
};
#else
#define goodix_match_table NULL
#endif


static struct i2c_driver goodix_ts_driver = {
    .driver = {
        .name     = GTP_I2C_NAME,
        .owner    = THIS_MODULE,
        .of_match_table = goodix_match_table,
#if defined(CONFIG_PM) && !defined(CONFIG_FB)
	.pm   = &goodix_pm_ops,
#endif
   },
    .probe      = goodix_ts_probe,
    .remove     = goodix_ts_remove,
    .id_table   = goodix_ts_id,
};

/*******************************************************
Function:
	Driver Install function.
Input:
  None.
Output:
	Executive Outcomes. 0---succeed.
********************************************************/
static int __init goodix_ts_init(void)
{
    s32 ret;

    GTP_DEBUG_FUNC();
    GTP_INFO("GTP driver install.");
    goodix_wq = create_singlethread_workqueue("goodix_wq");
    if (!goodix_wq)
    {
        GTP_ERROR("Creat workqueue failed.");
        return -ENOMEM;
    }

/* begin to transplant TW GTP_ESD_PROTECT code from 8675_C00 when initialize goodix touch panel, it should INIT and check esd by liushilong@yulong.com on 2014-11-6 17:24*/
#if GTP_ESD_PROTECT
    INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
    gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
#endif
/* end to transplant TW GTP_ESD_PROTECT code from 8675_C00 when initialize goodix touch panel, it should INIT and check esd by liushilong@yulong.com on 2014-11-6 17:24*/
    ret = i2c_add_driver(&goodix_ts_driver);
    return ret;
}

/*******************************************************
Function:
	Driver uninstall function.
Input:
  None.
Output:
	Executive Outcomes. 0---succeed.
********************************************************/
static void __exit goodix_ts_exit(void)
{
    GTP_DEBUG_FUNC();
    GTP_INFO("GTP driver exited.");
    i2c_del_driver(&goodix_ts_driver);
    if (goodix_wq)
    {
        destroy_workqueue(goodix_wq);
    }
}

late_initcall(goodix_ts_init);
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
