/*
 * This file is part of the AP3426, AP3212C and AP3216C sensor driver.
 * AP3426 is combined proximity and ambient light sensor.
 * AP3216C is combined proximity, ambient light sensor and IRLED.
 *
 * Contact: John Huang <john.huang@dyna-image.com>
 *	    Templeton Tsai <templeton.tsai@dyna-image.com>
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *
 * Filename: ap3426.c
 *
 * Summary:
 *	AP3426 device driver.
 *
 * Modification History:
 * Date     By       Summary
 * -------- -------- -------------------------------------------------------
 * 02/02/12 YC       1. Modify irq function to seperate two interrupt routine.
 *                   2. Fix the index of reg array error in em write.
 * 02/22/12 YC       3. Merge AP3426 and AP3216C into the same driver. (ver 1.8)
 * 03/01/12 YC       Add AP3212C into the driver. (ver 1.8)
 * 07/25/14 John	  Ver.2.1 , ported for Nexus 7
 * 08/21/14 Templeton AP3426 Ver 1.0, ported for Nexus 7
 * 09/24/14 kevin    Modify for Qualcomm8x10 to support device tree
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/string.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>
#include "ap3426.h"
#include <linux/regulator/consumer.h>
#include <linux/ioctl.h>
#include <linux/atomic.h>

#define AP3426_DRV_NAME		"ap3426"
#define DRIVER_VERSION		"1.0"

#define PL_TIMER_DELAY 200

/* Proximity high threshold, the px value ranges: 0 (FAR) to 1023 (NEAR) */
// In ps sensor hysteresis mode (default), object distance > PX_HIGH_THRESHOLD
// triggers distance 0 (near); distance < PX_LOW_THRESHOLD triggers distance 1 (far)
#define PX_HIGH_THRESHOLD 450
#define PX_LOW_THRESHOLD 300

/* misc define */
#define MIN_ALS_POLL_DELAY_MS	110


#define AP3426_VDD_MIN_UV	2000000
#define AP3426_VDD_MAX_UV	3300000
#define AP3426_VIO_MIN_UV	1750000
#define AP3426_VIO_MAX_UV	1950000

/*
 * Un-comment the 3 lines within the #ifndef...#endif to enable pr_debug()
 * locally. pr_debug() is used by DBG macros; ex: ALS_DBG() and PS_DB().
 */
#ifndef CONFIG_DYNAMIC_DEBUG
// #define CONFIG_DYNAMIC_DEBUG
// #undef pr_debug
// #define pr_debug(args...) printk(args)
#endif

// #define PS_POLLING_DEBUG 1           /* Uncomment to enable polling PS debug; ... */
                                        /* ... normally PS is only interrupt driven */

// #define ALS_DEBUG                    /* Uncomment for additional ALS Debug */

/*
 * DEBUG, INFO and ERR Logging are normally defined below and enabled.
 * pr_debug() is enabled above or if the kernel configs CONFIG_DYNAMIC_DEBUG.
 *
 * FIXME:
 *       Currently not indenting when debug is enabled via kernel config file.
 *       Low priority issue, will fix when it's convenient.
 *
 * Since pr_debug() is by default a null function the gcc optimizer will normally remove the
 *       {if ... pr_debug}
 * fragments in production builds.
 *
 * The indent logic is ifdef's out if CONFIG_DYNAMIC_DEBUG isn't defined.
 *
 * Keeping this driver similar to the Tomato proximity driver;
 *     See: drivers/sensors/alsprox/pa12200001/pa12200001.c
 * where it appears the debug code is enabled for a CONFIG_DYNAMIC_DEBUG kernel.
 *
 * These can be disabled below to disable logging.
 */
#define PS_debug        1		/* Increase to 2 for more details */
#define PS_info         1
#define PS_err          1

#define ALS_debug       2		/* Increase to 2 for more details */
#define ALS_info        1
#define ALS_err         1

#define MUTEX_debug     0               /* Usually not too interesting */

#if defined(CONFIG_DYNAMIC_DEBUG)
static atomic_t log_indent = ATOMIC_INIT(0);	/* Log Indent */

static inline int indent(void)  { return((int) log_indent.counter); }
static inline void ind(void)    { pr_debug("%*s",  (indent() * 2), " "); }
static inline void inc(void)    { atomic_inc(&log_indent); }
static inline void dec(void)    { atomic_dec_if_positive(&log_indent); }

#else
#define indent() 0
#define ind()
#define inc()
#define dec()
#endif

/*
 * Indent Logging:
 *
 *    fun1(args) {
 *        fun2(args) {
 *            return(rv);
 *        }
 *    }
 */
#define ENTRY(fmt, args...)   {                      \
        if (indent() == 0) {                         \
                pr_debug("\n");                      \
        }                                            \
        ind();                                       \
        pr_debug("%s(",   __func__);                 \
        pr_debug(fmt"): {\n", args);                 \
        inc();                                       \
}

#define RETURN(fmt, args...) {                       \
        ind();                                       \
        pr_debug("%s: return(",   __func__);         \
        pr_debug(fmt");\n", args);                   \
        dec();                                       \
        ind();                                       \
        pr_debug("}\n");                             \
}
#define RETURN_VOID() {                              \
        dec();                                       \
        ind();                                       \
        pr_debug("}\n");                             \
}

#define DBG(args...)    { ind();  pr_debug("%s: ",  __func__);  pr_debug(args); }
#define ERR(args...)    { ind();    pr_err("%s: ",  __func__);  pr_err(args);   }
#define INFO(args...)   { ind();   pr_info("%s: ",  __func__);  pr_info(args);  }

/* Proximity Sensor Logging */
#define PS_ENTRY(args...)         { if (PS_debug)        ENTRY(args);     }
#define PS_RETURN(args...)        { if (PS_debug)        RETURN(args);    }
#define PS_RETURN_VOID()          { if (PS_debug)        RETURN_VOID();   }
#define PS_DBG(args...)           { if (PS_debug)        DBG(args);       }
#define PS_DBG2(args...)          { if (PS_debug >= 2)   DBG(args);       }
#define PS_ERR(args...)           { if (PS_err)          ERR(args);       }
#define PS_INFO(args...)          { if (PS_info)         INFO(args);      }

/* Ambient Light Sensor Logging */
#define ALS_ENTRY(args...)        { if (ALS_debug)       ENTRY(args);     }
#define ALS_RETURN(args...)       { if (ALS_debug)       RETURN(args);    }
#define ALS_RETURN_VOID()         { if (ALS_debug)       RETURN_VOID();   }
#define ALS_DBG(args...)          { if (ALS_debug)       DBG(args);       }
#define ALS_DBG2(args...)         { if (ALS_debug >= 2)  DBG(args);       }
#define ALS_ERR(args...)          { if (ALS_err)         ERR(args);       }
#define ALS_INFO(args...)         { if (ALS_info)        INFO(args);      }

/* Mutex Check Logging */
#define MUTEX_ENTRY(args...)      { if (MUTEX_debug)     ENTRY(args);     }
#define MUTEX_RETURN(args...)     { if (MUTEX_debug)     RETURN(args);    }
#define MUTEX_RETURN_VOID()       { if (MUTEX_debug)     RETURN_VOID();   }
#define MUTEX_DBG(args...)        { if (MUTEX_debug)     DBG(args);       }


#define DI_AUTO_CAL

#ifdef DI_AUTO_CAL
       #define DI_PS_CAL_THR_MAX 500
       #define DI_PS_CAL_THR_EXPECTED 150
       #define DI_PS_CAL_THR_MIN 0
#endif

static void pl_timer_callback(unsigned long pl_data);
static int ap3426_power_ctl(struct ap3426_data *data, bool on);
static int ap3426_power_init(struct ap3426_data*data, bool on);

static struct ap3426_data *private_pl_data = NULL;
// AP3426 register
static u8 ap3426_reg_to_idx_array[AP3426_MAX_REG_NUM] = {
	0,	1,	2,	0xff,	0xff,	0xff,	3,	0xff,
	0xff,	0xff,	4,	5,	6,	7,	8,	9,
	10,	0xff,	0xff,	0xff,	0xff,	0xff,	0xff,	0xff,
	0xff,	0xff,	11,	12,	13,	14,	0xff,	0xff,
	15,	16,	17,	18,	19,	20,	21,	0xff,
	22,	23,	24,	25,	26,	27         //20-2f
};
static u8 ap3426_reg[AP3426_NUM_CACHABLE_REGS] = {
	0x00,0x01,0x02,0x06,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,
	0x10,0x1A,0x1B,0x1C,0x1D,0x20,0x21,0x22,0x23,0x24,
	0x25,0x26,0x28,0x29,0x2A,0x2B,0x2C,0x2D
};
static u8 *reg_array = ap3426_reg;
// AP3426 range
static int ap3426_range[4] = {32768,8192,2048,512};
//static u16 ap3426_threshole[8] = {28,444,625,888,1778,3555,7222,0xffff};

static int *range = ap3426_range;

static int als_calibration = 320;
static int misc_ps_opened = 0;
static int misc_als_opened = 0;
#ifdef CONFIG_AP3426_HEARTBEAT_SENSOR
static int misc_ht_opened = 0;
#endif
struct regulator *vdd;
struct regulator *vio;
bool power_enabled;
/*
 * register access helpers
 */

/*
fixed for msm8916 kevindang20141010
*/
static struct sensors_classdev sensors_light_cdev = { 
	.name = "light", 
	.vendor = "DI", 
	.version = 1, 
	.handle = SENSORS_LIGHT_HANDLE, 
	.type = SENSOR_TYPE_LIGHT, 
	.max_range = "6500", 
	.resolution = "0.0625", 
	.sensor_power = "0.09", 
	.min_delay = 0,	/* us */ 
	.fifo_reserved_event_count = 0, 
	.fifo_max_event_count = 0, 
	.enabled = 0, 
	.delay_msec = 200, 
	.sensors_enable = NULL, 
	.sensors_poll_delay = NULL, 
}; 


static struct sensors_classdev sensors_proximity_cdev = { 
	.name = "proximity", 
	.vendor = "DI", 
	.version = 1, 
	.handle = SENSORS_PROXIMITY_HANDLE, 
	.type = SENSOR_TYPE_PROXIMITY, 
	.max_range = "5.0", 
	.resolution = "5.0", 
	.sensor_power = "0.1", 
	.min_delay = 0, 
	.fifo_reserved_event_count = 0, 
	.fifo_max_event_count = 0, 
	.enabled = 0, 
	.delay_msec = 200, 
	.sensors_enable = NULL, 
	.sensors_poll_delay = NULL, 
}; 

static int __ap3426_read_reg(struct i2c_client *client,
	u32 reg, u8 mask, u8 shift)
{
    struct ap3426_data *data = i2c_get_clientdata(client);

    return (data->reg_cache[ap3426_reg_to_idx_array[reg]] & mask) >> shift;
}

static int __ap3426_write_reg(struct i2c_client *client,
	u32 reg, u8 mask, u8 shift, u8 val)
{
    struct ap3426_data *data = i2c_get_clientdata(client);
    int ret = 0;
    u8 tmp;

    tmp = data->reg_cache[ap3426_reg_to_idx_array[reg]];
    tmp &= ~mask;
    tmp |= val << shift;

    ret = i2c_smbus_write_byte_data(client, reg, tmp);
    if (!ret)
	data->reg_cache[ap3426_reg_to_idx_array[reg]] = tmp;

    return ret;
}

/*
 * internally used functions
 */

/* range */
static int ap3426_get_range(struct i2c_client *client)
{
    u8 idx = __ap3426_read_reg(client, AP3426_REG_ALS_CONF,
	    AP3426_ALS_RANGE_MASK, AP3426_ALS_RANGE_SHIFT); 
    return range[idx];
}

static int ap3426_set_range(struct i2c_client *client, int range)
{
    return __ap3426_write_reg(client, AP3426_REG_ALS_CONF,
	    AP3426_ALS_RANGE_MASK, AP3426_ALS_RANGE_SHIFT, range);
}

/* mode */
static int ap3426_get_mode(struct i2c_client *client)
{
    int ret;

    ret = __ap3426_read_reg(client, AP3426_REG_SYS_CONF,
	    AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT);
    return ret;
}

static int ap3426_set_mode(struct i2c_client *client, int mode)
{
    int ret;

    misc_ps_opened = mode & AP3426_SYS_ALS_ENABLE;
    misc_als_opened = mode & AP3426_SYS_PS_ENABLE;

    ret = __ap3426_write_reg(client, AP3426_REG_SYS_CONF,
	    AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, mode);
    return ret;
}

/* ALS low threshold */
static int ap3426_get_althres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __ap3426_read_reg(client, AP3426_REG_ALS_THDL_L,
	    AP3426_REG_ALS_THDL_L_MASK, AP3426_REG_ALS_THDL_L_SHIFT);
    msb = __ap3426_read_reg(client, AP3426_REG_ALS_THDL_H,
	    AP3426_REG_ALS_THDL_H_MASK, AP3426_REG_ALS_THDL_H_SHIFT);
    return ((msb << 8) | lsb);
}

static int ap3426_set_althres(struct i2c_client *client, int val)
{

    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3426_REG_ALS_THDL_L_MASK;

    err = __ap3426_write_reg(client, AP3426_REG_ALS_THDL_L,
	    AP3426_REG_ALS_THDL_L_MASK, AP3426_REG_ALS_THDL_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3426_write_reg(client, AP3426_REG_ALS_THDL_H,
	    AP3426_REG_ALS_THDL_H_MASK, AP3426_REG_ALS_THDL_H_SHIFT, msb);

    return err;
}

/* ALS high threshold */
static int ap3426_get_ahthres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __ap3426_read_reg(client, AP3426_REG_ALS_THDH_L,
	    AP3426_REG_ALS_THDH_L_MASK, AP3426_REG_ALS_THDH_L_SHIFT);
    msb = __ap3426_read_reg(client, AP3426_REG_ALS_THDH_H,
	    AP3426_REG_ALS_THDH_H_MASK, AP3426_REG_ALS_THDH_H_SHIFT);
    return ((msb << 8) | lsb);
}

static int ap3426_set_ahthres(struct i2c_client *client, int val)
{
    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3426_REG_ALS_THDH_L_MASK;

    err = __ap3426_write_reg(client, AP3426_REG_ALS_THDH_L,
	    AP3426_REG_ALS_THDH_L_MASK, AP3426_REG_ALS_THDH_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3426_write_reg(client, AP3426_REG_ALS_THDH_H,
	    AP3426_REG_ALS_THDH_H_MASK, AP3426_REG_ALS_THDH_H_SHIFT, msb);

    return err;
}

/* PX low threshold */
static int ap3426_get_plthres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __ap3426_read_reg(client, AP3426_REG_PS_THDL_L,
	    AP3426_REG_PS_THDL_L_MASK, AP3426_REG_PS_THDL_L_SHIFT);
    msb = __ap3426_read_reg(client, AP3426_REG_PS_THDL_H,
	    AP3426_REG_PS_THDL_H_MASK, AP3426_REG_PS_THDL_H_SHIFT);
    return ((msb << 8) | lsb);
}

static int ap3426_set_plthres(struct i2c_client *client, int val)
{
    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3426_REG_PS_THDL_L_MASK;

    err = __ap3426_write_reg(client, AP3426_REG_PS_THDL_L,
	    AP3426_REG_PS_THDL_L_MASK, AP3426_REG_PS_THDL_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3426_write_reg(client, AP3426_REG_PS_THDL_H,
	    AP3426_REG_PS_THDL_H_MASK, AP3426_REG_PS_THDL_H_SHIFT, msb);

    return err;
}

/* PX high threshold */
static int ap3426_get_phthres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __ap3426_read_reg(client, AP3426_REG_PS_THDH_L,
	    AP3426_REG_PS_THDH_L_MASK, AP3426_REG_PS_THDH_L_SHIFT);
    msb = __ap3426_read_reg(client, AP3426_REG_PS_THDH_H,
	    AP3426_REG_PS_THDH_H_MASK, AP3426_REG_PS_THDH_H_SHIFT);
    return ((msb << 8) | lsb);
}

static int ap3426_set_phthres(struct i2c_client *client, int val)
{
    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3426_REG_PS_THDH_L_MASK;

    err = __ap3426_write_reg(client, AP3426_REG_PS_THDH_L,
	    AP3426_REG_PS_THDH_L_MASK, AP3426_REG_PS_THDH_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3426_write_reg(client, AP3426_REG_PS_THDH_H,
	    AP3426_REG_PS_THDH_H_MASK, AP3426_REG_PS_THDH_H_SHIFT, msb);

    return err;
}

static int ap3426_get_adc_value(struct i2c_client *client)
{
	unsigned int lsb, msb, val;

	ALS_ENTRY("client:%p", client);

	val = lsb = i2c_smbus_read_byte_data(client, AP3426_REG_ALS_DATA_LOW);

	if (lsb < 0) {
		ALS_ERR("lsb:%d < 0\n", lsb);
		goto done;
	}

	val = msb = i2c_smbus_read_byte_data(client, AP3426_REG_ALS_DATA_HIGH);

	if (msb < 0) {
		ALS_ERR("msb:%d < 0\n", msb);
		goto done;
	}

#ifdef ALS_DEBUG
	{
		unsigned int tmp, tmp2, range_tmp;;

		range_tmp = ap3426_get_range(client);
		tmp = (((msb << 8) | lsb) * range_tmp) >> 16;
		tmp2 = tmp * als_calibration / 100;

		ALS_DBG("tmp2:%d = tmp:%d * als_calibration:%d\n",
			 tmp2,     tmp,     als_calibration);
	}
#endif

	val = (msb << 8) | lsb;

done:
	ALS_RETURN("val:0x%x:%d = (msb:%x << 8) || lsb:%x",
		    val, val,      msb,            lsb);

	return val;
}

/* Get PS Distance: Near:0 or Far:1 */
static int ap3426_get_object(struct i2c_client *client)
{
    int val;
    int rv;

    PS_ENTRY("client:%p", client);

    val = i2c_smbus_read_byte_data(client, AP3426_OBJ_COMMAND);

    PS_DBG2("val:0X%x &= AP3426_OBJ_MASK:0x%x\n", val, AP3426_OBJ_MASK);
    val &= AP3426_OBJ_MASK;

    rv = !(val >> AP3426_OBJ_SHIFT);
    PS_DBG2("rv = 0x%x = !(val:0x%x >> AP3426_OBJ_SHIFT):%d;\n",
             rv,           val,        AP3426_OBJ_SHIFT);

    PS_RETURN("rv:%d", rv);
    return(rv);
}

static int ap3426_get_intstat(struct i2c_client *client)
{
	int val;
	int rv;

	PS_ENTRY("client:%p", client);
	val = i2c_smbus_read_byte_data(client, AP3426_REG_SYS_INTSTATUS);
	val &= AP3426_REG_SYS_INT_MASK;

	rv = val >> AP3426_REG_SYS_INT_SHIFT;

	PS_RETURN("rv:%d", rv);
	return(rv);
}

static int ap3426_get_px_value(struct i2c_client *client)
{
    int lsb, msb;
    int rv;

    PS_ENTRY("client:%p", client);

    rv = lsb = i2c_smbus_read_byte_data(client, AP3426_REG_PS_DATA_LOW);

    if (lsb < 0) {
	PS_ERR("lsb < 0\n");
	goto done;
    }
    PS_DBG2("IR = 0X%x = lsb\n", (u32)(lsb));
    rv = msb = i2c_smbus_read_byte_data(client, AP3426_REG_PS_DATA_HIGH);

    if (msb < 0) {
	PS_ERR("msb < 0\n");
	goto done;
    }
    // PS_DBG("IR = 0X%x = msb\n", (u32)(msb));
    rv = (u32)(((msb & AL3426_REG_PS_DATA_HIGH_MASK) << 8) | (lsb & AL3426_REG_PS_DATA_LOW_MASK));

done:
    PS_RETURN("rv:0X%x:%d", rv, rv);
    return(rv);
}


static inline void ap3426_disable_ps_and_als_interrupts(struct i2c_client *client)
{
	i2c_smbus_write_byte_data(client, AP3426_REG_SYS_INTCTRL, 0);
}

static inline void ap3426_disable_ps_interrupts(struct i2c_client *client)
{
	int val;;

	val = i2c_smbus_read_byte_data(client, AP3426_REG_SYS_INTCTRL);
	val &= ~0x80;
	i2c_smbus_write_byte_data(client, AP3426_REG_SYS_INTCTRL, val);
}

static inline void ap3426_enable_ps_interrupts(struct i2c_client *client)
{
	int val;

	val = i2c_smbus_read_byte_data(client, AP3426_REG_SYS_INTCTRL);
	val |= 0x80;
	i2c_smbus_write_byte_data(client, AP3426_REG_SYS_INTCTRL, val);
}

/* Power on PS and IR Part of Chip */
static inline int ap3426_power_on_ps(struct i2c_client *client)
{
	int err;

	err = __ap3426_write_reg(client, AP3426_REG_SYS_CONF,
				AP3426_REG_SYS_INT_PMASK, 1, 1);
	return(err);
}

static inline int ap3426_power_off_ps(struct i2c_client *client)
{
	int err;

	err = __ap3426_write_reg(client, AP3426_REG_SYS_CONF,
				AP3426_REG_SYS_INT_PMASK, 1, 0);
	return(err);
}

/* Power on ALS Part of Chip */
static inline int ap3426_power_on_als(struct i2c_client *client)
{
	int err;

	err = __ap3426_write_reg(client, AP3426_REG_SYS_CONF,
				AP3426_REG_SYS_INT_AMASK, 0, 1);
	return(err);
}

static inline int ap3426_power_off_als(struct i2c_client *client)
{
	int err;

	err = __ap3426_write_reg(client, AP3426_REG_SYS_CONF,
				AP3426_REG_SYS_INT_AMASK, 0, 0);
	return(err);
}

static int ap3426_ps_enable(struct ap3426_data *ps_data, int enable)
{
	int32_t ret;
	int pxvalue;
	int distance;
	struct i2c_client *client = ps_data->client;

	PS_ENTRY("ps_data:%p, enable:%d", ps_data, enable);

	if (ps_data->suspended) {
		printk(KERN_ERR "%s: suspended! ret = -EINVAL:%d\n", __func__,
				                       EINVAL);
		ret = EINVAL;
		goto done;
	}
	if (misc_ps_opened == enable) {
		PS_DBG("misc_ps_opened == enabe\n");
		ret = 0;
		goto done;
	}
	if (enable) {
		ret = ap3426_power_on_ps(client);
		ap3426_enable_ps_interrupts(client);
	} else {
		ap3426_disable_ps_interrupts(client);
		ret = ap3426_power_off_ps(client);
	}
	if (ret < 0) {
		printk("ret:%d = ap3426_power_on/off_ps(); Error\n", ret);
	} else {
		misc_ps_opened = enable;
	}

	// Ensure psensor wakeup the system
	msleep(50);
	if (misc_ps_opened){
		distance = ap3426_get_object(client);
		if (distance != 1) {
			PS_ERR("Unexpected distance:%d upon enable\n", distance);
		}
		input_report_abs(ps_data->psensor_input_dev, ABS_DISTANCE, 1);
		input_sync(ps_data->psensor_input_dev);
		wake_lock_timeout(&ps_data->ps_wakelock, 2*HZ);
		pxvalue = ap3426_get_px_value(client);
		PS_DBG("pxvalue:%d, distance:%d\n", pxvalue, distance);

#ifdef PS_POLLING_DEBUG
		ret = mod_timer(&ps_data->pl_timer, jiffies + msecs_to_jiffies(PL_TIMER_DELAY));
#endif
	}
done:
	PS_RETURN("ret:%d", ret);
	return ret;
}

static int ap3426_als_enable(struct ap3426_data *ps_data, int enable)
{
	int32_t ret;
	struct i2c_client *client = ps_data->client;

	ALS_ENTRY("ps_data:%p, enable:%d", ps_data, enable);

	ALS_DBG("misc_als_opened:%d\n", misc_als_opened);


        if (ps_data->suspended) {
                printk(KERN_ERR "%s: suspended! ret = EINVAL:%d; misc_als_opened:%d\n",
                        __func__,                     EINVAL,    misc_als_opened);

                ret = -EINVAL;
                goto done;
        }
	if (misc_als_opened == enable) {
		 ALS_DBG("misc_als_opened:%d == enable:%d and !als_re_enable; Done.", \
		          misc_als_opened,      enable);

		ret = 0;
		goto done;
	}
	misc_als_opened = enable;

	if (enable)
		ret = ap3426_power_on_als(client);
	else
		ret = ap3426_power_off_als(client);

	if (ret < 0){
		printk(KERN_ERR "%s: Error powering ALS On/Off\n", __func__);
	}

	msleep(50);
	if (misc_als_opened) {
		ALS_DBG("Enable Polling Timer\n");
		ret = mod_timer(&ps_data->pl_timer, jiffies + msecs_to_jiffies(PL_TIMER_DELAY));
	} else {
#ifndef PS_POLLING_DEBUG
		ALS_DBG("Disable Polling Timer\n");
		ret = del_timer_sync(&ps_data->pl_timer);
#endif
	}
done:
	ALS_RETURN("ret:%d", ret);
	return ret;
}

/*********************************************************************
light sensor register & unregister
********************************************************************/
static ssize_t ls_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ret;

    ret = misc_als_opened;
    return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t ls_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct ap3426_data *ps_data =  dev_get_drvdata(dev);
	uint8_t en;

	ENTRY("dev:%p, attr:%p, buf:%p, size:%zd", dev, attr, buf, size);

	if (sysfs_streq(buf, "1"))
		en = 1;
	else if (sysfs_streq(buf, "0"))
		en = 0;
	else {
		printk(KERN_ERR "invalid value %d\n",  *buf);
		return -EINVAL;
	}
	ALS_DBG("en = %d\n", (u32)(en));
	ap3426_als_enable(ps_data, en);

	RETURN("%zd", size);
	return size;
}

static struct device_attribute ls_enable_attribute = __ATTR(enable, 0666, ls_enable_show, ls_enable_store);

static struct attribute *ap3426_ls_attrs [] =
{
    &ls_enable_attribute.attr,
    NULL
};

static struct attribute_group ap3426_ls_attribute_group = {
        .attrs = ap3426_ls_attrs,
};

static int ap3426_register_lsensor_device(struct i2c_client *client, struct ap3426_data *data)
{
    struct input_dev *input_dev;
    int rc;

    ALS_DBG("allocating input device lsensor\n");
    input_dev = input_allocate_device();
    if (!input_dev) {
	dev_err(&client->dev,"%s: could not allocate input device for lsensor\n", __FUNCTION__);
	rc = -ENOMEM;
	goto done;
    }
    data->lsensor_input_dev = input_dev;
    input_set_drvdata(input_dev, data);
    input_dev->name = "light";//"lightsensor-level";
    input_dev->dev.parent = &client->dev;
    set_bit(EV_ABS, input_dev->evbit);
    input_set_abs_params(input_dev, ABS_MISC, 0, ((1 << 16) -1), 0, 0);

    rc = input_register_device(input_dev);
    if (rc < 0) {
	pr_err("%s: could not register input device for lsensor\n", __FUNCTION__);
	goto done;
    }
    rc = sysfs_create_group(&input_dev->dev.kobj, &ap3426_ls_attribute_group);// every devices register his own devices
done:
    return rc;
}

static void ap3426_unregister_lsensor_device(struct i2c_client *client, struct ap3426_data *data)
{
    input_unregister_device(data->lsensor_input_dev);
}

/*********************************************************************
heartbeat sensor register & unregister
********************************************************************/
#ifdef CONFIG_AP3426_HEARTBEAT_SENSOR
static int ap3426_register_heartbeat_sensor_device(struct i2c_client *client, struct ap3426_data *data)
{
    struct input_dev *input_dev;
    int rc;

    ALS_ENTRY("client:%pm data:%p", client, data);

    input_dev = input_allocate_device();
    if (!input_dev) {
	dev_err(&client->dev,"%s: could not allocate input device for heartbeat sensor\n", __FUNCTION__);
	rc = -ENOMEM;
	goto done;
    }
    data->hsensor_input_dev = input_dev;
    input_set_drvdata(input_dev, data);
    input_dev->name = "heartbeat";
    input_dev->dev.parent = &client->dev;
    set_bit(EV_ABS, input_dev->evbit);
    input_set_abs_params(input_dev, ABS_WHEEL, 0, 8, 0, 0);

    rc = input_register_device(input_dev);
    if (rc < 0) {
	pr_err("%s: could not register input device for heartbeat sensor\n", __FUNCTION__);
	goto done;
    }
done:
    ALS_RETURN("rc:%d", rc):
    return rc;
}

static void ap3426_unregister_heartbeat_device(struct i2c_client *client, struct ap3426_data *data)
{
    input_unregister_device(data->hsensor_input_dev);
}
#endif

/*********************************************************************
proximity sensor register & unregister
********************************************************************/
static ssize_t ps_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ret;

    ret = misc_als_opened;
    return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t ps_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct ap3426_data *ps_data =  dev_get_drvdata(dev);
	uint8_t en;

	if (sysfs_streq(buf, "1"))
		en = 1;
	else if (sysfs_streq(buf, "0"))
		en = 0;
	else {
		printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
                return -EINVAL;
        }
	ALS_DBG("en = %d\n", (u32)(en));
	ap3426_ps_enable(ps_data, en);
	return size;
}

static struct device_attribute ps_enable_attribute = __ATTR(enable, 0666, ps_enable_show, ps_enable_store);

static struct attribute *ap3426_ps_attrs [] =
{
    &ps_enable_attribute.attr,
    NULL
};

static struct attribute_group ap3426_ps_attribute_group = {
        .attrs = ap3426_ps_attrs,
};

static int ap3426_register_psensor_device(struct i2c_client *client, struct ap3426_data *data)
{
    struct input_dev *input_dev;
    int rc;

    PS_ENTRY("(client:%p, data:%p)", client, data);

    input_dev = input_allocate_device();
    PS_DBG("input_dev = %p = input_allocate_device();\n", input_dev);

    if (!input_dev) {
	dev_err(&client->dev,"%s: could not allocate input device for psensor\n", __FUNCTION__);
	rc = -ENOMEM;
	goto done;
    }
    data->psensor_input_dev = input_dev;
    input_set_drvdata(input_dev, data);
    input_dev->name = "proximity";
    input_dev->dev.parent = &client->dev;
    set_bit(EV_ABS, input_dev->evbit);
    input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);

    rc = input_register_device(input_dev);
    if (rc < 0) {
	PS_ERR("Could not register input device for psensor; rc:%d\n", rc);
	goto done;
    }

    // every devices register his own devices
    rc = sysfs_create_group(&input_dev->dev.kobj, &ap3426_ps_attribute_group);
    if (rc < 0) {
        PS_ERR(" rc = %d = sysfs_create_group(...);\n", rc);
    }

done:
    PS_RETURN("rc:%d", rc);
    return rc;
}

static void ap3426_unregister_psensor_device(struct i2c_client *client, struct ap3426_data *data)
{
    input_unregister_device(data->psensor_input_dev);
}
#if 0
static void ap3426_change_ls_threshold(struct i2c_client *client)
{
    struct ap3426_data *data = i2c_get_clientdata(client);
    int value;

    value = ap3426_get_adc_value(client);
    if(value > 0){
	ap3426_set_althres(client,ap3426_threshole[value-1]);
	ap3426_set_ahthres(client,ap3426_threshole[value]);
    }
    else{
	ap3426_set_althres(client,0);
	ap3426_set_ahthres(client,ap3426_threshole[value]);
    }

    input_report_abs(data->lsensor_input_dev, ABS_MISC, value);
    input_sync(data->lsensor_input_dev);

}
#endif


/* range */
static ssize_t ap3426_show_range(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    return sprintf(buf, "%i\n", ap3426_get_range(data->client));
}

static ssize_t ap3426_store_range(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    unsigned long val;
    int ret;

    if ((strict_strtoul(buf, 10, &val) < 0) || (val > 3))
	return -EINVAL;

    ret = ap3426_set_range(data->client, val);

    return (ret < 0)? ret:count;
}


//kevindang for msm8916 20141010
/*
 * ALS is typically enabled when Adaptive Brightness is enabled in setup.
 */
static int ap3426_als_enable_set(struct sensors_classdev *sensors_cdev,
						unsigned int enabled) 
{ 
	struct ap3426_data *als_data = container_of(sensors_cdev,
						struct ap3426_data, als_cdev); 
	int err; 
	int rv = 0;

	ALS_ENTRY("sensors_cdev:%p, enabled:%d", sensors_cdev, enabled);

	if (enabled && als_data->suspended) {
		printk(KERN_ERR "%s: Enabled while Suspended!\n", __func__);
	}

	err = ap3426_als_enable(als_data, enabled);

	if (err < 0)
		rv = err;;

	ALS_RETURN("rv:%d", rv);
	return 0;
}

static int ap3426_als_poll_delay_set(struct sensors_classdev *sensors_cdev, unsigned int delay_msec)
{
	struct ap3426_data *als_data = container_of(sensors_cdev, struct ap3426_data, als_cdev);
	int ret;
	
	if (delay_msec < MIN_ALS_POLL_DELAY_MS) {
		ret = mod_timer(&als_data->pl_timer, jiffies + msecs_to_jiffies(MIN_ALS_POLL_DELAY_MS));
	}
   
	if (delay_msec > PL_TIMER_DELAY) {
		ret = mod_timer(&als_data->pl_timer, jiffies + msecs_to_jiffies(PL_TIMER_DELAY));
	}
	return 0;
} 

#ifdef DI_AUTO_CAL
static u8 ps_calibrated = 0;

static inline void swap_at(u16 *x, u16 *y)
{
	u8 temp = *x;

	*x = *y;
	*y = temp;
}

static inline void ap3426_sort(u16 *sample_data, int size)
{
	int i, j;

	for (i = 0; i < size - 1; i++) {
		for (j = i + 1; j < size; j++) {
			if (sample_data[i] > sample_data[j]) {
				 swap_at(&sample_data[i], &sample_data[j]);
			}
		}
	}
	if (PS_info) {
		printk("%s: Sorted sample_data[ ", __func__);
		for (i = 0; i < size; i++) {
			printk("%d ", sample_data[i]);
		}
		printk("]\n");
	}
}


static int ap3426_set_ps_crosstalk_calibration(struct i2c_client *client, int val)
{
	int lsb, msb, err;

	msb = val >> 8;
	lsb = val & 0xFF;
	err = __ap3426_write_reg(client, 0x28, 0xFF, 0x00, lsb);
	     
	err =__ap3426_write_reg(client, 0x29, 0xFF, 0x00, msb);

	printk("%s: Calibration Register = %d.\n", __func__, val);
          
	return err;
}

#define CAL_SAMPLES 9

/*
 * Try to get 5 samples to set the calibration register.
 * Discard the 1st sample to two (as done on Tomato).
 * Discard any samples that are not in the expected range.
 * Samples taken just after changing the calibration
 * register haven't changed yet.
 */
int ap3426_ps_calibration(struct i2c_client *client)
{
	struct ap3426_data *pdata = i2c_get_clientdata(client);
	int i = 0;
	int rv = 0;
	u16 ave;
	u16 sample;
	u16 sample_data[CAL_SAMPLES];
	int samples = 0;

	if (!ps_calibrated) {
		ap3426_set_ps_crosstalk_calibration(client, 0);		/* Baseline */

		for (i = 0; i < CAL_SAMPLES; i++) {
			msleep(40);

			sample = sample_data[samples] = ap3426_get_px_value(client);

			PS_DBG("sample_data[samples:%d] = %d\n",
				            samples, sample);

			if (sample > pdata->ps_calibration_max) continue;
			if (sample < pdata->ps_calibration_min) continue;

			samples++;
		}
		if (samples >= CAL_SAMPLES/2) {
			u16 total = 0;
			int mid_point = (samples/2);
			int start_point = mid_point - (samples/4);
			int end_point = mid_point + (samples/4);

			ap3426_sort(sample_data, samples);

			/* Use median values of sorted data */
			samples = 0;
			for (i = start_point; i < end_point; i++) {
				PS_DBG("total:%d += sample_data[i:%d]:%d;\n",
					total,   i, sample_data[i]);

				total += sample_data[i];
				samples++;
			}
			ave = total/samples;
			PS_DBG("ave = %d\n", ave);
			ap3426_set_ps_crosstalk_calibration(client, ave);
			msleep(50);
			sample = ap3426_get_px_value(client);
			PS_DBG("sample = %d\n", sample);
			msleep(50);
			sample = ap3426_get_px_value(client);
			PS_DBG("sample = %d\n", sample);
			ps_calibrated = 1;
			rv = 1;
		} else {
			ave = pdata->ps_calibration_expected;
			printk("%s: Failed to compute a good calibration;\n", __func__);

			printk("%s: Using device's expected calibration value: %d for now.\n",
				__func__,                                     ave);

			ap3426_set_ps_crosstalk_calibration(client, ave);
			msleep(50);
			sample = ap3426_get_px_value(client);
			PS_DBG("sample = %d\n", sample);
			rv = -1;
		}
	}
	return rv;
}
#endif  /* DI_AUTO_CAL */

static int ap3426_ps_enable_set(struct sensors_classdev *sensors_cdev,
					   unsigned int enabled) 
{
	struct ap3426_data *ps_data = container_of(sensors_cdev,
					   struct ap3426_data, ps_cdev); 
	int err;

	err = ap3426_ps_enable(ps_data, enabled);

#ifdef DI_AUTO_CAL
	if (enabled == 1 && !ps_calibrated) {
		struct i2c_client *client = ps_data->client;

		/*
		 * Apparently we weren't able to calibrate during the probe.
		 * We are about to use the PS and we are curently using the
		 * default calibration value but it won't be used while
		 * calibrating.
		 *
		 * Might be better to prevent interrupts during calibration and
		 * avoid a less accurate value being reported during a unlikely
		 * race.
		 */
		ap3426_disable_ps_interrupts(client);
		ap3426_ps_calibration(client);
		ap3426_enable_ps_interrupts(client);
	}
#endif

	if (err < 0)
		return err;

	return 0;
}


//end
static int ap3426_power_ctl(struct ap3426_data *data, bool on)
{
	int ret = 0;

	if (!on && data->power_enabled)
	{
		if (!IS_ERR(data->vdd)) {
			ret = regulator_disable(data->vdd);
			if (ret) {
				dev_err(&data->client->dev,
					"Regulator vdd disable failed ret=%d\n", ret);
				return ret;
			}
		}

		if (!IS_ERR(data->vio)) {
			ret = regulator_disable(data->vio);
			if (ret) {
				dev_err(&data->client->dev,
					"Regulator vio disable failed ret=%d\n", ret);
				if (!IS_ERR(data->vdd)) {
					ret = regulator_enable(data->vdd);
					if (ret) {
						dev_err(&data->client->dev,
							"Regulator vdd enable failed ret=%d\n",
							ret);
					}
				}
				return ret;
			}
		}

		data->power_enabled = on;
		printk(KERN_INFO "%s: disable ap3426 power\n", __func__);
		dev_dbg(&data->client->dev, "ap3426_power_ctl on=%d\n",
				on);
	}
	else if (on && !data->power_enabled)
	{
		ret = regulator_enable(data->vdd);
		if (ret)
		{
			dev_err(&data->client->dev,
				"Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_enable(data->vio);
		if (ret) 
		{
			dev_err(&data->client->dev,
				"Regulator vio enable failed ret=%d\n", ret);
			regulator_disable(data->vdd);
			return ret;
		}

		data->power_enabled = on;
		printk(KERN_INFO "%s: enable ap3426 power\n", __func__);
	} 
	else
	{
		dev_warn(&data->client->dev,
				"Power on=%d. enabled=%d\n",
				on, data->power_enabled);
	}

	return ret;
}

static int ap3426_power_init(struct ap3426_data*data, bool on)
{
	int ret;

	ENTRY("data:%p, on:%d)", data, on);

	if (!on)
	{
		if (!IS_ERR(data->vdd)) {
			if (regulator_count_voltages(data->vdd) > 0)
				regulator_set_voltage(data->vdd, 0, AP3426_VDD_MAX_UV);
			regulator_put(data->vdd);
			data->vdd = ERR_PTR(-EINVAL);
		}

		if (!IS_ERR(data->vio)) {
			if (regulator_count_voltages(data->vio) > 0)
				regulator_set_voltage(data->vio, 0, AP3426_VIO_MAX_UV);

			regulator_put(data->vio);
			data->vio = ERR_PTR(-EINVAL);
		}
	}
	else
	{
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd))
		{
			ret = PTR_ERR(data->vdd);
			dev_err(&data->client->dev,
				"Regulator get failed vdd ret=%d\n", ret);
			return ret;
		}

		if (regulator_count_voltages(data->vdd) > 0)
		{
			ret = regulator_set_voltage(data->vdd,
					AP3426_VDD_MIN_UV,
					AP3426_VDD_MAX_UV);
			if (ret) 
			{
				dev_err(&data->client->dev,
					"Regulator set failed vdd ret=%d\n",
					ret);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio)) 
		{
			ret = PTR_ERR(data->vio);
			dev_err(&data->client->dev,
				"Regulator get failed vio ret=%d\n", ret);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0)
		{
			ret = regulator_set_voltage(data->vio,
					AP3426_VIO_MIN_UV,
					AP3426_VIO_MAX_UV);
			if (ret) 
			{
				dev_err(&data->client->dev,
				"Regulator set failed vio ret=%d\n", ret);
				goto reg_vio_put;
			}
		}
	}

	ret = 0;			/* Return(0); */
	goto done;

reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, AP3426_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);

done:
	RETURN("ret:%d", ret);
	return ret;
}


static DEVICE_ATTR(range, S_IWUSR | S_IRUGO,
	ap3426_show_range, ap3426_store_range);


/* mode */
static ssize_t ap3426_show_mode(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    return sprintf(buf, "%d\n", ap3426_get_mode(data->client));
}

static ssize_t ap3426_store_mode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    unsigned long val;
    int ret;

    if ((strict_strtoul(buf, 10, &val) < 0) || (val > 7))
	return -EINVAL;

    ret = ap3426_set_mode(data->client, val);

    if (ret < 0)
	return ret;
    ALS_DBG("Starting timer to fire in 200ms (%ld)\n", jiffies );
    ret = mod_timer(&data->pl_timer, jiffies + msecs_to_jiffies(PL_TIMER_DELAY));

    if(ret) {
	ALS_DBG("ret = %d = mod_timer(...); [Timer Error?] \n", ret);
    }
    return count;
}

static DEVICE_ATTR(mode, S_IRUGO | S_IWUGO,
	ap3426_show_mode, ap3426_store_mode);


/* lux */
static ssize_t ap3426_show_lux(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);

    /* No LUX data if power down */
    if (ap3426_get_mode(data->client) == AP3426_SYS_DEV_DOWN)
	return sprintf((char*) buf, "%s\n", "Please power up first!");

    return sprintf(buf, "%d\n", ap3426_get_adc_value(data->client));
}

static DEVICE_ATTR(lux, S_IRUGO, ap3426_show_lux, NULL);


/* Px data */
static ssize_t ap3426_show_pxvalue(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);

    /* No Px data if power down */
    if (ap3426_get_mode(data->client) == AP3426_SYS_DEV_DOWN)
	return -EBUSY;

    return sprintf(buf, "%d\n", ap3426_get_px_value(data->client));
}

static DEVICE_ATTR(pxvalue, S_IRUGO, ap3426_show_pxvalue, NULL);


/* proximity object detect */
static ssize_t ap3426_show_object(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    return sprintf(buf, "%d\n", ap3426_get_object(data->client));
}

static DEVICE_ATTR(object, S_IRUGO, ap3426_show_object, NULL);


/* ALS low threshold */
static ssize_t ap3426_show_althres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    return sprintf(buf, "%d\n", ap3426_get_althres(data->client));
}

static ssize_t ap3426_store_althres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3426_set_althres(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}

static DEVICE_ATTR(althres, S_IWUSR | S_IRUGO,
	ap3426_show_althres, ap3426_store_althres);


/* ALS high threshold */
static ssize_t ap3426_show_ahthres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    return sprintf(buf, "%d\n", ap3426_get_ahthres(data->client));
}

static ssize_t ap3426_store_ahthres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3426_set_ahthres(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}

static DEVICE_ATTR(ahthres, S_IWUSR | S_IRUGO,
	ap3426_show_ahthres, ap3426_store_ahthres);

/* Px low threshold */
static ssize_t ap3426_show_plthres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    return sprintf(buf, "%d\n", ap3426_get_plthres(data->client));
}

static ssize_t ap3426_store_plthres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3426_set_plthres(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}

static DEVICE_ATTR(plthres, S_IWUSR | S_IRUGO,
	ap3426_show_plthres, ap3426_store_plthres);

/* Px high threshold */
static ssize_t ap3426_show_phthres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    return sprintf(buf, "%d\n", ap3426_get_phthres(data->client));
}

static ssize_t ap3426_store_phthres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3426_set_phthres(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}

static DEVICE_ATTR(phthres, S_IWUSR | S_IRUGO,
	ap3426_show_phthres, ap3426_store_phthres);


/* calibration */
static ssize_t ap3426_show_calibration_state(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	return sprintf(buf, "%d\n", als_calibration);
}

static ssize_t ap3426_store_calibration_state(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    int stdls, lux; 
    char tmp[10];
    ssize_t rv = 0;

    ALS_ENTRY("dev:%p, attr:%p, buf:%p, count:%zd", dev, attr, buf, count);

    /* No LUX data if not operational */
    if (ap3426_get_mode(data->client) == AP3426_SYS_DEV_DOWN)
    {
	printk("%s: Please power up first!\n", __func__);
	rv = -EINVAL;
	goto done;
    }

    als_calibration = 100;
    sscanf(buf, "%d %s", &stdls, tmp);

    if (!strncmp(tmp, "-setcv", 6))
    {
	als_calibration = stdls;
	rv = -EBUSY;
	goto done;
    }

    if (stdls < 0)
    {
	printk("Std light source: [%d] < 0 !!!\nCheck again, please.\n\
		Set calibration factor to 100.\n", stdls);

	rv = -EBUSY;
	goto done;
    }

    lux = ap3426_get_adc_value(data->client);
    als_calibration = stdls * 100 / lux;

    rv = -EBUSY;		/* WHY? */

done:
    ALS_RETURN("rv:%zd", rv);
    return(rv);
}

static DEVICE_ATTR(calibration, S_IWUSR | S_IRUGO,
	ap3426_show_calibration_state, ap3426_store_calibration_state);

#ifdef ALS_DEBUG
/* engineer mode */
static ssize_t ap3426_em_read(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct ap3426_data *data = i2c_get_clientdata(client);
    int i;
    u8 tmp;

    ALS_DBG("DEBUG ap3426_em_read..\n");

    for (i = 0; i < AP3426_NUM_CACHABLE_REGS; i++)
    {
	tmp = i2c_smbus_read_byte_data(data->client, reg_array[i]);

	printk("Reg[0x%x] Val[0x%x]\n", reg_array[i], tmp);
    }

    return 0;
}

static ssize_t ap3426_em_write(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct ap3426_data *data = i2c_get_clientdata(client);
    u32 addr,val;
    int ret = 0;

    ALS_ENTRY("dev:%p, attr:%p, buf:%p, count:%zd", dev, attr, buf, count);

    sscanf(buf, "%x%x", &addr, &val);

    printk("Write [%x] to Reg[%x]...\n",val,addr);

    ret = i2c_smbus_write_byte_data(data->client, addr, val);
    if (!ret)
	    data->reg_cache[ap3426_reg_to_idx_array[addr]] = val;

    ALS_RETURN("count:%zd", count);
    return count;
}
static DEVICE_ATTR(em, S_IWUSR |S_IRUGO,
	ap3426_em_read, ap3426_em_write);
#endif

static struct attribute *ap3426_attributes[] = {
    &dev_attr_range.attr,
    &dev_attr_mode.attr,
    &dev_attr_lux.attr,
    &dev_attr_object.attr,
    &dev_attr_pxvalue.attr,
    &dev_attr_althres.attr,
    &dev_attr_ahthres.attr,
    &dev_attr_plthres.attr,
    &dev_attr_phthres.attr,
    &dev_attr_calibration.attr,
#ifdef ALS_DEBUG
    &dev_attr_em.attr,
#endif
    NULL
};

static const struct attribute_group ap3426_attr_group = {
    .attrs = ap3426_attributes,
};

static int32_t di_ap3426_set_ps_thd_l(struct ap3426_data *ps_data, uint16_t thd_l)
{
    return i2c_smbus_write_word_data(ps_data->client, 0x2A, thd_l);
}

static int32_t di_ap3426_set_ps_thd_h(struct ap3426_data *ps_data, uint16_t thd_h)
{
    return i2c_smbus_write_word_data(ps_data->client, 0x2C, thd_h);
}


static int ap3426_init_client(struct i2c_client *client)
{
    struct ap3426_data *data = i2c_get_clientdata(client);
    int rv = 0;
    int i;

    ENTRY("client:%p", client);

		/*lsensor high low thread*/
    i2c_smbus_write_byte_data(client, 0x1A, 0);
    i2c_smbus_write_byte_data(client, 0x1B, 0);

    i2c_smbus_write_byte_data(client, 0x1C, 0xFF);
    i2c_smbus_write_byte_data(client, 0x1D, 0XFF);
		/*psensor high low thread*/
    di_ap3426_set_ps_thd_l(data, data->ps_thd_l);
    di_ap3426_set_ps_thd_h(data, data->ps_thd_h);

    /* read all the registers once to fill the cache.
     * if one of the reads fails, we consider the init failed */
    for (i = 0; i < AP3426_NUM_CACHABLE_REGS; i++) {
	int v = i2c_smbus_read_byte_data(client, reg_array[i]);
	if (v < 0) {
	    rv = -ENODEV;
	    goto done;
	}
	data->reg_cache[i] = v;
    }
    /* set defaults */
    ap3426_set_range(client, AP3426_ALS_RANGE_0);
    ap3426_set_mode(client, AP3426_SYS_DEV_DOWN);

    ap3426_disable_ps_and_als_interrupts(client);

    //ps/IR integrated time as 9T,for sensitivity
    // PS mean time default = 0x00 (1 converseion time = 5ms)
    // 5 + 0x08 x 0.0627 = 5.5 ms (ADC photodiode sample period)
    i2c_smbus_write_byte_data(client, 0x25, 0x08);

done:
    RETURN("rv:%d", rv);
    return rv;;
}

static int ap3426_check_id(struct ap3426_data *data)
{
		return 0;
}

void pl_timer_callback(unsigned long pl_data)
{
	struct ap3426_data *data;
	int ps_wants_timer = 0;
	int ret = 0;

	ALS_ENTRY("pl_data:0X%lx", pl_data);

	data = private_pl_data;

#ifdef PS_POLLING_DEBUG
	if (misc_ps_opened)
		ps_wants_timer = 1;
#endif
	if (ps_wants_timer || misc_als_opened) {
		DBG("Re-Launching Timer. PL_TIMER_DELAY:%d ms\n", PL_TIMER_DELAY);
		queue_work(data->lsensor_wq, &data->lsensor_work);
		ret = mod_timer(&private_pl_data->pl_timer, jiffies + msecs_to_jiffies(PL_TIMER_DELAY));
		if (ret) {
			DBG("ret = %d = mod_timer(...); ret, [Timer Error?]\n", ret);
		}
	}
	ALS_RETURN_VOID();
	return;
}

static void psensor_work_handler(struct work_struct *w)
{

    struct ap3426_data *data =
	container_of(w, struct ap3426_data, psensor_work);
    int distance,pxvalue;

    distance = ap3426_get_object(data->client);
    pxvalue = ap3426_get_px_value(data->client); //test
	
    input_report_abs(data->psensor_input_dev, ABS_DISTANCE, distance);
    input_sync(data->psensor_input_dev);
    ALS_DBG("distance:%d, value:%d\n", distance, pxvalue);
}

static void lsensor_work_handler(struct work_struct *w)
{

    struct ap3426_data *data =
	container_of(w, struct ap3426_data, lsensor_work);
    int value;

    ALS_ENTRY("w:%p", w);

    value = ap3426_get_adc_value(data->client);
    value = value * als_calibration / 100;
    input_report_abs(data->lsensor_input_dev, ABS_MISC, value);
    input_sync(data->lsensor_input_dev);

    ALS_RETURN_VOID();
    return;
}

/*
 * The Interrupt Clear Manner (CLR_MNR) has been set to 0 (Automatic) so
 * this ISR must read the ALS and PS sensor values to clear the interripts
 * even if PS or ALS aren't currently open.
 */
static irqreturn_t ap3426_threaded_isr(int irq, void *client_data)
{

	struct ap3426_data *data = (struct ap3426_data *) client_data;
	u8 int_stat;
	int ps_value;
	int distance;
	int als_value;

	PS_ENTRY("irg:%d, client_data:%p", irq, client_data);

	int_stat = ap3426_get_intstat(data->client);

	ps_value = ap3426_get_px_value(data->client);   	/* Clear PS  Interrupt */
	als_value = ap3426_get_adc_value(data->client); 	/* Clear ALS Interrupt */

	if (int_stat & AP3426_REG_SYS_INT_PMASK) {
		/* We have a PS Interrupt */
		if (misc_ps_opened) {
			distance = ap3426_get_object(data->client);
			input_report_abs(data->psensor_input_dev, ABS_DISTANCE, distance);
			input_sync(data->psensor_input_dev);
			wake_lock_timeout(&data->ps_wakelock, 2*HZ);
		}
	}

#ifdef CONFIG_AP3426_HEARTBEAT_SENSOR
	if (misc_ht_opened) {
		input_report_abs(data->hsensor_input_dev, ABS_WHEEL, ps_value);
		input_sync(data->hsensor_input_dev);
	}
#endif

	if (int_stat & AP3426_REG_SYS_INT_AMASK) {
		/* We have an ALS Interrupt */
		if (misc_als_opened) {
			input_report_abs(data->lsensor_input_dev, ABS_MISC, als_value);
			input_sync(data->lsensor_input_dev);
		}
	}
	PS_DBG("ps_value:%d, als_value:%d, distance:%d;\n",
		ps_value,    als_value,    distance);

	PS_RETURN("IRQ_HANDLED:%d", IRQ_HANDLED);
	return IRQ_HANDLED;
}


static irqreturn_t ap3426_irq(int irq, void *data_)
{
    return IRQ_WAKE_THREAD;
}

#ifdef CONFIG_OF	/* Open Firmware */
static int ap3426_parse_dt(struct device *dev, struct ap3426_data *pdata)
{
	struct device_node *dt = dev->of_node;
	int rv = 0;
	u32 temp_val;
	int rc;

	ENTRY("dev:%p, pdata:%p", dev, pdata);

	if (pdata == NULL) {
		PS_DBG("pdata is NULL; rv = -EINVAL\n");
		rv = -EINVAL;
		goto done;
	}

	pdata->int_pin = of_get_named_gpio_flags(dt, "ap3426,irq-gpio",
	                            0, &pdata->irq_flags);
	if (pdata->int_pin < 0) {
		dev_err(dev, "Unable to read 'ap3426,irq-gpio' from Device Tree.\n");
		rv = pdata->int_pin;
		goto done;
	}

	rc = of_property_read_u32(dt, "ap3426,ps-thdl", &temp_val);
	if (!rc) {
		pdata->ps_thd_l = (u16)temp_val;
		PS_DBG("ps-thdl = %d\n", pdata->ps_thd_l);
	} else {
		pdata->ps_thd_l = PX_LOW_THRESHOLD;
		dev_err(dev, "Unable to read 'ap3426,ps-thdl', using default %d\n",
		                              pdata->ps_thd_l);
	}

	rc = of_property_read_u32(dt, "ap3426,ps-thdh", &temp_val);
	if (!rc) {
		pdata->ps_thd_h= (u16)temp_val;
		PS_DBG("ps-thdh = %d\n", pdata->ps_thd_h);
	} else {
		pdata->ps_thd_h = PX_HIGH_THRESHOLD;
		dev_err(dev, "Unable to read 'ap3426,ps-thdh', using default %d\n",
		                              pdata->ps_thd_h);
	}

#ifdef DI_AUTO_CAL
	rc = of_property_read_u32(dt, "ap3426,ps-calibration-min", &temp_val);
	if (!rc) {
		pdata->ps_calibration_min = (u16)temp_val;
		PS_DBG("ps-calibration-min = %d\n", pdata->ps_calibration_min);
	} else {
		pdata->ps_calibration_min = DI_PS_CAL_THR_MIN;
		dev_err(dev, "Unable to read 'ap3426,ps-calibration-min', using default %d\n",
		                              pdata->ps_calibration_min);
	}

	rc = of_property_read_u32(dt, "ap3426,ps-calibration-expected", &temp_val);
	if (!rc) {
		pdata->ps_calibration_expected = (u16)temp_val;
		PS_DBG("ps-calibration_expected = %d\n", pdata->ps_calibration_expected);
	} else {
		pdata->ps_calibration_expected = DI_PS_CAL_THR_EXPECTED;
		dev_err(dev, "Unable to read 'ap3426,ps-calibration-expected', using default %d\n",
	                                      pdata->ps_calibration_expected);
	}

	rc = of_property_read_u32(dt, "ap3426,ps-calibration-max", &temp_val);
	if (!rc) {
		pdata->ps_calibration_max = (u16)temp_val;
		PS_DBG("ps-calibration-max = %d\n", pdata->ps_calibration_max);
	} else {
		pdata->ps_calibration_max = DI_PS_CAL_THR_MAX;
		dev_err(dev, "Unable to read 'ap3426,ps-calibration-max', using default %d\n",
		                              pdata->ps_calibration_max);
	}
#endif

done:
	RETURN("rv:%d", rv);
	return rv;
}
#endif

static int ap3426_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct ap3426_data *data;
	int err = 0;

	ENTRY("client:%p, id:%p)", client, id);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		err = -EIO;
		goto exit_free_gpio;
	}

	data = kzalloc(sizeof(struct ap3426_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit_free_gpio;
	}

#ifdef CONFIG_OF
	if (client->dev.of_node) {
		ALS_DBG("Device Tree parsing.\n");

		err = ap3426_parse_dt(&client->dev, data);
		if (err) {
			dev_err(&client->dev, "%s: ap3426_parse_dt "
				"for pdata failed. err = %d",
				__func__, err);

			goto exit_parse_dt_fail;
		}
	}
#else
	data->irq = client->irq;
	// No dtsi values for ps thresholds, use defaults
	data->ps_thd_l = PX_LOW_THRESHOLD;
	data->ps_thd_h = PX_HIGH_THRESHOLD;
#endif

	data->client = client;
	i2c_set_clientdata(client, data);

	err = ap3426_power_init(data, true);
	if (err)
		goto err_power_on;

	err = ap3426_power_ctl(data, true);
	if (err)
		goto err_power_ctl;

	/* initialize the AP3426 chip - Makes sure interrupts are disabled. */
	err = ap3426_init_client(data->client);
	if (err)
		goto err_init_client;

	if (ap3426_check_id(data) !=0 ) {
		dev_err(&client->dev, "failed to check ap3426 id\n");
		goto err_init_client;
	}

	err = ap3426_register_lsensor_device(client,data);
	if (err) {
		dev_err(&client->dev, "failed to register_lsensor_device\n");
		goto err_init_client;
	}

	err = ap3426_register_psensor_device(client, data);
	if (err) {
		dev_err(&client->dev, "failed to register_psensor_device\n");
		goto exit_free_ls_device;
	}

#ifdef CONFIG_AP3426_HEARTBEAT_SENSOR
	err = ap3426_register_heartbeat_sensor_device(client, data);
	if (err) {
		dev_err(&client->dev, "failed to register_heartbeatsensor_device\n");
		goto exit_free_ps_device;
	}
#endif
	err = gpio_request(data->int_pin,"ap3426-int");
	if (err < 0) {
		printk(KERN_ERR "%s: gpio_request, err=%d\n", __func__, err);
#ifdef CONFIG_AP3426_HEARTBEAT_SENSOR
		goto exit_free_heartbeat_device;
#else
		goto exit_free_ps_device;
#endif
	}

	err = gpio_direction_input(data->int_pin);
	if (err < 0) {
		printk(KERN_ERR "%s: gpio_direction_input, err=%d\n", __func__, err);
		goto exit_free_gpio_int;
	}

	err = request_threaded_irq(gpio_to_irq(data->int_pin), ap3426_irq, ap3426_threaded_isr,
		IRQF_TRIGGER_LOW  | IRQF_ONESHOT,"ap3426", data);

	if (err) {
		dev_err(&client->dev, "err:%d, could not get IRQ %d\n", err, gpio_to_irq(data->int_pin));
		goto exit_free_gpio_int;
	}

	/* enable irq wake up */
	DBG("enable_irq_wake(gpio_to_irq(data->int_pin:%d)); irq:%d\n", data->int_pin, gpio_to_irq(data->int_pin));
	err = enable_irq_wake(gpio_to_irq(data->int_pin));
	if (err) {
		dev_err(&client->dev, "err: %d, could not enable irq wake for irq:%d\n", err, gpio_to_irq(data->int_pin));
		goto exit_request_irq;
	}

	data->psensor_wq = create_singlethread_workqueue("psensor_wq");

	if (!data->psensor_wq) {
		ALS_ERR("create psensor_wq workqueue failed\n");
		err = -ENOMEM;
		goto exit_request_irq;
	}
	INIT_WORK(&data->psensor_work, psensor_work_handler);

	data->lsensor_wq = create_singlethread_workqueue("lsensor_wq");
	if (!data->lsensor_wq) {
		ALS_DBG("create lsensor_wq workqueue failed\n");
		err = -ENOMEM;
		goto err_create_psensor_wq;
	}
	INIT_WORK(&data->lsensor_work, lsensor_work_handler);

	/*
	 * Seems a bit cleaner if we were to provid 'data' as 3rd arg to setup_timer()
	 * and avoiding us of global pointer in pl_timer_callback().
	 */
	setup_timer(&data->pl_timer, pl_timer_callback, 0);

	err = sysfs_create_group(&data->client->dev.kobj, &ap3426_attr_group);
	if (err)
		goto err_sysfs_create_group;


	DBG("wake_lock_init(&data->ps_wakelock, WAKE_LOCK_SUSPEND, 'ps_wakelock');\n");

	wake_lock_init(&data->ps_wakelock, WAKE_LOCK_SUSPEND, "ps_wakelock");

	data->als_cdev = sensors_light_cdev;
	data->als_cdev.sensors_enable = ap3426_als_enable_set;
	data->als_cdev.sensors_poll_delay = ap3426_als_poll_delay_set;
	err = sensors_classdev_register(&client->dev, &data->als_cdev);
	if (err)
		goto err_sensors_classdev_register;

	data->ps_cdev = sensors_proximity_cdev;
	data->ps_cdev.sensors_enable = ap3426_ps_enable_set;
	err = sensors_classdev_register(&client->dev, &data->ps_cdev);
	if(err)
		goto err_sensors_classdev_register_ps;

	private_pl_data = data;

	// init the ps event value because it may cause screen off at first call
	input_report_abs(data->psensor_input_dev, ABS_DISTANCE, 1);
	input_sync(data->psensor_input_dev);

	dev_info(&client->dev, "Driver version %s enabled\n", DRIVER_VERSION);

#ifdef DI_AUTO_CAL
	/*
         * Try to calibrate now;
	 * if PS is covered try again later when PS is used.
	 */
	ap3426_power_on_ps(data->client);
	msleep(100);
	ap3426_ps_calibration(data->client);
	ap3426_power_off_ps(data->client);
#endif
	err = 0;		/* Return(0) */
	goto done;

err_sensors_classdev_register_ps:
	sensors_classdev_unregister(&data->als_cdev);

err_sensors_classdev_register:
	DBG("wake_lock_destroy(&data->ps_wakelock);\n");
	wake_lock_destroy(&data->ps_wakelock);
	sysfs_remove_group(&data->client->dev.kobj, &ap3426_attr_group);

err_sysfs_create_group:
	if (data->lsensor_wq)
		destroy_workqueue(data->lsensor_wq);

err_create_psensor_wq:
	if (data->psensor_wq)
		destroy_workqueue(data->psensor_wq);

exit_request_irq:
	free_irq(gpio_to_irq(data->int_pin), data);

exit_free_gpio_int:
	gpio_free(data->int_pin);

#ifdef CONFIG_AP3426_HEARTBEAT_SENSOR
exit_free_heartbeat_device:
	ap3426_unregister_heartbeat_device(client, data);
#endif

exit_free_ps_device:
	ap3426_unregister_psensor_device(client,data);

exit_free_ls_device:
	ap3426_unregister_lsensor_device(client,data);

err_init_client:
	ap3426_power_ctl(data, false);

err_power_ctl:
	ap3426_power_init(data, false);

err_power_on:
#ifdef CONFIG_OF
exit_parse_dt_fail:
	ALS_DBG("dts initialize failed.");
#endif
	kfree(data);

exit_free_gpio:

done:
    RETURN("err:%d", err);
	return err;
}

static int ap3426_remove(struct i2c_client *client)
{
    struct ap3426_data *data = i2c_get_clientdata(client);

    ENTRY("client:%p", client);

    free_irq(gpio_to_irq(data->int_pin), data);

    ap3426_power_ctl(data, false);

    sysfs_remove_group(&data->client->dev.kobj, &ap3426_attr_group);
//kevindang20140924
    sysfs_remove_group(&data->psensor_input_dev->dev.kobj, &ap3426_ps_attribute_group);// every devices register his own devices
    sysfs_remove_group(&data->lsensor_input_dev->dev.kobj, &ap3426_ls_attribute_group);// every devices register his own devices
//end
    ap3426_unregister_psensor_device(client,data);
    ap3426_unregister_lsensor_device(client,data);
#ifdef CONFIG_AP3426_HEARTBEAT_SENSOR
    ap3426_unregister_heartbeat_device(client,data);
#endif
    ap3426_power_init(data, false);


    ap3426_set_mode(client, 0);
    kfree(i2c_get_clientdata(client));

    if (data->psensor_wq)
	destroy_workqueue(data->psensor_wq);
    if (data->lsensor_wq)
	destroy_workqueue(data->lsensor_wq);
    if(&data->pl_timer)
	del_timer(&data->pl_timer);

    DBG("wake_lock_destroy(&data->ps_wakelock);\n");
    wake_lock_destroy(&data->ps_wakelock);

    RETURN("%d", 0);
    return 0;
}

static const struct i2c_device_id ap3426_id[] =
{
    { AP3426_DRV_NAME, 0 },
    {}
};
MODULE_DEVICE_TABLE(i2c, ap3426_id);

#ifdef CONFIG_OF
static struct of_device_id ap3426_match_table[] =
{
        {.compatible = "di_ap3426" },
        {},
};
#else
#define ap3426_match_table NULL
#endif


static int ap3426_suspend(struct device *dev)
{
	struct ap3426_data *ps_data = dev_get_drvdata(dev);

	ENTRY("dev:%p)", dev);

	if (misc_als_opened == 1) {
		ap3426_als_enable(ps_data, false);
		ps_data->als_re_enable = 1;
	}
        if (misc_ps_opened == 1) {
            ap3426_ps_enable(ps_data, false);
            ps_data->ps_re_enable = 1;


        }

	// Power off while suspended
	//ap3426_power_init(ps_data,false); // may cause system can't wake up

	ap3426_power_ctl(ps_data,false);
	ps_data->suspended = 1;

	RETURN("%d", 0);
	return 0;
}

static int ap3426_resume(struct device *dev)
{
	struct ap3426_data *ps_data = dev_get_drvdata(dev);

	ENTRY("dev:%p)", dev);
	DBG("ps_data:%p->rals_re_enable:%d\n", ps_data, ps_data->als_re_enable);

	// power_int() currently disabled it because
	// it may prevent system from wakeing up.
	//
	// ap3426_power_init(ps_data,true);
	ap3426_power_ctl(ps_data, true);

	 // Always re-iniialized our register cache and stuff.
	ap3426_init_client(ps_data->client);

	if (ps_data->ps_re_enable ) {
                ap3426_ps_enable(ps_data, misc_ps_opened);
                ps_data->ps_re_enable = 0;
        }
        if (ps_data->als_re_enable ) {
                ap3426_als_enable(ps_data, misc_als_opened);
                ps_data->als_re_enable = 0;
        }
	ps_data->suspended = 0;

	RETURN("%d", 0);
	return 0;
}

static SIMPLE_DEV_PM_OPS(ap3426_pm_ops, ap3426_suspend, ap3426_resume);

static struct i2c_driver ap3426_driver = {
    .driver = {
	.name	= AP3426_DRV_NAME,
	.owner	= THIS_MODULE,
	.of_match_table = ap3426_match_table,
	.pm     = &ap3426_pm_ops,
    },
    .probe	= ap3426_probe,
    .remove	= ap3426_remove,
    .id_table = ap3426_id,
};

static int __init ap3426_init(void)
{
    int ret;

    ret = i2c_add_driver(&ap3426_driver);
    return ret;	

}

static void __exit ap3426_exit(void)
{
    i2c_del_driver(&ap3426_driver);
}

module_init(ap3426_init);
module_exit(ap3426_exit);
MODULE_AUTHOR("Kevin.dang, <kevin.dang@dyna-image.com>");
MODULE_DESCRIPTION("AP3426 driver.");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
