/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
*
* File Name          : hts221.c
* Authors            : Motion MEMS
*                      Lorenzo SARCHI <lorenzo.sarchi@st.com> 
*                      Morris Chen (morris.chen@st.com)
* Version            : V.1.0.2
* Date               : 08/11/2013
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
********************************************************************************
********************************************************************************
Version History.
 
Revision 1-0-0 06/03/2013
 first revision
 
Revision 1-0-1 07/23/2013
 Optimization process
 
Revision 1-0-2 8/11/2013
 Increase Accuracy
   a. multiplying by 8000 instead of 1000
   b. the conversion of the read values,  by shifting 4 (H case) or 6 (T) bit

*******************************************************************************/
 
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
 
#include <linux/sensors/hts221.h>
#include <linux/of_gpio.h>
#include <linux/of.h>

#define I2C_AUTO_INCREMENT      (0x80)
#define MS_TO_NS(x)             (x*1000000L)
 
#define MAG_MAX_POS             214748364       /** Max positive value */
#define MAG_MAX_NEG             214748364       /** Max negative value */
 
 
/* Address registers */
#define REG_WHOAMI_ADDR         (0x0f)  /** Who am i address register */
#define REG_H_RES_ADDR          (0x10)  /** Humidity res conf register */
#define REG_T_RES_ADDR          (0x10)  /** Temperature res conf register */
#define REG_CNTRL1_ADDR         (0x20)  /** CNTRL1 address register */
#define REG_CNTRL2_ADDR         (0x21)  /** CNTRL2 address register */
#define REG_H_OUT_L             (0x28)  /** OUT humidity address register */
#define REG_T_OUT_L             (0x2A)  /** OUT temperature address register */
#define REG_0RH_CAL_X_H         (0X36)  /** Calibration H 0 address register */
#define REG_1RH_CAL_X_H         (0X3a)  /** Calibration H 1 address register */
#define REG_0RH_CAL_Y_H         (0x30)  /** Calibration H 0 RH address register */
#define REG_1RH_CAL_Y_H         (0x31)  /** Calibration H 1 RH address register */
#define REG_0T_CAL_X_L          (0x3c)  /** Calibration T 0 address register */
#define REG_1T_CAL_X_L          (0x3e)  /** Calibration T 1 address register */
#define REG_0T_CAL_Y_H          (0x32)  /** Calibration T 0 C address register */
#define REG_1T_CAL_Y_H          (0x33)  /** Calibration T 1 C address register */
#define REG_STATUS              (0x27)  /** Status address register */

#define REG_T1_T0_CAL_Y_H       (0x35)  /** Calibration T0 and T! Address register  **/
 
/* Enable/Disable Sensor */
#define MASK_ENABLE             (0x80)
#define ENABLE_SENSOR           (0x80)
#define DISABLE_SENSOR          (0x00)
 
/* Sensor Resolution */
#define HTS221_H_RESOLUTION_MASK (0xf8)
#define HTS221_T_RESOLUTION_MASK (0xC7)
 
/* Default values loaded in probe function */
#define WHOIAM_VALUE            (0xbc)  /** Who Am I default value */
#define REG_DEF_H_RES_ADDR      (0x00)  /** Humidity res register */
#define REG_DEF_T_RES_ADDR      (0x00)  /** Temperature res register */
#define REG_DEF_CNTRL1          (0x00)  /** CNTRL1 default value */
#define REG_DEF_CNTRL2          (0x00)  /** CNTRL2 default value */
 
#define REG_DEF_ALL_ZEROS       (0x00)

#define REG_DEF_BDU             (0x40)

//#ifndef DEBUG
//#define DEBUG
//#endif

//#define BDU
 
 
 // Global variables, to be read only 1 time    LS
u8 humidity_calibration[2];
u8 temperature_calibration[2], temperature_calibration2[2];
int calibX0, calibX1, calibW0, calibW1;
u16 calibY0, calibY1;

u16 calibZ0a, calibZ0b, calibZ1a, calibZ1b;
int calibZ0, calibZ1;
int h_slope, h_b, t_slope, t_b;

 
struct workqueue_struct *hts221_workqueue = 0;
 
struct hts221_status {
        struct i2c_client *client;
        struct hts221_platform_data *pdata;
 
        struct mutex lock;
        struct work_struct input_work;
 
        struct hrtimer hr_timer;
        ktime_t ktime;
 
        struct input_dev *input_dev_humidity;
        struct input_dev *input_dev_temperature;
 
        int hw_initialized;
        /* hw_working=-1 means not tested yet */
        int hw_working;
        
        u8 reg_addr;
 
        atomic_t enabled;
 
        int on_before_suspend;
        int use_smbus;
};
 
static const struct hts221_platform_data default_hts221_pdata = {
        .poll_interval = HTS221_DEFAULT_POLL_PERIOD_MS,
        .min_interval = HTS221_MIN_POLL_PERIOD_MS,
        .h_resolution = HTS221_H_RESOLUTION_32,
        .t_resolution = HTS221_T_RESOLUTION_16,
};
 
struct {
        unsigned int resolution;
        u8 value;
} resolution_h[] = {
                {  4, HTS221_H_RESOLUTION_4   },
                {  8, HTS221_H_RESOLUTION_8   },
                { 16, HTS221_H_RESOLUTION_16  },
                { 32, HTS221_H_RESOLUTION_32  },
                { 64, HTS221_H_RESOLUTION_64  },
                {128, HTS221_H_RESOLUTION_128 },
                {256, HTS221_H_RESOLUTION_256 },
                {512, HTS221_H_RESOLUTION_512 },
};
 
struct {
        unsigned int resolution;
        u8 value;
} resolution_t[] = {
                {  2, HTS221_T_RESOLUTION_2   },
                {  4, HTS221_T_RESOLUTION_4   },
                {  8, HTS221_T_RESOLUTION_8   },
                { 16, HTS221_T_RESOLUTION_16  },
                { 32, HTS221_T_RESOLUTION_32  },
                { 64, HTS221_T_RESOLUTION_64  },
                {128, HTS221_T_RESOLUTION_128 },
                {256, HTS221_T_RESOLUTION_256 },
};
 
struct reg_rw {
        u8 address;
        u8 default_value;
        u8 resume_value;
};
 
struct reg_r {
        u8 address;
        u8 value;
};
 
static struct status_registers {
        struct reg_r who_am_i;
        struct reg_rw cntrl1;
        struct reg_rw cntrl2;
        struct reg_rw h_res;
        struct reg_rw t_res;
} status_registers = {
        .who_am_i.address=REG_WHOAMI_ADDR, .who_am_i.value=WHOIAM_VALUE,
        .cntrl1.address=REG_CNTRL1_ADDR, .cntrl1.default_value=REG_DEF_CNTRL1,
        .cntrl2.address=REG_CNTRL2_ADDR, .cntrl2.default_value=REG_DEF_CNTRL2,
        .h_res.address=REG_H_RES_ADDR, .h_res.default_value=REG_DEF_H_RES_ADDR,
        .t_res.address=REG_T_RES_ADDR, .t_res.default_value=REG_DEF_T_RES_ADDR,
};
 
static int hts221_i2c_read(struct hts221_status *stat, u8 *buf, int len)
{
        int ret;
        u8 reg = buf[0];
        u8 cmd = reg;
#ifdef DEBUG
        unsigned int ii;
#endif
 
 
        if (len > 1)
                cmd = (I2C_AUTO_INCREMENT | reg);
        if (stat->use_smbus) {
                if (len == 1) {
                        ret = i2c_smbus_read_byte_data(stat->client, cmd);
                        buf[0] = ret & 0xff;
#ifdef DEBUG
                        dev_warn(&stat->client->dev,
                                "i2c_smbus_read_byte_data: ret=0x%02x, len:%d ,"
                                "command=0x%02x, buf[0]=0x%02x\n",
                                ret, len, cmd , buf[0]);
#endif
                } else if (len > 1) {
                        ret = i2c_smbus_read_i2c_block_data(stat->client,
                                                                cmd, len, buf);
#ifdef DEBUG
                        dev_warn(&stat->client->dev,
                                "i2c_smbus_read_i2c_block_data: ret:%d len:%d, "
                                "command=0x%02x, ",
                                ret, len, cmd);
                        for (ii = 0; ii < len; ii++)
                                printk(KERN_DEBUG "buf[%d]=0x%02x,",
                                                                ii, buf[ii]);
 
                        printk("\n");
#endif
                } else
                        ret = -1;
 
                if (ret < 0) {
                        dev_err(&stat->client->dev,
                                "read transfer error: len:%d, command=0x%02x\n",
                                len, cmd);
                        return 0;
                }
                return len;
        }
 
        ret = i2c_master_send(stat->client, &cmd, sizeof(cmd));
        if (ret != sizeof(cmd))
                return ret;
 
        return i2c_master_recv(stat->client, buf, len);
}
 
static int hts221_i2c_write(struct hts221_status *stat, u8 *buf, int len)
{
        int ret;
        u8 reg, value;
#ifdef DEBUG
        unsigned int ii;
#endif
 
        if (len > 1)
                buf[0] = (I2C_AUTO_INCREMENT | buf[0]);
 
        reg = buf[0];
        value = buf[1];
 
        if (stat->use_smbus) {
                if (len == 1) {
                        ret = i2c_smbus_write_byte_data(stat->client,
                                                                reg, value);
#ifdef DEBUG
                        dev_warn(&stat->client->dev,
                                "i2c_smbus_write_byte_data: ret=%d, len:%d, "
                                "command=0x%02x, value=0x%02x\n",
                                ret, len, reg , value);
#endif
                        return ret;
                } else if (len > 1) {
                        ret = i2c_smbus_write_i2c_block_data(stat->client,
                                                        reg, len, buf + 1);
#ifdef DEBUG
                        dev_warn(&stat->client->dev,
                                "i2c_smbus_write_i2c_block_data: ret=%d, "
                                "len:%d, command=0x%02x, ",
                                ret, len, reg);
                        for (ii = 0; ii < (len + 1); ii++)
                                printk(KERN_DEBUG "value[%d]=0x%02x,",
                                                                ii, buf[ii]);
 
                        printk("\n");
#endif
                        return ret;
                }
        }
 
        ret = i2c_master_send(stat->client, buf, len+1);
        return (ret == len+1) ? 0 : ret;
}
 
static int hts221_hw_init(struct hts221_status *stat)
{
        int err = -1;
        u8 buf[1];
 
        pr_info("%s: hw init start\n", HTS221_DEV_NAME);
 
        buf[0] = status_registers.who_am_i.address;
        err = hts221_i2c_read(stat, buf, 1);
 
        if (err < 0) {
                dev_warn(&stat->client->dev, "Error reading WHO_AM_I: is device"
                " available/working?\n");
                goto err_firstread;
        } else
                stat->hw_working = 1;
 
        if (buf[0] != status_registers.who_am_i.value) {
        dev_err(&stat->client->dev,
                "device unknown. Expected: 0x%02x,"
                " Replies: 0x%02x\n", status_registers.who_am_i.value, buf[0]);
                err = -1;
                goto err_unknown_device;
        }
 
        status_registers.cntrl1.resume_value =
                                        status_registers.cntrl1.default_value;
        status_registers.cntrl2.resume_value =
                                        status_registers.cntrl2.default_value;
        status_registers.h_res.resume_value =
                                        status_registers.h_res.default_value;
        status_registers.t_res.resume_value =
                                        status_registers.t_res.default_value;
 
        stat->hw_initialized = 1;
        pr_info("%s: hw init done\n", HTS221_DEV_NAME);
 
        return 0;
 
err_unknown_device:
err_firstread:
        stat->hw_working = 0;
        stat->hw_initialized = 0;
        return err;
}
 
static int hts221_device_power_off(struct hts221_status *stat)
{
        int err;
        u8 buf[2];
 
        buf[0] = status_registers.cntrl1.address;
        buf[1] = ((MASK_ENABLE & DISABLE_SENSOR) |
                ((~MASK_ENABLE) & status_registers.cntrl1.resume_value));
 
        err = hts221_i2c_write(stat, buf, 1);
        if (err < 0)
                dev_err(&stat->client->dev, "humidity soft power off "
                                                        "failed: %d\n", err);
 
        if (stat->pdata->power_off) {
                stat->pdata->power_off();
        }
 
        atomic_set(&stat->enabled, 0);
 
        return 0;
}
 
static int hts221_device_power_on(struct hts221_status *stat)
{
        int err = -1;
        u8 buf[6];
 
        if (stat->pdata->power_on) {
                err = stat->pdata->power_on();
                if (err < 0) {
                        dev_err(&stat->client->dev,
                                "humidity power_on failed: %d\n", err);
                        return err;
                }
        }
 
        buf[0] = status_registers.h_res.address;
        buf[1] = status_registers.h_res.resume_value;
        err = hts221_i2c_write(stat, buf, 1);
        if (err < 0)
                goto err_resume_state;
 
        buf[0] = status_registers.t_res.address;
        buf[1] = status_registers.t_res.resume_value;
        err = hts221_i2c_write(stat, buf, 1);
        if (err < 0)
                goto err_resume_state;
 
        buf[0] = status_registers.cntrl1.address;
                
    /* BDU */
    #ifdef BDU
        buf[1] = ((MASK_ENABLE & ENABLE_SENSOR) |
                 ((~MASK_ENABLE) & status_registers.cntrl1.resume_value) |
                   REG_DEF_BDU);
    #else
        buf[1] = ((MASK_ENABLE & ENABLE_SENSOR) |
                ((~MASK_ENABLE) & status_registers.cntrl1.resume_value));
    #endif
 
        err = hts221_i2c_write(stat, buf, 1);
        if (err < 0)
                goto err_resume_state;
 
        atomic_set(&stat->enabled, 1);
 
        return 0;
 
err_resume_state:
        atomic_set(&stat->enabled, 0);
        dev_err(&stat->client->dev, "humidity hw power on error "
                                "0x%02x,0x%02x: %d\n", buf[0], buf[1], err);
        return err;
}
 
static int hts221_update_resolution(struct hts221_status *stat, u8 new_resolution, 
                                                                int type)
{
        int err=-1;
        u8 buf[2];
        u8 addr;
 
        if(type==0)
                addr = status_registers.h_res.address;
        else
                addr = status_registers.t_res.address;
 
        buf[0] = addr;
        err = hts221_i2c_read(stat, buf, 1);
        if (err < 0)
                goto error;
 
        if(type==0) {
                status_registers.h_res.resume_value = buf[0];
                new_resolution = (buf[0] & HTS221_H_RESOLUTION_MASK) | new_resolution;
        }              
        else {
                status_registers.t_res.resume_value = buf[0];
                new_resolution = (buf[0] & HTS221_T_RESOLUTION_MASK) | new_resolution;
        }
                
 
        buf[1] = new_resolution;
        buf[0] = addr;      
 
        err = hts221_i2c_write(stat, buf, 1);
        if (err < 0)
                goto error;

        if(type==0)
                status_registers.h_res.resume_value = buf[0];    
        else
                status_registers.t_res.resume_value = buf[0];
 
        return err;
 
error:
        dev_err(&stat->client->dev, "update humidity resolution failed "
                "0x%02x,0x%02x: %d\n", buf[0], buf[1], err);
        return err;
}
 
static int hts221_validate_polling(unsigned int *min_interval,
                                        unsigned int *poll_interval,
                                        unsigned int min,
                                        struct i2c_client *client)
{
        *min_interval = max(min, *min_interval);
        *poll_interval = max(*poll_interval, *min_interval);
 
        return 0;
}
 
 
static int hts221_validate_pdata(struct hts221_status *stat)
{
        hts221_validate_polling(&stat->pdata->min_interval,
                                &stat->pdata->poll_interval,
                                (unsigned int)HTS221_MIN_POLL_PERIOD_MS,
                                stat->client);
 
        return 0;
}
 
static int hts221_enable(struct hts221_status *stat)
{
        int err;
 
        if (!atomic_cmpxchg(&stat->enabled, 0, 1)) {
                err = hts221_device_power_on(stat);
                if (err < 0) {
                        atomic_set(&stat->enabled, 0);
                        return err;
                }
                hrtimer_start(&stat->hr_timer, stat->ktime, HRTIMER_MODE_REL);
        }
 
        return 0;
}
 
static int hts221_disable(struct hts221_status *stat)
{
        if (atomic_cmpxchg(&stat->enabled, 1, 0)) {
                cancel_work_sync(&stat->input_work);
                hrtimer_cancel(&stat->hr_timer);
                hts221_device_power_off(stat);
        }
 
        return 0;
}
 
static int hts221_update_odr(struct hts221_status *stat,
                        unsigned int poll_interval_ms)
{
        stat->ktime = ktime_set(0, MS_TO_NS(poll_interval_ms));
        return 0;
}
 
static ssize_t attr_get_polling_rate(struct device *dev, struct device_attribute *attr,
                char *buf)
{
        unsigned int val;
        struct hts221_status *stat = dev_get_drvdata(dev);
        mutex_lock(&stat->lock);
        val = stat->pdata->poll_interval;
        mutex_unlock(&stat->lock);
        return sprintf(buf, "%u\n", val);
}
 
static ssize_t attr_set_polling_rate(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t size)
{
        struct hts221_status *stat = dev_get_drvdata(dev);
        unsigned long interval_ms;
 
        if (strict_strtoul(buf, 10, &interval_ms))
                return -EINVAL;
        if (!interval_ms)
                return -EINVAL;
        interval_ms = (unsigned int)max((unsigned int)interval_ms,
                                                stat->pdata->min_interval);
        mutex_lock(&stat->lock);
        stat->pdata->poll_interval = (unsigned int)interval_ms;
        hts221_update_odr(stat, stat->pdata->poll_interval);
        mutex_unlock(&stat->lock);
        return size;
}
 
static ssize_t attr_get_enable(struct device *dev, struct device_attribute *attr,
                char *buf)
{
        struct hts221_status *stat = dev_get_drvdata(dev);
        int val = (int)atomic_read(&stat->enabled);
        return sprintf(buf, "%d\n", val);
}
 
static ssize_t attr_set_enable(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t size)
{
        struct hts221_status *stat = dev_get_drvdata(dev);
        unsigned long val;
 
        if (strict_strtoul(buf, 10, &val))
                return -EINVAL;
 
        if (val)
                hts221_enable(stat);
        else
                hts221_disable(stat);
 
        return size;
}
 
static ssize_t attr_get_resolution_h(struct device *dev, 
                                struct device_attribute *attr, char *buf)
{
        u8 val;
        int i;
        struct hts221_status *stat = dev_get_drvdata(dev);
        int resolution = 2;
        mutex_lock(&stat->lock);
        val = stat->pdata->h_resolution;
        for(i=0;i<ARRAY_SIZE(resolution_h);i++)
                if(resolution==resolution_h[i].resolution)
                        break;
        mutex_unlock(&stat->lock);
        return sprintf(buf, "%d\n", resolution);
}
 
static ssize_t attr_set_resolution_h(struct device *dev, 
                struct device_attribute *attr, const char *buf, size_t size)
{
        struct hts221_status *stat = dev_get_drvdata(dev);
        unsigned long val;
        int i;
        int err = -1;
        if (strict_strtoul(buf, 10, &val))
                return -EINVAL;
 
        for(i=0;i<ARRAY_SIZE(resolution_h);i++)
                if(val==resolution_h[i].resolution) {
                        err = 1;
                        break;
                }
        if(err<0)
                goto error;
 
        mutex_lock(&stat->lock);
        err = hts221_update_resolution(stat, resolution_h[i].value, 0);
        if (err < 0) {
                mutex_unlock(&stat->lock);
                return err;
        }
        stat->pdata->h_resolution = resolution_h[i].resolution;
        mutex_unlock(&stat->lock);
        dev_info(&stat->client->dev, "humidity resolution set to:"
                                                        " %lu \n", val);
        return size;
 
error:
dev_err(&stat->client->dev, "humidity invalid range "
                                        "request: %lu, discarded\n", val);
        return size;
}
 
static ssize_t attr_get_resolution_t(struct device *dev, 
                                struct device_attribute *attr, char *buf)
{
        u8 val;
        int i;
        struct hts221_status *stat = dev_get_drvdata(dev);
        int resolution = 2;
        mutex_lock(&stat->lock);
        val = stat->pdata->t_resolution;
        for(i=0;i<ARRAY_SIZE(resolution_t);i++)
                if(resolution==resolution_t[i].resolution)
                        break;
        mutex_unlock(&stat->lock);
        return sprintf(buf, "%d\n", resolution);
}
 
static ssize_t attr_set_resolution_t(struct device *dev, 
                struct device_attribute *attr, const char *buf, size_t size)
{
        struct hts221_status *stat = dev_get_drvdata(dev);
        unsigned long val;
        int i;
        int err = -1;
        if (strict_strtoul(buf, 10, &val))
                return -EINVAL;
 
        for(i=0;i<ARRAY_SIZE(resolution_t);i++)
                if(val==resolution_t[i].resolution) {
                        err = 1;
                        break;
                }
        if(err<0)
                goto error;
 
        mutex_lock(&stat->lock);
        err = hts221_update_resolution(stat, resolution_t[i].value, 1);
        if (err < 0) {
                mutex_unlock(&stat->lock);
                return err;
        }
        stat->pdata->t_resolution = resolution_t[i].resolution;
        mutex_unlock(&stat->lock);
        dev_info(&stat->client->dev, "temperature resolution set to:"
                                                        " %lu \n", val);
        return size;
 
error:
dev_err(&stat->client->dev, "temperature invalid range "
                                        "request: %lu, discarded\n", val);
        return size;
}
 
#ifdef DEBUG
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t size)
{
    int rc;
    struct hts221_status *stat = dev_get_drvdata(dev);
    u8 x[2];
    unsigned long val;

    if (strict_strtoul(buf, 16, &val))
        return -EINVAL;
    mutex_lock(&stat->lock);
    x[0] = stat->reg_addr;
    mutex_unlock(&stat->lock);
    x[1] = val;
    rc = hts221_i2c_write(stat, x, 1);
    return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
                char *buf)
{
    ssize_t ret;
    struct hts221_status *stat = dev_get_drvdata(dev);
    int rc;
    u8 data;

    mutex_lock(&stat->lock);
    data = stat->reg_addr;
    mutex_unlock(&stat->lock);
    rc = hts221_i2c_read(stat, &data, 1);
    ret = sprintf(buf, "0x%02x\n", data);
    return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t size)
{
    struct hts221_status *stat = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 16, &val))
        return -EINVAL;

    mutex_lock(&stat->lock);

    stat->reg_addr = val;

    mutex_unlock(&stat->lock);

    return size;
}
#endif /* DEBUG */
 
static struct device_attribute attributes[] = {
        __ATTR(pollrate_ms, 0666, attr_get_polling_rate,
                                                attr_set_polling_rate),
        __ATTR(humidity_resolution, 0644, attr_get_resolution_h, 
                                                attr_set_resolution_h),
        __ATTR(temperature_resolution, 0644, attr_get_resolution_t, 
                                                attr_set_resolution_t),
        __ATTR(enable_device, 0666, attr_get_enable, attr_set_enable),
#ifdef DEBUG
        __ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
        __ATTR(reg_addr, 0200, NULL, attr_addr_set),
#endif
};
 
static int create_sysfs_interfaces(struct device *dev)
{
        int i;
        for (i = 0; i < ARRAY_SIZE(attributes); i++)
                if (device_create_file(dev, attributes + i))
                        goto error;
        return 0;
 
error:
        for (; i >= 0; i--)
                device_remove_file(dev, attributes + i);
        dev_err(dev, "%s:Unable to create interface\n", __func__);
        return -1;
}
 
static void remove_sysfs_interfaces(struct device *dev)
{
        int i;
        for (i = 0; i < ARRAY_SIZE(attributes); i++)
                device_remove_file(dev, attributes + i);
}
 
int hts221_input_open(struct input_dev *input)
{
        struct hts221_status *stat = input_get_drvdata(input);
 
        return hts221_enable(stat);
}
 
void hts221_input_close(struct input_dev *dev)
{
        struct hts221_status *stat = input_get_drvdata(dev);
 
        hts221_disable(stat);
}

/*****************************************************
 * Linear interpolation: (x0,y0) (x1,y1) y = ax+b
 *
 * a = (y1-y0)/(x1-x0)
 * b = (x1*y0-x0*y1)/(x1-x0)
 * 
 * result = ((y1-y0)*x+((x1*y0)-(x0*y1)))/(x1-x0)  
 * 
 * For Humidity
 * (x1,y1) = (H1_T0_OUT, H1_RH)
 * (x0,y0) = (H0_T0_OUT, H0_RH)
 * x       =  H_OUT
 * 
 * For Temperature  
 * (x1,y1) = (T1_OUT, T1_DegC)
 * (x0,y0) = (T0_OUT, T0_DegC)
 * x       =  T_OUT         
******************************************************/


// static int hts221_convert(int x1, u16 y1, int x0, u16 y0, int *x, int type)
static int hts221_convert(int slope, int b_gen, int *x, int type)
{
        int err = 0;
        int X = 0;
#ifdef DEBUG        
        //pr_info("%s: Covert process\n", HTS221_DEV_NAME);
#endif
        X = *x;
        
//        *x = ((y1-y0)*X+((x1*y0)-(x0*y1)))/(x1-x0);
        *x = ((slope*X)+b_gen);

        
//        /*do not use float in linux driver, so I multiplied by 1000 at least; in this case 8000 */
        if (type == 0)
//          *x = *x / 2; /*for Humidity, m RH*/   previous case, when mltiplied by 1000
//          *x = *x / 16; /*for Humidity, m RH*/    // actual
            *x = (*x) >>4;  /*for Humidity, m RH*/

        else 
//          *x = *x / 8; /*for Temperature*/    // previous    case
//          *x = *x / 64; /*for Temperature*/           // actual          
            *x = (*x) >>6;  /*for Humidity, m RH*/

        
        return err; 
}
 
static int hts221_get_data(struct hts221_status *stat, int *data_h, int *data_t)
{
        int err = -1;
        u8 humidity_data[2];
        u8 temperature_data[2];
         
        u8 tmp[2];
 
        tmp[0] = REG_CNTRL2_ADDR;
        tmp[1] = 0x01;
        err = hts221_i2c_write(stat, tmp, 1);
        if (err < 0)
                return err;

        humidity_data[0] = REG_H_OUT_L;
        err = hts221_i2c_read(stat, humidity_data, 2);
        if (err < 0)
                return err;
        *data_h = ((s32)( (s16)((humidity_data[1] << 8) | (humidity_data[0]))));           

      
#ifdef DEBUG
        pr_info("\n%s read H regs: x=%X %X(regH regL)\n",
                        HTS221_DEV_NAME, humidity_data[1], humidity_data[0]);
#endif                        

#ifdef DEBUG
        pr_info("%s Before conversion H=%d - %X \n", HTS221_DEV_NAME, *data_h, *data_h);
#endif                       
 
       
        /*convert for Humidity*/
//        err = hts221_convert(calibX1, calibY1, calibX0, calibY0, data_h, 0);
        err = hts221_convert(h_slope, h_b, data_h, 0);

        if (err < 0)
                return err;              
 
        temperature_data[0] = REG_T_OUT_L;
        err = hts221_i2c_read(stat, temperature_data, 2);
        if (err < 0)
                return err;
 
        *data_t = ((s32)( (s16)((temperature_data[1] << 8) | 
                                                (temperature_data[0]))));

#ifdef DEBUG
        pr_info("%s read T regs: x=%X %X(regH regL)\n",
                HTS221_DEV_NAME, temperature_data[1], temperature_data[0]);
#endif                                                

#ifdef DEBUG
        pr_info("%s Before conversion T=%d - %X \n", HTS221_DEV_NAME, *data_t, *data_t);
#endif                       
 
            
        /*convert for Tempeerature*/
//        err = hts221_convert(calibX1, calibY1, calibX0, calibY0, data_t, 1);
        err = hts221_convert(t_slope, t_b, data_t, 1);
        
        if (err < 0)
                return err;   
                
#ifdef DEBUG
        pr_info("%s Values: H=%d T=%d\n",
                        HTS221_DEV_NAME, *data_h, *data_t);
#endif                       
         
        return err;
}
 
static void hts221_report_values(struct hts221_status *stat, int *data_h, 
                                                                int *data_t)
{
        input_report_abs(stat->input_dev_humidity, ABS_MISC, *data_h);
        input_report_abs(stat->input_dev_temperature, ABS_MISC, *data_t);
        
        input_sync(stat->input_dev_humidity);
        input_sync(stat->input_dev_temperature);
}
 
static int hts221_input_init(struct hts221_status *stat)
{
        int err;
 
        stat->input_dev_humidity = input_allocate_device();
        if (!stat->input_dev_humidity) {
                err = -ENOMEM;
                dev_err(&stat->client->dev, "humidity "
                                        "input device allocation failed\n");
                goto err0;
        }
 
        stat->input_dev_humidity->open = hts221_input_open;
        stat->input_dev_humidity->close = hts221_input_close;
        stat->input_dev_humidity->name = HTS221_DEV_NAME_H;
        stat->input_dev_humidity->id.bustype = BUS_I2C;
        stat->input_dev_humidity->dev.parent = &stat->client->dev;
 
        input_set_drvdata(stat->input_dev_humidity, stat);
 
        set_bit(EV_ABS, stat->input_dev_humidity->evbit);
 
        input_set_abs_params(stat->input_dev_humidity, ABS_MISC,
                                -MAG_MAX_NEG, MAG_MAX_POS, 0, 0);
 
 
        err = input_register_device(stat->input_dev_humidity);
        if (err) {
                dev_err(&stat->client->dev,
                        "unable to register temperature input device %s\n",
                                stat->input_dev_humidity->name);
                goto err1;
        }
 
        stat->input_dev_temperature = input_allocate_device();
        if (!stat->input_dev_temperature) {
                err = -ENOMEM;
                dev_err(&stat->client->dev, "temperature "
                                        "input device allocation failed\n");
                goto err1;
        }
 
        stat->input_dev_temperature->open = hts221_input_open;
        stat->input_dev_temperature->close = hts221_input_close;
        stat->input_dev_temperature->name = HTS221_DEV_NAME_T;
        stat->input_dev_temperature->id.bustype = BUS_I2C;
        stat->input_dev_temperature->dev.parent = &stat->client->dev;
 
        input_set_drvdata(stat->input_dev_temperature, stat);
 
        set_bit(EV_ABS, stat->input_dev_temperature->evbit);
 
        input_set_abs_params(stat->input_dev_temperature, ABS_MISC,
                                -MAG_MAX_NEG, MAG_MAX_POS, 0, 0);
 
 
        err = input_register_device(stat->input_dev_temperature);
        if (err) {
                dev_err(&stat->client->dev,
                        "unable to register temperature input device %s\n",
                                stat->input_dev_temperature->name);
                goto err2;
        }
 
        return 0;
 
err2:
        input_free_device(stat->input_dev_temperature);
err1:
        input_free_device(stat->input_dev_humidity);
err0:
        return err;
}
 
static void hts221_input_cleanup(struct hts221_status *stat)
{
        input_unregister_device(stat->input_dev_humidity);
        input_free_device(stat->input_dev_humidity);
        input_unregister_device(stat->input_dev_temperature);
        input_free_device(stat->input_dev_temperature);
}
 
static void poll_function_work(struct work_struct *input_work)
{
        struct hts221_status *stat;
        int data_h = 0;
        int data_t = 0;
        int err;
 
        stat = container_of((struct work_struct *)input_work,
                        struct hts221_status, input_work);
 
        mutex_lock(&stat->lock);
 
        if(atomic_read(&stat->enabled)) {
                err = hts221_get_data(stat, &data_h, &data_t);
                if (err < 0)
                        dev_err(&stat->client->dev, "get data failed\n");
                else
                        hts221_report_values(stat, &data_h, &data_t);
        }
        mutex_unlock(&stat->lock);
        hrtimer_start(&stat->hr_timer, stat->ktime, HRTIMER_MODE_REL);
}
 
enum hrtimer_restart poll_function_read(struct hrtimer *timer)
{
        struct hts221_status *stat;
 
 
        stat = container_of((struct hrtimer *)timer,
                                struct hts221_status, hr_timer);
 
        queue_work(hts221_workqueue, &stat->input_work);
        return HRTIMER_NORESTART;
}
 
static int hts221_probe(struct i2c_client *client,
                const struct i2c_device_id *id)
{
        struct hts221_status *stat;
 
        u32 smbus_func = I2C_FUNC_SMBUS_BYTE_DATA |
                        I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK;
 
        int err = -1;
        dev_info(&client->dev, "probe start.\n");
        stat = kzalloc(sizeof(struct hts221_status), GFP_KERNEL);
        if (stat == NULL) {
                err = -ENOMEM;
                dev_err(&client->dev,
                                "failed to allocate memory for module data: "
                                        "%d\n", err);
                goto exit_check_functionality_failed;
        }
 
        stat->use_smbus = 0;
        if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
                dev_warn(&client->dev, "client not i2c capable\n");
                if (i2c_check_functionality(client->adapter, smbus_func)){
                        stat->use_smbus = 1;
                        dev_warn(&client->dev, "client using SMBUS\n");
                } else {
                        err = -ENODEV;
                        dev_err(&client->dev, "client nor SMBUS capable\n");
                        goto exit_check_functionality_failed;
                }
        }
 
        if(hts221_workqueue == 0)
                hts221_workqueue = create_workqueue("hts221_workqueue");
 
        hrtimer_init(&stat->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        stat->hr_timer.function = &poll_function_read;
 
        mutex_init(&stat->lock);
        mutex_lock(&stat->lock);
 
        stat->client = client;
        i2c_set_clientdata(client, stat);
 
        stat->pdata = kmalloc(sizeof(*stat->pdata), GFP_KERNEL);
        if(stat->pdata == NULL) {
                err = -ENOMEM;
                dev_err(&client->dev,
                        "failed to allocate memory for pdata: %d\n", err);
                goto err_mutexunlock;
        }
 
        if (client->dev.platform_data == NULL) {
                memcpy(stat->pdata, &default_hts221_pdata,
                                                sizeof(*stat->pdata));
                dev_info(&client->dev, "using default plaform_data for "
                                                "humidity\n");
        } else {
                memcpy(stat->pdata, client->dev.platform_data,
                                                sizeof(*stat->pdata));
                dev_info(&client->dev, "using user plaform_data for "
                                                "humidity\n");
        }
 
        err = hts221_validate_pdata(stat);
        if (err < 0) {
                dev_err(&client->dev, "failed to validate platform data for "
                                                        "humidity\n");
                goto exit_kfree_pdata;
        }
 
        if (stat->pdata->init) {
                err = stat->pdata->init();
                if (err < 0) {
                        dev_err(&client->dev, "humidity init failed: "
                                                                "%d\n", err);
                        goto err_pdata_init;
                }
        }
 
        err = hts221_hw_init(stat);
        if (err < 0) {
                dev_err(&client->dev, "hw init failed: %d\n", err);
                goto err_hw_init;
        }
 
        hts221_update_odr(stat, stat->pdata->poll_interval);
 
        err = hts221_device_power_on(stat);
        if (err < 0) {
                dev_err(&client->dev, "humidity power on failed: "
                                                                "%d\n", err);
                goto err_pdata_init;
        }
        err = hts221_update_resolution(stat, stat->pdata->h_resolution, 0);
        if (err < 0) {
                dev_err(&client->dev, "update_fs_range on humidity "
                                                                "failed\n");
                goto  err_power_off;
        }
        err = hts221_update_resolution(stat, stat->pdata->t_resolution, 1);
        if (err < 0) {
                dev_err(&client->dev, "update_fs_range on humidity "
                                                                "failed\n");
                goto  err_power_off;
        }
        err = hts221_input_init(stat);
        if (err < 0) {
                dev_err(&client->dev, "humidity input init failed\n");
                goto err_power_off;
        }
        err = create_sysfs_interfaces(&client->dev);
        if (err < 0) {
                dev_err(&client->dev,
                "device HTS221_DEV_NAME sysfs register failed\n");
                goto err_input_cleanup;
        }
        hts221_device_power_off(stat);
        
        // Reading H & T calibration LS

        humidity_calibration[0] = REG_0RH_CAL_X_H;
        err = hts221_i2c_read(stat, humidity_calibration, 2);
        if (err < 0)
                return err;
        calibX0 = ((s32)( (s16)((humidity_calibration[1] << 8) |
                                                (humidity_calibration[0]))));                                                  
 
        humidity_calibration[0] = REG_1RH_CAL_X_H;
        err = hts221_i2c_read(stat, humidity_calibration, 2);
        if (err < 0)
                return err;
        calibX1 = ((s32)( (s16)((humidity_calibration[1] << 8) |
                                                (humidity_calibration[0])))); 
                                                  
        humidity_calibration[0] = REG_0RH_CAL_Y_H;
        err = hts221_i2c_read(stat, humidity_calibration, 1);
        if (err < 0)
                return err;
        calibY0 = (u16) humidity_calibration[0];          
        
        humidity_calibration[0] = REG_1RH_CAL_Y_H;
        err = hts221_i2c_read(stat, humidity_calibration, 1);
        if (err < 0)
                return err;
        calibY1 = (u16) humidity_calibration[0];                                                       

        h_slope=((calibY1-calibY0)*8000)/(calibX1-calibX0);
        h_b=(((calibX1*calibY0)-(calibX0*calibY1))*1000)/(calibX1-calibX0);
        h_b=h_b*8;

        temperature_calibration[0] = REG_0T_CAL_X_L;
        err = hts221_i2c_read(stat, temperature_calibration, 2);
        if (err < 0)
                return err;
        calibW0 = ((s32)( (s16)((temperature_calibration[1] << 8) |
                                        (temperature_calibration[0]))));
 
        temperature_calibration[0] = REG_1T_CAL_X_L;
        err = hts221_i2c_read(stat, temperature_calibration, 2);
        if (err < 0)
                return err;
        calibW1 = ((s32)( (s16)((temperature_calibration[1] << 8) |
                                        (temperature_calibration[0]))));
 

        temperature_calibration[0] = REG_0T_CAL_Y_H;
        err = hts221_i2c_read(stat, temperature_calibration, 1);
        if (err < 0)
                return err;
        calibZ0a = (u16) temperature_calibration[0];  

        temperature_calibration[0] = REG_T1_T0_CAL_Y_H;
        err = hts221_i2c_read(stat, temperature_calibration, 1);
        if (err < 0)
                return err;
        calibZ0b = (u16) ( temperature_calibration[0] & (0x3) );
        calibZ0 = (( s32)( (calibZ0b << 8) | (calibZ0a)));     // LS  
        
        temperature_calibration[0] = REG_1T_CAL_Y_H;
        err = hts221_i2c_read(stat, temperature_calibration, 1);
        if (err < 0)
                return err;
        calibZ1a = (u16) temperature_calibration[0];  

        temperature_calibration[0] = REG_T1_T0_CAL_Y_H;
        err = hts221_i2c_read(stat, temperature_calibration, 1);
        if (err < 0)
                return err;
        calibZ1b = (u16)( temperature_calibration[0] & (0xC) );  
        calibZ1b = calibZ1b >> 2;                        // LS
        calibZ1 = ((s32) ( (calibZ1b << 8) | (calibZ1a)));

        t_slope=((calibZ1-calibZ0)*8000)/(calibW1-calibW0);
        t_b=(((calibW1*calibZ0)-(calibW0*calibZ1))*1000)/(calibW1-calibW0);
        t_b=t_b*8;

#ifdef DEBUG
        pr_info(" reading calibX0=%X calibX1=%X \n", calibX0, calibX1);
        pr_info(" reading calibX0=%d calibX1=%d \n", calibX0, calibX1);
        pr_info(" reading calibW0=%X calibW1=%X \n", calibW0, calibW1);
        pr_info(" reading calibW0=%d calibW1=%d \n", calibW0, calibW1);
        pr_info(" reading calibY0=%X calibY1=%X \n", calibY0, calibY1);
        pr_info(" reading calibY0=%u calibY1=%u \n", calibY0, calibY1);
        pr_info(" reading calibZ0a=%X calibZ0b=%X calibZ0=%X \n", calibZ0a, calibZ0b, calibZ0);
        pr_info(" reading calibZ0a=%u calibZ0b=%u calibZ0=%d \n", calibZ0a, calibZ0b, calibZ0);
        pr_info(" reading calibZ1a=%X calibZ1b=%X calibZ1=%X \n", calibZ1a, calibZ1b, calibZ1);
        pr_info(" reading calibZ1a=%u calibZ1b=%u calibZ1=%d \n", calibZ1a, calibZ1b, calibZ1);
        pr_info(" reading t_slope=%X t_b=%X h_slope=%X h_b=%X \n", t_slope, t_b, h_slope, h_b );
        pr_info(" reading t_slope=%d t_b=%d h_slope=%d h_b=%d \n", t_slope, t_b, h_slope, h_b );
#endif                        
 
        INIT_WORK(&stat->input_work, poll_function_work);
 
        mutex_unlock(&stat->lock);
        dev_info(&client->dev, "%s: probed\n", HTS221_DEV_NAME);
        return 0;
 
err_input_cleanup:
        hts221_input_cleanup(stat);
err_power_off:
        hts221_device_power_off(stat);
err_hw_init:
err_pdata_init:
        if (stat->pdata->exit)
                stat->pdata->exit();
exit_kfree_pdata:
        kfree(stat->pdata);
err_mutexunlock:
        mutex_unlock(&stat->lock);
        kfree(stat);
        if(!hts221_workqueue) {
                flush_workqueue(hts221_workqueue);
                destroy_workqueue(hts221_workqueue);
        }
exit_check_functionality_failed:
        pr_err("%s: Driver Init failed\n", HTS221_DEV_NAME);
        return err;
}
 
static int __devexit hts221_remove(struct i2c_client *client)
{
        struct hts221_status *stat = i2c_get_clientdata(client);
 
        hts221_disable(stat);
        hts221_input_cleanup(stat);
 
        remove_sysfs_interfaces(&client->dev);
 
        if (stat->pdata->exit)
                stat->pdata->exit();
 
        if(!hts221_workqueue) {
                flush_workqueue(hts221_workqueue);
                
                destroy_workqueue(hts221_workqueue);
        }
 
        kfree(stat->pdata);
        kfree(stat);
        return 0;
}

#ifdef CONFIG_OF
static struct of_device_id hts221_match_table[] = {
    { .compatible = "hts221",},
    { },
};
#else
#define hts221_match_table NULL
#endif

static const struct i2c_device_id hts221_id[]
                                        = { { HTS221_DEV_NAME, 0 }, { }, };
 
MODULE_DEVICE_TABLE(i2c, hts221_id);
 
static struct i2c_driver hts221_driver = {
        .driver = {
                        .owner = THIS_MODULE,
                        .name = HTS221_DEV_NAME,
                    #ifdef CONFIG_OF
                        .of_match_table = hts221_match_table,
                    #endif
                  },
        .probe = hts221_probe,
        .remove = __devexit_p(hts221_remove),
        .id_table = hts221_id,
};
 
static int __init hts221_init(void)
{
        pr_info("%s driver: init\n", HTS221_DEV_NAME);
        return i2c_add_driver(&hts221_driver);
}
 
static void __exit hts221_exit(void)
{
        pr_info("%s driver exit\n", HTS221_DEV_NAME);
        i2c_del_driver(&hts221_driver);
}
 
module_init(hts221_init);
module_exit(hts221_exit);
 
MODULE_DESCRIPTION("hts221 humidity driver");
MODULE_AUTHOR("Lorenzo SARCHI, Morris Chen, STMicroelectronics");
MODULE_LICENSE("GPL");
