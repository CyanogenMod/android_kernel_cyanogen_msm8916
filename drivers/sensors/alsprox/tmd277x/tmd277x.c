/****************************************************************************/
/*                                                                          */
/*             Copyright (c) 2011-2012  YULONG Company                      */
/*                                                                          */
/* PROPRIETARY RIGHTS of YULONG Company are involved in the                 */
/* subject matter of this material.  All manufacturing, reproduction, use,  */
/* and sales rights pertaining to this subject matter are governed by the   */
/* license agreement.  The recipient of this software implicitly accepts    */
/* the terms of the license.                                                */
/*                                                                          */
/****************************************************************************/

/*****************************************************************************
**  FileName   : tmd277x.c
**  Author     : Jay.HF
**  Version    : 1.00
**  Date       : 2012-11-11
**  Description: light and proximity sensors driver
**  History    :
**  <author>     <time>      <version >      <desc>
**  Jay.HF       2012-11-11  1.00            create
*****************************************************************************/

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/timer.h>
#include <asm/errno.h>
#include <asm/delay.h>
#include <linux/delay.h>
#include <linux/irq.h> 
#include <linux/interrupt.h> 
#include <linux/slab.h>
#include <mach/gpio.h> 
#include <linux/poll.h> 
#include <linux/wakelock.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
//#include <linux/earlysuspend.h>
#include <linux/sensors/tmd277x.h>
#include <linux/gpio.h>
#include <asm/mach/time.h>
//#include <linux/android_alarm.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
//#include <linux/sysdev.h>
#include <linux/uaccess.h>
#include <linux/hardirq.h>
#include <linux/workqueue.h>
#include <linux/cpumask.h>
#include <linux/smp.h>
#include <linux/yl_params.h>
#include <linux/sensors/sensparams.h>

#ifdef CONFIG_YULONG_MODULE_MANAGEMENT
#include <yl_debug/management.h>
#endif

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

#define TAOS_PROX_CROSSTALK_MAX   600

#define TAOS_MAX(a,b)             ((a)>(b)?(a):(b))

struct taos_dev {
    struct i2c_client *client;
    struct input_dev  *input_dev_light;
    struct input_dev  *input_dev_proximity;
    struct work_struct irq_work;
    //struct work_struct probe_work;
    wait_queue_head_t  wqh_suspend;
    struct wake_lock   wake_lock;
    struct wake_lock   wake_lock_timeout;
    unsigned int       als_ps_int;
    struct taos_platform_data *pdata;
    unsigned short     sat_als;
    unsigned short     sat_prox;
    int                prox_on;
    int                als_on;
    int                prox_enabled;
    int                als_enabled;
    bool               debug_enabled;
    int                suspend_flag;
    int                polling;
    unsigned int       phone_state;
    int                data_ready;
#ifdef CONFIG_YULONG_MODULE_MANAGEMENT
    u8                 pm_state; /* the current pm state, add by Jay.HF 2012-12-06 */
#endif
    u8                 chip_id; /* add by Jay.HF, 2013-03-07 */
    u8                 prox_on_ok; /* add by Jay.HF, 2013-04-16 */
    u16                prox_factory_threshold_hi;
    u16                prox_factory_threshold_lo;
//longjiang add start 20131017
#ifdef CONFIG_SENSOR_POWER
    struct regulator *vcc_i2c;
    struct regulator *vdd_ana;
#endif
//add end
};
static struct taos_dev *tmd277x_dev = NULL;

bool isProximityEnabled = false;

u8 taos_triton_reg_init[16] = {
    0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 
    0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00
};
u8 taos_triton_gain_table[] = {1, 8, 16, 120};

struct time_scale_factor {
    u16 numerator;
    u16 denominator;
    u16 saturation;
};
struct time_scale_factor TritonTime = {1, 0, 0};
struct time_scale_factor *lux_timep = &TritonTime;

struct lux_data {
    u16 ratio;
    u16 clear;
    u16 ir;
};
struct lux_data TritonFN_lux_data[] = {
    { 9830,  8320,  15360 },
    { 12452, 10554, 22797 },
    { 14746, 6234,  11430 },
    { 17695, 3968,  6400  },
    { 0,     0,     0     }
};
struct lux_data *lux_tablep = TritonFN_lux_data;

//longjiang add start 20131017
#ifdef CONFIG_SENSOR_POWER
static int sensors_power_on(struct taos_dev *data, bool on)
{
    int rc, ret;

    if (!on)
        goto power_off;

    rc = regulator_enable(data->vdd_ana);
    if (rc) {
        dev_err(&data->client->dev,
            "Regulator vdd_ana enable failed rc=%d\n", rc);
        return rc;
    }
    rc = regulator_enable(data->vcc_i2c);
    if (rc) {
        dev_err(&data->client->dev,
            "Regulator vcc_i2c enable failed rc=%d\n", rc);
        regulator_disable(data->vdd_ana);
    }
    return rc;

power_off:
    rc = regulator_disable(data->vdd_ana);
    if (rc) {
        dev_err(&data->client->dev,
            "Regulator vdd_ana disable failed rc=%d\n", rc);
        return rc;
    }
    rc = regulator_disable(data->vcc_i2c);
    if (rc) {
        dev_err(&data->client->dev,
            "Regulator vcc_i2c disable failed rc=%d\n", rc);
		ret = regulator_enable(data->vdd_ana);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vdd_ana enable failed ret=%d\n", ret);
			return ret;
		}
    }
    return rc;
}

static int sensors_power_init(struct taos_dev *data, bool on)
{
    int rc;

    if (!on)
        goto pwr_deinit;

    data->vdd_ana = regulator_get(&data->client->dev, "vdd_ana");
    if (IS_ERR(data->vdd_ana)) {
        rc = PTR_ERR(data->vdd_ana);
        dev_err(&data->client->dev,
            "Regulator get failed vdd_ana rc=%d\n", rc);
        return rc;
    }

    if (regulator_count_voltages(data->vdd_ana) > 0) {
        rc = regulator_set_voltage(data->vdd_ana, 2600000,3300000);
        if (rc) {
            dev_err(&data->client->dev,
                "Regulator set_vtg failed vdd_ana rc=%d\n", rc);
            goto reg_vdd_ana_put;
        }
    }

    data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
    if (IS_ERR(data->vcc_i2c)) {
        rc = PTR_ERR(data->vcc_i2c);
        dev_err(&data->client->dev,
            "Regulator get failed vcc_i2c rc=%d\n", rc);
        goto reg_vdd_ana_set_vtg;
    }

    if (regulator_count_voltages(data->vcc_i2c) > 0) {
        rc = regulator_set_voltage(data->vcc_i2c, 1800000,1800000);
        if (rc) {
            dev_err(&data->client->dev,
            "Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
            goto reg_vcc_i2c_put;
        }
    }
    return 0;

reg_vcc_i2c_put:
    regulator_put(data->vcc_i2c);

reg_vdd_ana_set_vtg:
    if (regulator_count_voltages(data->vdd_ana) > 0)
        regulator_set_voltage(data->vdd_ana, 0, 3300000);
reg_vdd_ana_put:
    regulator_put(data->vdd_ana);
    return rc;

pwr_deinit:
    if (regulator_count_voltages(data->vdd_ana) > 0)
        regulator_set_voltage(data->vdd_ana, 0, 3300000);

    regulator_put(data->vdd_ana);
    if (regulator_count_voltages(data->vcc_i2c) > 0)
        regulator_set_voltage(data->vcc_i2c, 0, 1800000);

    regulator_put(data->vcc_i2c);
    return 0;
}
#endif
//add end

#ifdef CONFIG_OF
static int taos_parse_configs(struct device *dev, char *name, u32 *array) {
    int i, rc;
    struct property *prop;
    struct device_node *np = dev->of_node;

    prop = of_find_property(np, name, NULL);
    if (!prop)
        return -EINVAL;
    if (!prop->value)
        return -ENODATA;

    rc = of_property_read_u32_array(np, name, array, prop->length/sizeof(u32));
    if (rc && (rc != -EINVAL)) {
        dev_err(dev, "%s: Unable to read %s\n", __func__, name);
        return rc;
    }

    dev_info(dev, "%s size is %d\n", name, prop->length/sizeof(u32));
    for (i = 0; i < prop->length/sizeof(u32); i++) {
        dev_info(dev, "arrary[%d]=%d, ", i, array[i]);
    }
    dev_info(dev, "\n");
    return rc;
}

static int taos_parse_dt(struct device *dev, struct taos_platform_data *pdata) {
    int ret = 0;
    int index = 0;
    u32 value;
    u32 array[24];
    enum of_gpio_flags flags;
    struct device_node *np = dev->of_node;

    pdata->init      = NULL; 
    pdata->exit      = NULL;
    pdata->power_on  = NULL;
    pdata->power_off = NULL;
    pdata->suspend   = NULL;
    pdata->resume    = NULL;

    /* parse irq and request gpio */
    value = of_get_named_gpio_flags(np, "taos,irq-gpio", 0, &flags);
    pdata->gpio_int = value;
    dev_info(dev, "%s: irq gpio num is %d\n", __func__, pdata->gpio_int);
    if (pdata->gpio_int < 0) {
        return -EINVAL;
    }
    
    //chenyz del the gpio request code here and move out,20140504 
    //gpio_request(pdata->gpio_int, "taos_irq");
    //gpio_direction_input(pdata->gpio_int);

    /* general_reg */
    ret = taos_parse_configs(dev, "taos,cfgs", array);
    if (ret) {
        dev_err(dev, "Looking up %s property in node %s failed", "taos,cfgs", np->full_name);
        return -ENODEV;
    }

    pdata->calibrate_target  = array[index++];
    pdata->als_time          = array[index++];
    pdata->scale_factor      = array[index++];
    pdata->gain_trim         = array[index++];
    pdata->filter_history    = array[index++];
    pdata->filter_count      = array[index++];
    pdata->gain              = array[index++];
    pdata->prox_threshold_hi = array[index++];
    pdata->prox_threshold_lo = array[index++];
    pdata->prox_avg_max      = array[index++];
    pdata->prox_avg_more     = array[index++];
    pdata->prox_avg_less     = array[index++];
    pdata->als_threshold_hi  = array[index++];
    pdata->als_threshold_lo  = array[index++];
    pdata->prox_int_time     = array[index++];
    pdata->prox_adc_time     = array[index++];
    pdata->prox_wait_time    = array[index++];
    pdata->prox_intr_filter  = array[index++];
    pdata->prox_config       = array[index++];
    pdata->prox_pulse_cnt    = array[index++];
    pdata->prox_gain         = array[index++];
    pdata->prox_offset_max   = array[index++];
    pdata->prox_offset       = 0;

    return 0;
}
#endif

static int taos_get_lux(void) {
#if TAOS_ALGO_OPTIMIZE
    u8 dev_gain = 0;
    u16 Tint = 0;
#endif

    int ret = 0, i = 0;
    u8 chdata[4];
    u32 lux = 0, max_lux = 0;
    u16 raw_clear = 0, raw_ir = 0;
#if 1
    u16 raw_lux = 0;
    u32 ratio = 0;
    int tmp = 0;
    struct lux_data *p;
#else
    u8 dev_gain = 0;
    int cpl;
    int lux1 = 0, lux2 = 0;
#endif
    for (i=0;i<4;i++) {
        if ((ret = (i2c_smbus_write_byte(tmd277x_dev->client, 
                (TAOS_TRITON_CMD_REG | (TAOS_TRITON_ALS_CHAN0LO + i))))) < 0) {
            printk(KERN_ERR"%s: i2c_write to ch0/1/lo/hi regs failed \n", __func__);
            return (ret);
        }

        chdata[i] = i2c_smbus_read_byte(tmd277x_dev->client);
    }

    if (TMD2772_CHIP_ID == tmd277x_dev->chip_id) {
        max_lux = TAOS_2772_MAX_LUX;
    }
    else {
        max_lux = TAOS_2771_MAX_LUX;
    }

    /* if atime =100  tmp = (atime+25)/50=2.5   time = 2.7*(256-atime)=  412.5 */
    tmp = (tmd277x_dev->pdata->als_time+25)/50;
    TritonTime.denominator = tmp;

    tmp = 300*tmd277x_dev->pdata->als_time; /* tmp = 300*atime  400 */
    if (tmp > 65535)
        tmp = 65535;
    TritonTime.saturation = tmp;
    raw_clear = chdata[1];
    raw_clear <<= 8;
    raw_clear |= chdata[0];
    raw_ir = chdata[3];
    raw_ir <<= 8;
    raw_ir |= chdata[2];

    if (raw_ir > raw_clear) {
        raw_lux = raw_ir;
        raw_ir = raw_clear;
        raw_clear = raw_lux;
    }
    //printk(KERN_ERR"%s: Cr(%d), Ir(%d)\n", __func__, raw_clear, raw_ir);

#if TAOS_ALGO_OPTIMIZE
    raw_clear *= (tmd277x_dev->pdata->scale_factor);
    raw_ir *= (tmd277x_dev->pdata->scale_factor);
    dev_gain = taos_triton_gain_table[tmd277x_dev->pdata->gain & 0x3];
#endif
    if (raw_clear >= lux_timep->saturation) {
        return(max_lux);
    }

    if (raw_ir >= lux_timep->saturation) {
        return(max_lux);
    }

    if (raw_clear == 0) {
        printk(KERN_ERR"%s: raw_clear == 0\n", __func__);
        return(0);
    }

#if TAOS_ALGO_OPTIMIZE
    if (dev_gain == 0 || dev_gain > 127) {
        printk(KERN_ERR"%s: dev_gain = 0 or > 127 in taos_get_lux()\n", __func__);
        return -1;
    }
#endif
    if (lux_timep->denominator == 0) {
        printk(KERN_ERR"%s: lux_timep->denominator = 0 in taos_get_lux()\n", __func__);
        return -1;
    }

    ratio = (raw_ir << 15) / raw_clear;
    //printk(KERN_ERR"%s: ratio(%d)\n", __func__, ratio);

    for (p = lux_tablep; p->ratio && p->ratio<ratio; p++);
    if (!p->ratio) {
        p--;
        //printk(KERN_ERR"%s: p->ratio == 0\n", __func__);
        //return 0;
    }

#if TAOS_ALGO_OPTIMIZE
    Tint = tmd277x_dev->pdata->als_time;
    raw_clear = ((raw_clear*400 + (dev_gain>>1))/dev_gain + (Tint>>1))/Tint;
    raw_ir = ((raw_ir*400 +(dev_gain>>1))/dev_gain + (Tint>>1))/Tint;
    lux = ((raw_clear*(p->clear)) - (raw_ir*(p->ir)));
    lux = (lux + 32000)/64000;
#else
    //lux = ((raw_clear*(p->clear)) - (raw_ir*(p->ir)) + 8000)/16000;
    lux = ((raw_clear*(p->clear)) - (raw_ir*(p->ir)) + 12000)/5000;
#endif

    if (lux > max_lux) {
        lux = max_lux;
        return(lux);
    }

    if (tmd277x_dev->debug_enabled) {
        printk(KERN_INFO"%s: lux(%d)\n", __func__, lux);
    }
    return(lux);
}

static int taos_als_threshold_set(void) {
    u8 chdata[2];
    u16 ch0, thresh_h, thresh_l;
    u8 buf[4];
    int i, count, ret = 0;

    for (i = 0; i < 2; i++) {
        chdata[i] = (i2c_smbus_read_byte_data(tmd277x_dev->client, 
                    (TAOS_TRITON_CMD_REG | (TAOS_TRITON_ALS_CHAN0LO + i))));
    }

    ch0 = chdata[0] + ((u16)chdata[1]<<8);
#if 0
    /* (25, 10)(70, 30)(93, 40) */
    if (ch0 < 25) {
        thresh_h = 93;
        thresh_l = 0;
    }
    else{
        thresh_h = (16*ch0)/10;
        if (thresh_h >= 65535)
            thresh_h = 65535;
        if (thresh_h < 93)//(25,10)(70,30)(93,40)
            thresh_h = 93;
        thresh_l = (6*ch0)/10;
    }
#else
    thresh_h = (12/*15*/*ch0)/10;
    if (thresh_h >= 65535)
        thresh_h = 65535;
    thresh_l = (8/*6*/*ch0)/10;
#endif

#if 0
    if (tmd277x_dev->debug_enabled) {
        printk(KERN_INFO"%s: ch0(%d), thr_l(%d) thr_h(%d)\n", __func__,
            ch0, thresh_l, thresh_h);
    }
#endif

    buf[0] = thresh_l & 0x0ff;
    buf[1] = thresh_l >> 8;
    buf[2] = thresh_h & 0x0ff;
    buf[3] = thresh_h >> 8;

    for (count=0; count<4; count++) {
        ret = i2c_smbus_write_byte_data(tmd277x_dev->client, 
                (TAOS_TRITON_CMD_REG|TAOS_TRITON_ALS_MINTHRESHLO) + count, buf[count]);
        if (ret < 0) {
            printk(KERN_ERR"%s: ALS threshold set fail\n", __func__);
            return (ret);
        }
    }

    return ret;
}

static int taos_als_default_threshold_set(void) {
    u8 buf[4];
    int count, ret = 0;
    //int lux_val = 0;

    buf[0] = 0xff;
    buf[1] = 0xff;
    buf[2] = 0x00;
    buf[3] = 0x00;

    printk(KERN_ERR"%s\n", __func__);

    /* add for the first time no value report on the night, by Jay.HF */
    if (tmd277x_dev->als_on) {
        tmd277x_dev->als_on = 0;
	#if 0
        if ((lux_val = taos_get_lux()) < 0) {
            printk(KERN_ERR"%s: returned error [%d] \n", __func__, lux_val);
            goto out;
        }

        input_event(tmd277x_dev->input_dev_light, EV_MSC, MSC_SCAN, lux_val);
        //input_report_abs(tmd277x_dev->input_dev_light, ABS_MISC, lux_val);
        input_sync(tmd277x_dev->input_dev_light);
        printk(KERN_ERR"%s: als %d for the first time open\n", __func__, lux_val);
	#endif
    }
    /* end */

//out:
    for (count=0; count<4; count++) {
        ret = i2c_smbus_write_byte_data(tmd277x_dev->client, 
                (TAOS_TRITON_CMD_REG|TAOS_TRITON_ALS_MINTHRESHLO) + count, buf[count]);
        if (ret < 0) {
            printk(KERN_ERR"%s: ALS set fail\n", __func__);
            return (ret);
        }
    }

    return ret;
}

static int taos_prox_threshold_set(void) {
    u8 chdata[4];
    u16 proxdata = 0;
    u16 cleardata = 0;
    u16 cleartemp = 0;
    u8 pro_buf[4];
    int i, ret = 0;
    int distance = 0;

    chdata[0] = i2c_smbus_read_byte_data(tmd277x_dev->client, 
        (TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_CHAN0LO));
    chdata[1] = i2c_smbus_read_byte_data(tmd277x_dev->client, 
        (TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_CHAN0HI));
    chdata[2] = i2c_smbus_read_byte_data(tmd277x_dev->client, 
        (TAOS_TRITON_CMD_REG | TAOS_TRITON_PRX_LO));
    chdata[3] = i2c_smbus_read_byte_data(tmd277x_dev->client, 
        (TAOS_TRITON_CMD_REG | TAOS_TRITON_PRX_HI));

    cleardata = chdata[1];
    cleardata <<= 8;
    cleardata |= chdata[0];
    proxdata = chdata[3];
    proxdata <<= 8;
    proxdata |= chdata[2];

    if (tmd277x_dev->prox_on || (proxdata < tmd277x_dev->pdata->prox_threshold_lo)) {
        pro_buf[0] = 0x00; /* set the prox_threshold_lo to zero, */
        pro_buf[1] = 0x00; /* next time the proxdata must be bigger than the prox_threshold_lo, 
                              this is avoid two anear appear. add by Jay.HF 2012-09-15 */
        pro_buf[2] = tmd277x_dev->pdata->prox_threshold_hi & 0x0ff;
        pro_buf[3] = tmd277x_dev->pdata->prox_threshold_hi >> 8;
        //if (tmd277x_dev->prox_on) {
        //    input_event(tmd277x_dev->input_dev_proximity, EV_MSC, MSC_SCAN, 5);
        //    input_sync(tmd277x_dev->input_dev_proximity);
        //}

        if (proxdata < tmd277x_dev->pdata->prox_threshold_hi) {
            distance = 5;
        }
        tmd277x_dev->prox_on = 0;
    }
    else if(proxdata > tmd277x_dev->pdata->prox_threshold_hi) {
        cleartemp = (tmd277x_dev->sat_als*80)/100;
        if (cleardata > cleartemp) {
            printk(KERN_ERR"%s: clear(%6d) > tmep(%6d)\n", __func__, cleardata, cleartemp);
            return -ENODATA;
        }

        pro_buf[0] = tmd277x_dev->pdata->prox_threshold_lo & 0x0ff;
        pro_buf[1] = tmd277x_dev->pdata->prox_threshold_lo >> 8;
        pro_buf[2] = 0xff; /* set the prox_threshold_hi to 0xFFFF, */
        pro_buf[3] = 0xff; /* next time the proxdata must be less than the prox_threshold_hi, 
                              this is avoid two apart appear. add by Jay.HF 2012-09-15 */
        distance = 3; /* nearby */
    }
    else {
    #if 0
        pro_buf[0] = tmd277x_dev->pdata->prox_threshold_lo & 0x0ff;
        pro_buf[1] = tmd277x_dev->pdata->prox_threshold_lo >> 8;
        pro_buf[2] = tmd277x_dev->pdata->prox_threshold_hi & 0x0ff;
        pro_buf[3] = tmd277x_dev->pdata->prox_threshold_hi >> 8;
        distance = 4;
    #else
        pro_buf[0] = 0x00; /* set the prox_threshold_lo to zero, */
        pro_buf[1] = 0x00; /* next time the proxdata must be bigger than the prox_threshold_lo, 
                              this is avoid two anear appear. add by Jay.HF 2012-09-15 */
        pro_buf[2] = tmd277x_dev->pdata->prox_threshold_hi & 0x0ff;
        pro_buf[3] = tmd277x_dev->pdata->prox_threshold_hi >> 8;
        distance = 5; /* faraway */
    #endif
    }

    if (distance != 0) {
        input_event(tmd277x_dev->input_dev_proximity, EV_MSC, MSC_SCAN, distance);
        input_sync(tmd277x_dev->input_dev_proximity);
    }

    if (tmd277x_dev->debug_enabled) {
        printk(KERN_ERR"%s: dist(%d) adc(%d) hi(%d) lo(%d) offset(%x)\n", 
            __func__, distance, proxdata, 
            tmd277x_dev->pdata->prox_threshold_hi, tmd277x_dev->pdata->prox_threshold_lo, 
            tmd277x_dev->pdata->prox_offset);
    }

    for (i=0; i<4; i++) {
        if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
                (TAOS_TRITON_CMD_REG|0x08) + i, pro_buf[i]))) < 0) {
            printk(KERN_ERR"%s: i2c fail(%d)\n", __func__, __LINE__);
            return (ret);
        }
    }

    return ret;
}

static int taos_als_init_regs(void) {
    int  ret = 0;
    u8 itime = 0, reg_val = 0, reg_cntrl = 0;

    if ((ret = (i2c_smbus_write_byte(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|TAOS_TRITON_CMD_ALS_INTCLR)))) < 0) {
        printk(KERN_ERR"%s: i2c fail(%d)\n", __func__, __LINE__);
        return (ret);
    }

    itime = (unsigned char)(((tmd277x_dev->pdata->als_time*32)/87) - 1);
    itime = (~itime);
    if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_TIME), itime))) < 0) {
        printk(KERN_ERR"%s: i2c fail(%d)\n", __func__, __LINE__);
        return (ret);
    }

    if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG|TAOS_TRITON_INTERRUPT), 
            tmd277x_dev->pdata->prox_intr_filter))) < 0) {
        printk(KERN_ERR"%s: i2c fail(%d)\n", __func__, __LINE__);
        return (ret);
    }

    if ((ret = (i2c_smbus_write_byte(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG | TAOS_TRITON_GAIN)))) < 0) {
        printk(KERN_ERR"%s: i2c fail(%d)\n", __func__, __LINE__);
        return (ret);
    }

    reg_val = i2c_smbus_read_byte(tmd277x_dev->client);
    reg_val = reg_val & 0xFC;
    reg_val = reg_val | (tmd277x_dev->pdata->gain & 0x03);
    if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG | TAOS_TRITON_GAIN), reg_val))) < 0) {
        printk(KERN_ERR"%s: i2c fail(%d)\n", __func__, __LINE__);
        return (ret);
    }

    reg_cntrl = (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON | TAOS_TRITON_CNTL_ALS_INT_ENBL);
    if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_cntrl))) < 0) {
        printk(KERN_ERR"%s: i2c fail(%d)\n", __func__, __LINE__);
        return (ret);
    }

    msleep(5);
    return ret;
}

static int taos_prox_init_regs(void) {
    int i, ret = 0;
    u8 threshold[4];
    u8 reg_cntrl = 0;

    if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG|0x01), tmd277x_dev->pdata->prox_int_time))) < 0) {
        printk(KERN_ERR"%s: i2c fail(%d)\n", __func__, __LINE__);
        return (ret);
    }

    if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG|0x02), tmd277x_dev->pdata->prox_adc_time))) < 0) {
        printk(KERN_ERR"%s: i2c fail(%d)\n", __func__, __LINE__);
        return (ret);
    }

    if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG|0x03), tmd277x_dev->pdata->prox_wait_time))) < 0) {
        printk(KERN_ERR"%s: i2c fail(%d)\n", __func__, __LINE__);
        return (ret);
    }

    threshold[0] = 0x00;
    threshold[1] = 0x00;
    threshold[2] = 0xff;
    threshold[3] = 0xff;
    for (i=0; i<4; i++) {
        if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
                (TAOS_TRITON_CMD_REG|0x08) + i, threshold[i]))) < 0) {
            printk(KERN_ERR"%s: i2c fail(%d)\n", __func__, __LINE__);
            return (ret);
        }
    }

    if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG|0x0C), tmd277x_dev->pdata->prox_intr_filter))) < 0) {
        printk(KERN_ERR"%s: i2c fail(%d)\n", __func__, __LINE__);
        return (ret);
    }

    if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG|0x0D), tmd277x_dev->pdata->prox_config))) < 0) {
        printk(KERN_ERR"%s: i2c fail(%d)\n", __func__, __LINE__);
        return (ret);
    }

    if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG|0x0E), tmd277x_dev->pdata->prox_pulse_cnt))) < 0) {
        printk(KERN_ERR"%s: i2c fail(%d)\n", __func__, __LINE__);
        return (ret);
    }

    if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG|0x0F), tmd277x_dev->pdata->prox_gain))) < 0) {
        printk(KERN_ERR"%s: i2c fail(%d)\n", __func__, __LINE__);
        return (ret);
    }

    if ((TMD2772_CHIP_ID == tmd277x_dev->chip_id) && 
        (ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG|0x1E), tmd277x_dev->pdata->prox_offset))) < 0) {
        printk(KERN_ERR"%s: i2c fail(%d)\n", __func__, __LINE__);
        return (ret);
    }

    reg_cntrl = TAOS_TRITON_CNTL_PROX_DET_ENBL | TAOS_TRITON_CNTL_PWRON | \
                TAOS_TRITON_CNTL_PROX_INT_ENBL | TAOS_TRITON_CNTL_ADC_ENBL | \
                TAOS_TRITON_CNTL_WAIT_TMR_ENBL;
    if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_cntrl))) < 0) {
        printk(KERN_ERR"%s: i2c fail(%d)\n", __func__, __LINE__);
        return (ret);
    }

    msleep(5);
    return ret;
}

static int taos_dym_set_prox_threshold(int prox_avg) {
    if( prox_avg > tmd277x_dev->pdata->prox_avg_max) {
        /* avg is too big, use the default value */
        printk(KERN_ERR "%s: Dynmic calibration fail, prox_avg = [%d] \n", __func__, prox_avg);
        return -1;
    }

    tmd277x_dev->pdata->prox_threshold_hi = prox_avg + tmd277x_dev->pdata->prox_avg_more;
    tmd277x_dev->pdata->prox_threshold_lo = prox_avg + tmd277x_dev->pdata->prox_avg_less;
    printk(KERN_ERR "%s: hi = [%d], lo = [%d], avg = [%d] \n", __func__,
        tmd277x_dev->pdata->prox_threshold_hi,
        tmd277x_dev->pdata->prox_threshold_lo, prox_avg);
    return 0;
}

static int taos_calc_crosstalk(int count) {
    int i = 0;
    int prox_sum = 0, prox_mean = 0;
    u16 temp = 0;
    u8 chdata[2];
    msleep(10);
    for (i = 0; i < count; i++) {
        chdata[0] = i2c_smbus_read_byte_data(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG | TAOS_TRITON_PRX_LO));
        chdata[1] = i2c_smbus_read_byte_data(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG | TAOS_TRITON_PRX_HI));

        temp = chdata[1];
        temp <<= 8;
        temp |= chdata[0];
        prox_sum += temp;
        msleep(10);
    }

    prox_mean = prox_sum/count;
    printk(KERN_ERR "%s: crosstalk mean=%d\n", __func__, prox_mean);
    return prox_mean;
}

static int taos_cal_init_prox(void) {
    int i, ret = 0;
    u8 threshold[4];
    u8 reg_cntrl = 0;

    if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG|0x01), tmd277x_dev->pdata->prox_int_time))) < 0) {
        printk(KERN_ERR "%s: i2c failed 1 \n", __func__);
        return (ret);
    }

    if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG|0x02), tmd277x_dev->pdata->prox_adc_time))) < 0) {
        printk(KERN_ERR "%s: i2c failed 2 \n", __func__);
        return (ret);
    }

    if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG|0x03), tmd277x_dev->pdata->prox_wait_time))) < 0) {
        printk(KERN_ERR "%s: i2c failed 3 \n", __func__);
        return (ret);
    }

    threshold[0] = 0x00;
    threshold[1] = 0x00;
    threshold[2] = 0xff;
    threshold[3] = 0xff;
    for (i=0; i<4; i++) {
        if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
                (TAOS_TRITON_CMD_REG|0x08) + i, threshold[i]))) < 0) {
            printk(KERN_ERR"%s: i2c fail(%d)\n", __func__, __LINE__);
            return (ret);
        }
    }

    if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG|0x0D), tmd277x_dev->pdata->prox_config))) < 0) {
        printk(KERN_ERR "%s: i2c failed 4 \n", __func__);
        return (ret);
    }

    if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG|0x0E), tmd277x_dev->pdata->prox_pulse_cnt))) < 0) {
        printk(KERN_ERR "%s: i2c failed 5 \n", __func__);
        return (ret);
    }

    if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG|0x0F), tmd277x_dev->pdata->prox_gain))) < 0) {
        printk(KERN_ERR "%s: i2c failed 6 \n", __func__);
        return (ret);
    }

    if ((TMD2772_CHIP_ID == tmd277x_dev->chip_id) && 
        (ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG|0x1E), tmd277x_dev->pdata->prox_offset))) < 0) {
        printk(KERN_ERR "%s: i2c failed 7 \n", __func__);
        return (ret);
    }

    reg_cntrl = (TAOS_TRITON_CNTL_PROX_DET_ENBL | TAOS_TRITON_CNTL_PWRON | TAOS_TRITON_CNTL_ADC_ENBL);
    if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_cntrl))) < 0) {
        printk(KERN_ERR "%s: i2c failed 8 \n", __func__);
        return (ret);
    }

    msleep(50);
    return ret;
}

static int taos_cal_set_prox_threshold(int prox_avg) {
    if (prox_avg > tmd277x_dev->pdata->prox_avg_max) {
        /* avg is too big, use the default value */
        tmd277x_dev->pdata->prox_threshold_hi = 0xFFFF;
        tmd277x_dev->pdata->prox_threshold_lo = 0x00;

        printk(KERN_ERR "%s: Calibration fail, prox_avg = [%d] \n", __func__, prox_avg);
        return -1;
    }

    tmd277x_dev->pdata->prox_threshold_hi = prox_avg + tmd277x_dev->pdata->prox_avg_more;
    tmd277x_dev->pdata->prox_threshold_lo = prox_avg + tmd277x_dev->pdata->prox_avg_less;
    printk(KERN_INFO "%s: thres_hi = [%d], thres_lo = [%d], prox_avg = [%d] \n", __func__,
        tmd277x_dev->pdata->prox_threshold_hi, 
        tmd277x_dev->pdata->prox_threshold_lo, prox_avg);
    return 0;
}

static int taos_cal_recover_regs(void) {

    int ret = 0;
    u8 count = 0;

    if (tmd277x_dev->prox_enabled) {
        tmd277x_dev->prox_on = 1;

        /* init regs */
        ret = taos_prox_init_regs();
        if (ret) {
            printk(KERN_ERR"%s: prox_init_regs fail \n", __func__);
            return ret;
        }

        /* set PROX threshold */
        ret = taos_prox_threshold_set();
        if (ret) {
            printk(KERN_ERR"%s: prox_threshold_set fail \n", __func__);
            return ret;
        }

        tmd277x_dev->prox_enabled = 1;
    }
    else if(tmd277x_dev->als_enabled) {
        tmd277x_dev->als_on = 1;
        ret = taos_als_init_regs();
        if (ret) {
            printk(KERN_ERR"%s: als_init_regs fail \n", __func__);
            return ret;
        }

        ret = taos_als_default_threshold_set();
        if (ret) {
            printk(KERN_ERR"%s: als_threshold_set fail(%d) \n", __func__, __LINE__);
        }
        tmd277x_dev->als_enabled = 1;
    }
    else {
        if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
                (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), 0x00))) < 0) {
            printk(KERN_ERR"%s: set ctl_reg fail \n", __func__);
            return (ret);
        }

        tmd277x_dev->prox_enabled = 0;
        cancel_work_sync(&tmd277x_dev->irq_work);
        tmd277x_dev->als_enabled = 0;
        if (tmd277x_dev->polling && (count < 50)) {
            printk(KERN_ERR"%s: %d ms to power down \n", __func__, count);
            count++;
            msleep(1);
        }
    }

    return ret;
}

static irqreturn_t taos_irq_handler(int irq, void *dev_id) {
    wake_lock(&tmd277x_dev->wake_lock); /* add by Jay.HF 2012-12-31 */
    if (tmd277x_dev->debug_enabled) {
        printk(KERN_INFO"%s\n", __func__);
    }
    tmd277x_dev->data_ready = 1;
    schedule_work(&tmd277x_dev->irq_work);
    return IRQ_HANDLED;
}

static int taos_als_get_data(void) {
    u8 reg_val;
    int ret = 0;
    int lux_val = 0;
    if ((ret = (i2c_smbus_write_byte(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL)))) < 0) {
        printk(KERN_ERR"%s: i2c fail[1] \n", __func__);
        return (ret);
    }

    reg_val = i2c_smbus_read_byte(tmd277x_dev->client);
    if ((reg_val & (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON)) != \
            (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON)) {
        printk(KERN_ERR"%s: ALS NOT enabled or power on !\n", __func__);
        return -1;
    }

    if ((ret = (i2c_smbus_write_byte(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG | TAOS_TRITON_STATUS)))) < 0) {
        printk(KERN_ERR"%s: i2c fail[2] \n", __func__);
        return (ret);
    }

    reg_val = i2c_smbus_read_byte(tmd277x_dev->client);
    if ((reg_val & TAOS_TRITON_STATUS_ADCVALID) != TAOS_TRITON_STATUS_ADCVALID) {
        printk(KERN_ERR"%s: ALS had NOT been integrated!\n", __func__);
        return -1;
    }

    if ((lux_val = taos_get_lux()) < 0) {
        printk(KERN_ERR"%s: returned error [%d] \n", __func__, lux_val);
        return -1;
    }

    input_event(tmd277x_dev->input_dev_light, EV_MSC, MSC_SCAN, lux_val);
    //input_report_abs(tmd277x_dev->input_dev_light, ABS_MISC, lux_val);
    input_sync(tmd277x_dev->input_dev_light);

    return ret;
}

static int taos_get_data(void) {
    int status;
    int ret = 0;
    if ((ret = (i2c_smbus_write_byte(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG | 0x13)))) < 0) {
        printk(KERN_ERR"%s: get_data fail \n", __func__);
        return (ret);
    }

    status = i2c_smbus_read_byte(tmd277x_dev->client);
#if 0
    if (tmd277x_dev->debug_enabled) {
        printk(KERN_ERR"%s: status[0x%x], prox->0x20, als->0x10\n", __func__, status);
    }
#endif
    if (tmd277x_dev->prox_enabled && (status & 0x20)) {
        ret = taos_prox_threshold_set();
        if (ret) {
            printk(KERN_ERR"%s: prox_threshold_set fail\n", __func__);
            return ret;
        }
    }
    else if(tmd277x_dev->als_enabled && (status & 0x10)) {
        ret = taos_als_threshold_set();
        if (ret) {
            printk(KERN_ERR"%s: als_threshold fail\n", __func__);
            return ret;
        }

        ret = taos_als_get_data();
        if (ret) {
            printk(KERN_ERR"%s: als_get_data fail\n", __func__);
            return ret;
        }
    }

    return ret;
}

static int taos_clear_interrupts(void) {
    int ret = 0;

    ret = i2c_smbus_write_byte(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07));
    if (ret < 0) {
        printk(KERN_ERR"%s: taos_clear_interrupts fail\n", __func__);
        return (ret);
    }
    tmd277x_dev->data_ready = 0;

    return ret;
}

static void taos_work_func(struct work_struct * work) {
#define TAOS_SUSPEND_TIMEOUT 5000 /* Timeout (miliseconds) */
    long remaining;

    if (tmd277x_dev->debug_enabled) {
        printk(KERN_ERR"%s: sus %d prox %d als %d\n", __func__,
               tmd277x_dev->suspend_flag, tmd277x_dev->prox_enabled,
               tmd277x_dev->als_enabled);
    }
    remaining = wait_event_interruptible_timeout(tmd277x_dev->wqh_suspend, \
                    tmd277x_dev->suspend_flag == 0, \
                    msecs_to_jiffies(TAOS_SUSPEND_TIMEOUT));
    if(tmd277x_dev->suspend_flag == 1) {
        printk(KERN_ERR"%s: timeout waiting for resume\n", __func__);
    }
    if (!tmd277x_dev->suspend_flag && \
        (tmd277x_dev->prox_enabled || tmd277x_dev->als_enabled) \
        && (!tmd277x_dev->polling)) { // && tmd277x_dev->data_ready) {
        tmd277x_dev->polling = 1;
        taos_get_data();
        tmd277x_dev->polling = 0;
        wake_lock_timeout(&tmd277x_dev->wake_lock_timeout, 2*HZ);
    }
    taos_clear_interrupts();
    wake_unlock(&tmd277x_dev->wake_lock);
}

static int taos_calc_offset(int value) {
    int ret = 0;
    int prox_avg = value;
    //unsigned char offset = 0x1f;
    unsigned char offset = 0x0f;

    while ((prox_avg >= TAOS_PROX_CROSSTALK_MAX) &&
    //while ((prox_avg >= tmd277x_dev->pdata->prox_avg_max) &&
            (offset <= tmd277x_dev->pdata->prox_offset_max)) {
        printk(KERN_ERR ":%s: offset = 0x%2x\n", __func__, offset);
        if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
                (TAOS_TRITON_CMD_REG|0x1E), offset))) < 0) {
            printk(KERN_ERR ":%s: i2c failed\n", __func__);
            return (ret);
        }

        msleep(5);
        prox_avg = taos_calc_crosstalk(10);
        if (prox_avg < TAOS_PROX_CROSSTALK_MAX) {
        //if (prox_avg < tmd277x_dev->pdata->prox_avg_max) {
            break;
        }

        //offset += 0x10;
        offset += 0x08;
    }

    if (offset > tmd277x_dev->pdata->prox_offset_max)
        offset = tmd277x_dev->pdata->prox_offset_max;
    tmd277x_dev->pdata->prox_offset = offset;

    return prox_avg;
}

static long taos_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
    int ret = 0;
    void __user *argp = (void __user *)arg;
    unsigned int phone_on = 0;
    int prox_avg = 0;
    u8 count = 0;
    u8 reg_tmp = 0;
    u8 prox_offset = 0;
    struct prox_offset taos_cal_data;
    struct prox_offset *taos_cal_ptr = &taos_cal_data;
    struct SensorCalInfo cal_info;
    switch (cmd) {
        case TAOS_IOCTL_ALS_ON:
            printk(KERN_INFO"%s: ALS_ON Entry\n", __func__);
            tmd277x_dev->als_enabled = 1;
            tmd277x_dev->als_on = 1;
        /* update tmd277x power state, modify by Jay.HF 2012-12-21 */
        #ifdef CONFIG_YULONG_MODULE_MANAGEMENT
            tmd277x_dev->pm_state = STATUS_ON;
        #endif
            if (0 == tmd277x_dev->prox_enabled) {
                printk(KERN_ERR"%s: ALS_ON taos_als_init_regs() \n", __func__);
                ret = taos_als_init_regs();
                if (ret) {
                    printk(KERN_ERR"%s: als_init_regs fail \n", __func__);
                    return ret;
                }

                ret = taos_als_default_threshold_set();
                if (ret) {
                    printk(KERN_ERR"%s: als_threshold_set fail(%d) \n", 
                        __func__, __LINE__);
                }
            }
            else if (tmd277x_dev->prox_enabled == 1) {
                reg_tmp = i2c_smbus_read_byte_data(tmd277x_dev->client, (TAOS_TRITON_CMD_REG | 0x01));
                /* resolv twice continuously move away, add by Jay.HF 2012-11-09 9:33 */
                // tmd277x_dev->prox_on = 1; 

                if (reg_tmp != tmd277x_dev->pdata->prox_int_time) {
                    printk(KERN_ERR"%s: ctl_reg1[0x%x] might be error, reset prox!\n", __func__,reg_tmp);
                    
                    /* init regs */
                    ret = taos_prox_init_regs();
                    if (ret) {
                        printk(KERN_ERR"%s: prox_init_regs fail \n", __func__);
                        return ret;
                    }
                    /* set prox threshold */
                    ret = taos_prox_threshold_set();
                    if (ret) {
                        printk(KERN_ERR"%s: prox_threshold_set fail \n", __func__);
                        return ret;
                    }
                }
            }
            taos_clear_interrupts(); /* Clear ALS or PROX Interrupt Flag, By Jay.HF 2012-10-17 */
            printk(KERN_INFO"%s: ALS_ON Exit\n", __func__);
            break;

        case TAOS_IOCTL_ALS_OFF:
            printk(KERN_INFO"%s: ALS_OFF Entry\n", __func__);
            tmd277x_dev->als_enabled = 0;
            tmd277x_dev->als_on = 0;
            /* update tmd277x power state, modify by Jay.HF 2012-12-21 */
        #ifdef CONFIG_YULONG_MODULE_MANAGEMENT
            tmd277x_dev->pm_state = STATUS_OFF;
        #endif
            if (0 == tmd277x_dev->prox_enabled) {
                if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
                        (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), 0x00))) < 0) {
                    printk(KERN_ERR"%s: clear ALS regs\n", __func__);
                    return (ret);
                }

                cancel_work_sync(&tmd277x_dev->irq_work);

                if (tmd277x_dev->polling && (count < 50)) {
                    printk(KERN_ERR"%s: %d ms\n", __func__, count);
                    count++;
                    msleep(1);
                }
                tmd277x_dev->data_ready = 0;
            }
            printk(KERN_INFO"%s: ALS_OFF Exit\n", __func__);
            break;

        case TAOS_IOCTL_PROX_ON:
            printk(KERN_INFO"%s: PROX_ON Entry\n", __func__);


            tmd277x_dev->prox_enabled = 1;
            tmd277x_dev->prox_on = 1;
        #ifdef CONFIG_YULONG_MODULE_MANAGEMENT
            /* update tmd277x power state, modify by Jay.HF 2012-12-21 */
            tmd277x_dev->pm_state = STATUS_ON;
        #endif
            if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
                    (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), 0x00))) < 0) {
                printk(KERN_ERR"%s: set ctl_reg fail \n", __func__);
                return (ret);
            }
            msleep(10);
            ret = taos_prox_init_regs();
            if (ret) {
                printk(KERN_ERR"%s: prox_init_regs failed\n", __func__);
                return ret;
            }
            ret = taos_calc_crosstalk(4);
            if (ret < 0) {
                printk(KERN_ERR"%s: taos_calc_crosstalk failed\n", __func__);
                return ret;
            }
            /* set prox threshold */
            taos_dym_set_prox_threshold(ret);
            if ((tmd277x_dev->prox_factory_threshold_hi != 0)&&
                (tmd277x_dev->pdata->prox_threshold_hi > \
                     tmd277x_dev->prox_factory_threshold_hi + 250)) {
                tmd277x_dev->pdata->prox_threshold_hi = tmd277x_dev->prox_factory_threshold_hi;
                tmd277x_dev->pdata->prox_threshold_lo = tmd277x_dev->prox_factory_threshold_lo; 
                printk(KERN_ERR "%s: use factory calibration hi = [%d], lo = [%d]\n", __func__,
                    tmd277x_dev->pdata->prox_threshold_hi, tmd277x_dev->pdata->prox_threshold_lo);
            }
        #if defined(CONFIG_ARCH_MSM8226)||defined(CONFIG_ARCH_MSM8916)
            ret = taos_prox_init_regs();
            if (ret) {
                printk(KERN_ERR"%s: prox_init_regs failed\n", __func__);
                return ret;
            }
        #endif

            ret = taos_prox_threshold_set();
            if (ret) {
                printk(KERN_ERR"%s: prox_threshold_set fail \n", __func__);
                return ret;
            }

            /* Clear ALS or PROX Interrupt Flag, By Jay.HF 2012-09-24 */
            taos_clear_interrupts(); 
            if (tmd277x_dev->prox_on_ok == 0) {
                ret = irq_set_irq_wake(tmd277x_dev->als_ps_int, 1);
                if (ret != 0) {
                    printk(KERN_ERR"%s: set irq wake source failed! \n", __func__);
                    return(ret);
                }
                tmd277x_dev->prox_on_ok = 1;
            }
            isProximityEnabled = true;
            printk(KERN_INFO"%s: PROX_ON Exit\n", __func__);
            break;

        case TAOS_IOCTL_PROX_OFF:
            printk(KERN_INFO"%s: PROX_OFF Entry\n", __func__);
            tmd277x_dev->prox_enabled = 0;
            tmd277x_dev->prox_on = 0;
        /* update tmd277x power state, modify by Jay.HF 2012-12-21 */
        #ifdef CONFIG_YULONG_MODULE_MANAGEMENT
            tmd277x_dev->pm_state = STATUS_OFF;
        #endif
            if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
                    (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), 0x00))) < 0) {
                printk(KERN_ERR"%s: set ctl_reg fail \n", __func__);
                return (ret);
            }
            if (tmd277x_dev->prox_on_ok == 1) {
                ret = irq_set_irq_wake(tmd277x_dev->als_ps_int, 0);
                if (ret != 0) {
                    printk(KERN_ERR"%s: clear irq wake source failed ! \n", __func__);
                    return(ret);
                }
                tmd277x_dev->prox_on_ok = 0;
            }
            if (tmd277x_dev->als_enabled == 1) {
                taos_als_init_regs();
                ret = taos_als_default_threshold_set();
                if(ret) {
                    printk(KERN_ERR"%s: als_threshold_set fail(%d) \n", 
                        __func__, __LINE__);
                }

                tmd277x_dev->als_enabled = 1;
            } 
            else {
                cancel_work_sync(&tmd277x_dev->irq_work);
                tmd277x_dev->als_enabled = 0;

                if (tmd277x_dev->polling && (count < 50)) {
                    printk(KERN_ERR"%s: %d ms to power down \n", __func__, count);
                    count++;
                    msleep(1);
                }

                
                tmd277x_dev->data_ready = 0;
            }
            isProximityEnabled = false;
            printk(KERN_INFO"%s: PROX_OFF Exit\n", __func__);
            break;

        case TAOS_IOCTL_PHONE_STATE:
            printk(KERN_INFO"%s: PHONE_STATE!\n", __func__);
            if (copy_from_user(&phone_on, argp, sizeof(phone_on))) {
                printk(KERN_ERR"[Jay.HF]copy failed(%s),line(%d)\n", __FUNCTION__, __LINE__);
                return -EFAULT;
            }

            if (!(phone_on & 0xFE)) {
                if (0x01 == phone_on) {
                    tmd277x_dev->phone_state = 0x01;
                    printk(KERN_ERR"%s: set tmd277x_dev->phone_state ++ \n", __func__);
                }
                else if(0x00 == phone_on) {
                    tmd277x_dev->phone_state = 0x00;
                    printk(KERN_ERR"%s: set tmd277x_dev->phone_state -- \n", __func__);
                }
            }
            break;

        case TAOS_IOCTL_PROX_CALIBRATE:
            printk(KERN_INFO"%s: PROX_CALIBRATE\n", __func__);
            memset(&cal_info, 0, sizeof(cal_info));
            if ((struct prox_offset *)arg != NULL) {
                wake_lock(&tmd277x_dev->wake_lock);
                /* save the offset register */
                prox_offset = tmd277x_dev->pdata->prox_offset;
                /* clear the offset register */
                tmd277x_dev->pdata->prox_offset = 0x00;
                /* power off sensor */
                if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, 
                    (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), 0x00))) < 0) {
                    printk(KERN_ERR"%s: set ctl_reg fail(%d)\n", __func__, __LINE__);
                    goto wake_unlock;
                }
                msleep(10);
                ret = taos_cal_init_prox();
                if (ret) {
                    printk(KERN_ERR"%s: regs for caling failed \n", __func__);
                    goto wake_unlock;
                }
                ret = taos_calc_crosstalk(20);
                if (ret < 0) {
                    printk(KERN_ERR"%s: failed in ioctl prox_calibrate\n", __func__);
                    goto wake_unlock;
                }
                prox_avg = ret;
                if (TMD2772_CHIP_ID == tmd277x_dev->chip_id) {
                    if ( prox_avg >= TAOS_PROX_CROSSTALK_MAX) {
                    //if (prox_avg >= tmd277x_dev->pdata->prox_avg_max) {
                        ret = taos_calc_offset(prox_avg);
                        if (ret < 0) {
                            goto wake_unlock;
                        }
                        prox_avg = ret;
                    }
                }
                /* if threshold if too big, then cal failed  */
                ret = taos_cal_set_prox_threshold(prox_avg);
                if (ret) {
                    cal_info.SensorCalStatus = 2;
                    printk(KERN_ERR"%s: taos_cal_set_prox_threshold failed \n", __func__);
                }
                else {
                    cal_info.SensorCalStatus = 1;
                    cal_info.value[0] = TAOS_FLASH_MAGIC;
                    if (TMD2772_CHIP_ID == tmd277x_dev->chip_id) {
                        cal_info.value[1] = tmd277x_dev->pdata->prox_offset;
                        cal_info.value[2] = tmd277x_dev->pdata->prox_threshold_hi & 0x00ff;
                        cal_info.value[3] = (tmd277x_dev->pdata->prox_threshold_hi & 0xff00) >> 8;
                    }
                    else {
                        cal_info.value[1] = tmd277x_dev->pdata->prox_threshold_hi & 0x00ff;
                        cal_info.value[2] = (tmd277x_dev->pdata->prox_threshold_hi & 0xff00) >> 8;
                    }
                    tmd277x_dev->prox_factory_threshold_hi = tmd277x_dev->pdata->prox_threshold_hi;
                    tmd277x_dev->prox_factory_threshold_lo = tmd277x_dev->pdata->prox_threshold_lo;
                }
                if (sizeof(cal_info) != sensparams_write_to_flash(SENSPARAMS_TYPE_PROX, 
                            (unsigned char *)&cal_info, sizeof(cal_info))) {
                    printk(KERN_ERR"%s: (%d) write to flash failed !\n", __func__, __LINE__);
                    ret = -1;
                    goto wake_unlock;
                }

                printk(KERN_ERR"%s: avg(%d), hig(%d), low(%d), offset(0x%2x)\n",
                        __func__, prox_avg,
                        tmd277x_dev->pdata->prox_threshold_hi, 
                        tmd277x_dev->pdata->prox_threshold_lo,
                        tmd277x_dev->pdata->prox_offset);
                ((struct prox_offset *)taos_cal_ptr)->x = \
                    (unsigned short)(tmd277x_dev->pdata->prox_threshold_hi);
                ((struct prox_offset *)taos_cal_ptr)->y = \
                    (unsigned short)(tmd277x_dev->pdata->prox_threshold_lo);
                ((struct prox_offset *)taos_cal_ptr)->z = (unsigned short)(prox_avg);
                ((struct prox_offset *)taos_cal_ptr)->key = cal_info.SensorCalStatus;
                ((struct prox_offset *)taos_cal_ptr)->offset = tmd277x_dev->pdata->prox_offset;
                if(copy_to_user((struct prox_offset *)arg, taos_cal_ptr, sizeof(taos_cal_data))) {
                    printk(KERN_ERR"%s: data trans error,use default offset ! \n", __func__);
                }
                taos_cal_recover_regs();
                wake_unlock(&tmd277x_dev->wake_lock);
            }
            else {
                printk(KERN_ERR"%s: (%d) null pointer !\n", __func__, __LINE__);
                return -1;
            }
            break;

        case TAOS_IOCTL_PROX_OFFSET:
            printk(KERN_INFO"%s: PROX_OFFSET\n", __func__);
            if (((struct prox_offset *)arg == NULL ) || \
                    copy_from_user(taos_cal_ptr, (struct prox_offset *)argp, sizeof(taos_cal_data))) {
                printk(KERN_ERR"%s: data trans error, use default offset ! \n", __func__);
                break;
            }
        #if 0
            tmd277x_dev->pdata->prox_threshold_hi = (unsigned short)(((struct prox_offset *)taos_cal_ptr)->x);
            tmd277x_dev->pdata->prox_threshold_lo = (unsigned short)(((struct prox_offset *)taos_cal_ptr)->y);
        #endif
            printk(KERN_INFO"%s: prox_hig = [%5d] \n", __func__, tmd277x_dev->pdata->prox_threshold_hi);
            printk(KERN_INFO"%s: prox_low = [%5d] \n", __func__, tmd277x_dev->pdata->prox_threshold_lo);
            break;

        default:
            printk(KERN_ERR"%s: DEFAULT!\n", __func__);
            return (-1);
    }

    return (ret);

wake_unlock:
    tmd277x_dev->pdata->prox_offset = prox_offset;
    taos_cal_recover_regs();
    wake_unlock(&tmd277x_dev->wake_lock);
    return (ret);
}

static int taos_open(struct inode *inode, struct file *file) {
    struct SensorCalInfo cal_info;
    printk(KERN_ERR"%s\n", __func__);

    if ((sizeof(cal_info) == sensparams_read_from_flash(SENSPARAMS_TYPE_PROX, \
            (unsigned char *)&cal_info, sizeof(cal_info))) && (TAOS_FLASH_MAGIC == cal_info.value[0])) {
        printk(KERN_ERR"%s: read from yulong params\n", __func__);
        if (TMD2772_CHIP_ID == tmd277x_dev->chip_id) {
            tmd277x_dev->pdata->prox_offset = cal_info.value[1];
            tmd277x_dev->pdata->prox_threshold_hi = \
                ((unsigned short)cal_info.value[3] << 8) | (unsigned short)cal_info.value[2];
            tmd277x_dev->pdata->prox_threshold_lo = tmd277x_dev->pdata->prox_threshold_hi - \
                (tmd277x_dev->pdata->prox_avg_more - tmd277x_dev->pdata->prox_avg_less);
        }
        else {
            tmd277x_dev->pdata->prox_threshold_hi = \
                ((unsigned short)cal_info.value[2] << 8) | (unsigned short)cal_info.value[1];
            tmd277x_dev->pdata->prox_threshold_lo = tmd277x_dev->pdata->prox_threshold_hi - \
                (tmd277x_dev->pdata->prox_avg_more - tmd277x_dev->pdata->prox_avg_less);
        }
        tmd277x_dev->prox_factory_threshold_hi = tmd277x_dev->pdata->prox_threshold_hi;
        tmd277x_dev->prox_factory_threshold_lo = tmd277x_dev->pdata->prox_threshold_lo;
    }
    else {
        printk(KERN_ERR"%s: use default value\n", __func__);
    #if 0
        if (TMD2772_CHIP_ID == tmd277x_dev->chip_id) {
            tmd277x_dev->pdata->prox_offset = 0x00;
        }
        tmd277x_dev->pdata->prox_threshold_hi = TAOS_PROX_THRESHOLD_Hi;
        tmd277x_dev->pdata->prox_threshold_lo = TAOS_PROX_THRESHOLD_Lo;
    #endif
    }
    printk(KERN_ERR"%s: thres_hi(%d), thres_lo(%d), offset_reg(0x%x)\n", __func__, 
            tmd277x_dev->pdata->prox_threshold_hi, 
            tmd277x_dev->pdata->prox_threshold_lo, tmd277x_dev->pdata->prox_offset);

    return (0);
}

static int taos_release(struct inode *inode, struct file *file) {
    printk(KERN_ERR"%s\n", __func__);
    return (0);
}

static struct file_operations tmd277x_fops = {
    .owner          = THIS_MODULE,
    .open           = taos_open,
    .release        = taos_release,
    .unlocked_ioctl = taos_ioctl,
};

static struct miscdevice tmd277x_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = TAOS_DEVICE_NAME,
    .fops  = &tmd277x_fops,
};

static ssize_t taos_debug_enable_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    char c;
    if (NULL == tmd277x_dev) {
        printk(KERN_ERR"%s: tmd277x_dev is NULL\n", __func__);
        return -EINVAL;
    }

    c = tmd277x_dev->debug_enabled ? '1' : '0';
    return scnprintf(buf, PAGE_SIZE, "%c\n", c);
}

static ssize_t taos_debug_enable_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    int i;
    if (NULL == tmd277x_dev) {
        printk(KERN_ERR"%s: tmd277x_dev is NULL\n", __func__);
        return -EINVAL;
    }

    if (sscanf(buf, "%u", &i) == 1 && i < 2) {
        tmd277x_dev->debug_enabled = (i == 1);
        printk(KERN_ERR"%s: %s\n", __func__, i ? "debug enabled" : "debug disabled");
        return count;
    } else {
        printk(KERN_ERR"%s: debug_enabled write error\n", __func__);
        return -EINVAL;
    }
}

static DEVICE_ATTR(debug_enable, S_IWUSR | S_IRUSR, taos_debug_enable_show,
           taos_debug_enable_store);

static struct attribute *taos_attrs[] = {
    &dev_attr_debug_enable.attr,
    NULL
};

static const struct attribute_group taos_attr_group = {
    .attrs = taos_attrs,
};

#ifdef CONFIG_PM
static int taos_suspend(struct device *dev) {
    int ret = 0;
    printk(KERN_INFO"%s\n", __func__);

    if (NULL == tmd277x_dev) {
        printk(KERN_ERR"%s: tmd277x_dev is NULL\n", __func__);
        return 0;
    }

    if (tmd277x_dev->pdata->suspend) {
        tmd277x_dev->pdata->suspend();
    }
    taos_clear_interrupts(); /* Clear ALS or PROX Interrupt Flag, By Jay.HF 2012-12-27 */
    tmd277x_dev->suspend_flag = 1;
    return ret;
}

static int taos_resume(struct device *dev) {
    int ret = 0;

    if (NULL == tmd277x_dev) {
        printk(KERN_ERR"%s: tmd277x_dev is NULL\n", __func__);
        return 0;
    }

    tmd277x_dev->suspend_flag = 0;
    printk(KERN_INFO"%s\n", __func__);

    if ((tmd277x_dev->prox_enabled || tmd277x_dev->als_enabled) && tmd277x_dev->data_ready) {
        printk(KERN_ERR"%s: prox_enabled(%d), als_enabled(%d), data_ready(%d)\n",
               __func__, tmd277x_dev->prox_enabled, 
               tmd277x_dev->als_enabled, tmd277x_dev->data_ready);
        wake_lock_timeout(&tmd277x_dev->wake_lock_timeout, 2*HZ);
        taos_get_data();
    }
    taos_clear_interrupts(); /* add by Jay.HF 2012-09-24 */
    if (tmd277x_dev->pdata->resume) {
        tmd277x_dev->pdata->resume();
    }
    wake_up_interruptible(&tmd277x_dev->wqh_suspend);
    return ret;
}
#endif

#ifdef CONFIG_YULONG_MODULE_MANAGEMENT
extern int yulong_register_module_management(struct module_management *handler);
extern int yulong_unregister_module_management(struct module_management *handler);

static int taos_pm_enable(struct taos_dev *alsprox) {
    if (NULL == alsprox) {
        printk(KERN_INFO"%s: alsprox is NULL\n", __func__);
        return -1;
    }
    printk(KERN_INFO"%s: alsprox enable\n", __func__);
    if (taos_prox_init_regs()) {
        printk(KERN_ERR"%s: prox_init_regs fail \n", __func__);
        return -1;
    }
    alsprox->prox_on = 1;
    taos_prox_threshold_set();
    taos_clear_interrupts();
    irq_set_irq_wake(alsprox->als_ps_int, 1);
    alsprox->prox_enabled = 1;
    alsprox->pm_state = STATUS_ON; /* update the pm state */
    return 0;
}

static int taos_pm_disable(struct taos_dev *alsprox) {
    int ret = 0;
    int count = 0;
    if (NULL == alsprox) {
        printk(KERN_INFO"%s: alsprox is NULL\n", __func__);
        return -1;
    }
    printk(KERN_INFO"%s: alsprox disable\n", __func__);
    if ((ret = (i2c_smbus_write_byte_data(alsprox->client, 
            (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), 0x00))) < 0) {
        printk(KERN_ERR"%s: set ctl_reg fail \n", __func__);
        return (ret);
    }

    ret = irq_set_irq_wake(alsprox->als_ps_int, 0);
    if (ret != 0) {
        printk(KERN_ERR"%s: clear irq wake source failed ! \n", __func__);
        return(ret);
    }
    cancel_work_sync(&alsprox->irq_work);
    if (alsprox->polling && (count < 50)) {
        printk(KERN_ERR"%s: %d ms to power down \n", __func__, count);
        count++;
        msleep(1);
    }
    alsprox->prox_enabled = 0;
    alsprox->als_enabled = 0;
    alsprox->data_ready = 0;
    alsprox->prox_on = 0;
    alsprox->pm_state = STATUS_OFF; /* update the pm state */
    return 0;
}

static int taos_pm_control(struct module_management *handler, unsigned int cmd) {
    int err = -1;
    struct taos_dev *alsprox = NULL;
    if (NULL == handler) {
        printk(KERN_ERR "%s: handler is NULL.\n", __func__);
        return -1;
    }
    alsprox = (struct taos_dev *)(handler->private_data);
    if (NULL == alsprox) {
        printk(KERN_ERR "%s: alsprox_data is NULL.\n", __func__);
        return -1;
    }
    switch(cmd)
    {
        case SET_MODULE_ON:
            printk(KERN_ERR "%s: cmd = SET_MODULE_ON \n", __func__);
            err = taos_pm_enable(alsprox);
            break;
            
        case SET_MODULE_OFF:
            printk(KERN_ERR "%s: cmd = SET_MODULE_OFF \n", __func__);
            err = taos_pm_disable(alsprox);
            break;
            
        default:
            printk(KERN_ERR "%s: cmd is not found \n", __func__);
            return -1;
    }
    return err;
}

static int taos_pm_get(struct module_management *handler) {
    int state;
    struct taos_dev *alsprox = NULL;
    if (NULL == handler) {
        printk(KERN_ERR "%s: handler is NULL.\n", __func__);
        return -1;
    }
    alsprox = (struct taos_dev *)(handler->private_data);
    if (NULL == alsprox) {
        printk(KERN_ERR "%s: alsprox_data is NULL.\n", __func__);
        return -1;
    }
    state = alsprox->pm_state;
    return state;
}

static struct module_management taos_pm_hdl = {
    .name = "alsprox",
    .get = taos_pm_get,
    .control = taos_pm_control,
};
#endif

//static void taos_probe_work(struct work_struct *work) {
static int taos_i2c_probe(void) {
    int ret;
    int chip_id = 0;
    printk("%s: cpu number is %d\n", __func__, smp_processor_id());
  //longjiang add start 20131017
  #ifdef CONFIG_SENSOR_POWER
    sensors_power_init(tmd277x_dev, true);
    sensors_power_on(tmd277x_dev, true);
    msleep(10);//modified by wudongxing for reduce the booting time, 2013.12.09
  #endif
  //add end

    if (tmd277x_dev->pdata->init) {
        ret = tmd277x_dev->pdata->init();
        if (ret < 0) {
            printk(KERN_ERR"%s: init failed: %d\n", __func__, ret);
            goto out_free_dev;
        }
    }
    if (tmd277x_dev->pdata->power_on) {
        ret = tmd277x_dev->pdata->power_on();
        if (ret < 0) {
            printk(KERN_ERR"%s: power on failed: %d\n", __func__, ret);
            goto out_pdata_exit;
        }
    }

    /* check taos chip id */
    chip_id = i2c_smbus_read_byte_data(tmd277x_dev->client, 
            (TAOS_TRITON_CMD_REG | TAOS_TRITON_CHIPID));
    if ((chip_id != TMD2771_CHIP_ID) && (chip_id != TMD2772_CHIP_ID)) {
        printk(KERN_ERR"%s: error chip_id = 0x%x\n", __func__, chip_id);
        ret = -EIO;
        goto out_power_off;
    }
    tmd277x_dev->chip_id = chip_id;
    if (tmd277x_dev->pdata->gpio_int >= 0) {
        tmd277x_dev->als_ps_int = gpio_to_irq(tmd277x_dev->pdata->gpio_int);
    }

    /* light Lux data */
    tmd277x_dev->input_dev_light = input_allocate_device();
    if (tmd277x_dev->input_dev_light == NULL) {
        printk(KERN_ERR"%s: allocate input device fail !\n", __func__);
        ret = -1;
        goto out_power_off;
    }
    tmd277x_dev->input_dev_light->name = TAOS_INPUT_NAME_L;
    tmd277x_dev->input_dev_light->id.bustype = BUS_I2C;
    //set_bit(EV_ABS, tmd277x_dev->input_dev_light->evbit);
    set_bit(EV_MSC, tmd277x_dev->input_dev_light->evbit);
    //input_set_capability(tmd277x_dev->input_dev_light, EV_ABS, ABS_MISC);
    input_set_capability(tmd277x_dev->input_dev_light, EV_MSC, MSC_SCAN);
    //input_set_abs_params(tmd277x_dev->input_dev_light, ABS_MISC, 0, 100000, 0, 0);
    ret = input_register_device(tmd277x_dev->input_dev_light);
    if (ret != 0) {
        printk(KERN_ERR"%s: input_register_device failed ! \n", __func__);
        goto out_free_input_l;
    }

    /* proximity data */
    tmd277x_dev->input_dev_proximity = input_allocate_device();
    if (tmd277x_dev->input_dev_proximity == NULL) {
        printk(KERN_ERR"%s: allocate input device fail !\n", __func__);
        ret = -1;
        goto out_unregister_input_l;
    }
    tmd277x_dev->input_dev_proximity->name = TAOS_INPUT_NAME_P;
    tmd277x_dev->input_dev_proximity->id.bustype = BUS_I2C;
    //set_bit(EV_ABS, tmd277x_dev->input_dev_proximity->evbit);
    set_bit(EV_MSC, tmd277x_dev->input_dev_proximity->evbit);
    //input_set_capability(tmd277x_dev->input_dev_proximity, EV_ABS, ABS_DISTANCE);
    input_set_capability(tmd277x_dev->input_dev_proximity, EV_MSC, MSC_SCAN);
    //input_set_abs_params(tmd277x_dev->input_dev_proximity, ABS_DISTANCE, 0, 1000, 0, 0);
    ret = input_register_device(tmd277x_dev->input_dev_proximity);
    if (ret != 0) {
        printk(KERN_ERR"%s: input_register_device failed ! \n", __func__);
        goto out_free_input_p;
    }

    wake_lock_init(&tmd277x_dev->wake_lock, WAKE_LOCK_SUSPEND, "taos_wake_lock");
    wake_lock_init(&tmd277x_dev->wake_lock_timeout, WAKE_LOCK_SUSPEND, "taos_wake_lock_timeout");
    INIT_WORK(&(tmd277x_dev->irq_work), taos_work_func);
    
    init_waitqueue_head(&tmd277x_dev->wqh_suspend);
    tmd277x_dev->prox_on      = 0;
    tmd277x_dev->als_on       = 0;
    tmd277x_dev->prox_enabled = 0;
    tmd277x_dev->prox_on_ok   = 0;
    tmd277x_dev->als_enabled  = 0;
    tmd277x_dev->debug_enabled= 1;
    tmd277x_dev->suspend_flag = 0;
    tmd277x_dev->polling      = 0;
    tmd277x_dev->sat_als      = (256-tmd277x_dev->pdata->prox_int_time)<<10;
    tmd277x_dev->sat_prox     = (256-tmd277x_dev->pdata->prox_adc_time)<<10;
    if ((ret = (i2c_smbus_write_byte_data(tmd277x_dev->client, \
            (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), 0x00))) < 0) {
        printk(KERN_ERR"%s: i2c_write failed in power down\n", __func__);
        goto out_unregister_input_p;
    }

    //tmd277x_dev->als_ps_int = tmd277x_dev->client->irq;
    printk(KERN_INFO"%s: request irq [%d] \n", __func__, tmd277x_dev->als_ps_int);
    ret = request_irq(tmd277x_dev->als_ps_int, taos_irq_handler, 
            IRQF_TRIGGER_FALLING, "taos_irq", NULL);
    if (ret != 0) {
        printk(KERN_ERR"%s: request_irq fail, releasing irq... \n", __func__);
        goto out_unregister_input_p;
    }

    ret = misc_register(&tmd277x_device);
    if (ret) {
        printk(KERN_ERR"%s: tmd277x_device register failed \n", __FUNCTION__);
        goto out_free_irq;
    }


    ret = sysfs_create_group(&tmd277x_dev->client->dev.kobj, &taos_attr_group);
    if (ret) {
        printk(KERN_ERR"%s: Failure %d creating sysfs group\n", __func__, ret);
        goto out_deregister_misc;
    }

#ifdef CONFIG_YULONG_MODULE_MANAGEMENT
    /* register the power manager module, add by Jay.HF 2012-12-07 */
    tmd277x_dev->pm_state = STATUS_OFF;
    taos_pm_hdl.private_data = (void *)tmd277x_dev;
    yulong_register_module_management(&taos_pm_hdl);
    /* end */
#endif

    printk("%s: Probe successful, Chip ID is 0x%02x\n", __func__, chip_id);
    return 0;

out_deregister_misc:
    misc_deregister(&tmd277x_device);
out_free_irq:
    free_irq(tmd277x_dev->als_ps_int, NULL);
out_unregister_input_p:
    input_unregister_device(tmd277x_dev->input_dev_proximity);
out_free_input_p:
    input_free_device(tmd277x_dev->input_dev_proximity);
out_unregister_input_l:
    input_unregister_device(tmd277x_dev->input_dev_light);
out_free_input_l:
    input_free_device(tmd277x_dev->input_dev_light);
out_power_off:
    if (tmd277x_dev->pdata->power_off) {
        tmd277x_dev->pdata->power_off();
    }
out_pdata_exit:
    if (tmd277x_dev->pdata->exit) {
        tmd277x_dev->pdata->exit();
    }
out_free_dev:
    kfree(tmd277x_dev);
    tmd277x_dev = NULL;
    printk(KERN_INFO"%s: probe failed(ret=%d)\n", __func__, ret);
    return ret;
}

static int /*__devinit*/ taos_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    //int cpu, ret = 0;
    int ret = 0;
    struct taos_platform_data *pdata = NULL;

#ifdef CONFIG_LPM_MODE
    extern unsigned int poweroff_charging;
    extern unsigned int recovery_mode;
    if (1 == poweroff_charging || 1 == recovery_mode) {
        printk(KERN_ERR"%s: probe exit, lpm=%d recovery=%d\n", __func__, poweroff_charging, recovery_mode);
        return -ENODEV;
    }
#endif

    printk("%s: start, cur cpu is %d\n", __func__, smp_processor_id());
    if (!client) { 
        printk(KERN_ERR"%s: i2c client null pointer! \n", __func__);
        return -EINVAL;
    }

    /* check i2c bus */
    if (!i2c_check_functionality(client->adapter, 
            I2C_FUNC_SMBUS_BYTE | \
            I2C_FUNC_SMBUS_BYTE_DATA | \
            I2C_FUNC_SMBUS_I2C_BLOCK)) {
        printk(KERN_ERR"%s: i2c check failed \n", __func__);
        return -EIO;
    }

#ifdef CONFIG_OF
    pdata = kzalloc(sizeof(struct taos_platform_data), GFP_KERNEL);
    if (pdata == NULL) {
        printk(KERN_ERR"%s: failed to allocate memory for pdata: %d\n",
               __func__, ret);
        return -ENOMEM;
    }
    ret = taos_parse_dt(&client->dev, pdata);
    if (ret) {
        dev_err(&client->dev, "faild to parse dt\n");
        goto out_free_pdata;
    }
#else
    if (!(client->dev.platform_data)) {
        printk(KERN_ERR"%s: i2c client null pointer! \n", __func__);
        ret = -EINVAL;
        goto out_free_pdata;
    }
    pdata = client->dev.platform_data;
#endif

    tmd277x_dev = kzalloc(sizeof(struct taos_dev), GFP_KERNEL);
    if (NULL == tmd277x_dev) {
        printk(KERN_ERR"%s: allocate tmd277x dev fail \n", __func__);
        ret = -ENOMEM;
        goto out_free_pdata;
    }

    memset(tmd277x_dev, 0, sizeof(struct taos_dev));
    tmd277x_dev->client = client;
    tmd277x_dev->pdata = pdata;

#if 0
    INIT_WORK(&(tmd277x_dev->probe_work), taos_probe_work);
    cpu = cpumask_next(smp_processor_id(), cpu_online_mask);
    if (cpu == NR_CPUS) { 
        cpu = cpumask_first(cpu_online_mask);
    }

    printk(KERN_ERR"%s: nr_cpus(%d) cur cpu(%d), netx cpu(%d)\n", __func__, NR_CPUS, smp_processor_id(), cpu);
    schedule_work_on(cpu, &tmd277x_dev->probe_work);
#endif

    //chenyz add for gpio cfg,20140504
    ret =  gpio_request(pdata->gpio_int, "taos_irq");
    if(ret) {
        printk("%s: gpio-%d requset failed!\n",__func__,pdata->gpio_int);
        goto out_gpio_request;
    }
    gpio_direction_input(pdata->gpio_int);

    ret = taos_i2c_probe();
    if (ret) {
        printk("%s: taos i2c probe fail \n", __func__);
        goto out_i2c_probe;
    }

    return 0;

out_i2c_probe:
    gpio_free(pdata->gpio_int);
out_gpio_request:
    kfree(tmd277x_dev);
    tmd277x_dev = NULL;
out_free_pdata:
#ifdef CONFIG_OF
    kfree(pdata);
#endif
    return ret;
}

static int /*__devexit*/ taos_remove(struct i2c_client *client) {
    int ret = 0;

    if (NULL == tmd277x_dev) {
        return 0;
    }
#ifdef CONFIG_YULONG_MODULE_MANAGEMENT
    /* unregister the power manager module, add by Jay.HF 2012-12-07 */
    yulong_unregister_module_management(&taos_pm_hdl);
    /* end */
#endif

    sysfs_remove_group(&client->dev.kobj, &taos_attr_group);
    misc_deregister(&tmd277x_device);
    disable_irq_nosync(tmd277x_dev->als_ps_int);
    free_irq(tmd277x_dev->als_ps_int, NULL);
    input_unregister_device(tmd277x_dev->input_dev_light);
    input_free_device(tmd277x_dev->input_dev_light);
    input_unregister_device(tmd277x_dev->input_dev_proximity);
    input_free_device(tmd277x_dev->input_dev_proximity);
    if (tmd277x_dev->pdata->power_off) {
        tmd277x_dev->pdata->power_off();
    }
    if(tmd277x_dev->pdata->gpio_int >= 0) {
        disable_irq_nosync(tmd277x_dev->als_ps_int);
        free_irq(tmd277x_dev->als_ps_int, NULL);
    }
    if (tmd277x_dev->pdata->exit) {
        tmd277x_dev->pdata->exit();
    }
    kfree(tmd277x_dev->pdata);
    kfree(tmd277x_dev);
    return (ret);
}

static const struct i2c_device_id taos_id[] = {
    { "tmd27723", 0 },
    { "tmd27713", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, taos_id);

#ifdef CONFIG_OF
static struct of_device_id taos_match_table[] = {
    { .compatible = "tmd27723",},
    { .compatible = "tmd27713",},
    { },
};
#else
#define taos_match_table NULL
#endif

#ifdef CONFIG_PM
static SIMPLE_DEV_PM_OPS(taos_pm_ops, taos_suspend, taos_resume);
#endif

static struct i2c_driver tmd277x_driver = {
    .driver = {
        .name  = TAOS_DRIVER_NAME,
        .owner = THIS_MODULE,
    #ifdef CONFIG_PM
        .pm    = &taos_pm_ops,
    #endif
    #ifdef CONFIG_OF
        .of_match_table = taos_match_table,
    #endif
    },
    .probe     = taos_probe,
    .remove    =taos_remove, //__devexit_p(taos_remove),
    .id_table  = taos_id,
};

static int __init taos_init(void) {
    return i2c_add_driver(&tmd277x_driver);
}

static void __exit taos_exit(void) {
    i2c_del_driver(&tmd277x_driver);
}

MODULE_AUTHOR("Jay.HF@CoolPad");
MODULE_DESCRIPTION("TAOS Light and Proximity (L&P) Sensor Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

module_init(taos_init);
module_exit(taos_exit);

