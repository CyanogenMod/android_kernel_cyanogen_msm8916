/*
	$License:
	Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
	Copyright (C) 2013 Yamaha Corporation, All Rights Reserved.

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
	$
 */

/**
  *  @addtogroup COMPASSDL
  *
  *  @{
  *      @file   yas533.c
  *      @brief  Magnetometer setup and handling methods for Yamaha YAS533
  *              compass when used in a user-space solution (no kernel driver).
  */

#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/mpu.h>
#include <linux/platform_device.h>
#include "mpu-dev.h"
#include "log.h"
#include "mlsl.h"
#include "mldl_cfg.h"

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-compass"

#define YAS533_REGADDR_DEVICE_ID		(0x80)
#define YAS533_REGADDR_ACTUATE_INIT_COIL	(0x81)
#define YAS533_REGADDR_MEASURE_COMMAND		(0x82)
#define YAS533_REGADDR_OFFSET_X			(0x85)
#define YAS533_REGADDR_OFFSET_Y1		(0x86)
#define YAS533_REGADDR_OFFSET_Y2		(0x87)
#define YAS533_REGADDR_TEST1			(0x88)
#define YAS533_REGADDR_TEST2			(0x89)
#define YAS533_REGADDR_CAL			(0x90)
#define YAS533_REGADDR_MEASURE_DATA		(0xb0)

#define YAS533_DEVICE_ID			(0x02)
#define YAS533_VERSION_AC_COEF_X		(850)
#define YAS533_VERSION_AC_COEF_Y1		(750)
#define YAS533_VERSION_AC_COEF_Y2		(750)
#define YAS533_DATA_CENTER			(4096)
#define YAS533_DATA_UNDERFLOW			(0)
#define YAS533_DATA_OVERFLOW			(8190)
#define YAS533_MAG_STATE_NORMAL			(0)
#define YAS533_MAG_STATE_INIT_COIL		(1)
#define YAS533_MAG_STATE_MEASURE_OFFSET		(2)
#define YAS533_MAG_INITCOIL_TIMEOUT		(500)	/* msec */
#define YAS533_MAG_TEMPERATURE_LOG		(10)

#define set_vector(to, from) \
{int _l; for (_l = 0; _l < 3; _l++) to[_l] = from[_l]; }
#define is_valid_offset(a) \
	(((a)[0] <= 31) && ((a)[1] <= 31) && ((a)[2] <= 31) \
	 && (-31 <= (a)[0]) && (-31 <= (a)[1]) && (-31 <= (a)[2]))

struct yas533_cal_data {
	int8_t rxy1y2[3];
	uint8_t fxy1y2[3];
	int32_t Cx, Cy1, Cy2;
	int32_t a2, a3, a4, a5, a6, a7, a8, a9, k;
};
#if 1 < YAS533_MAG_TEMPERATURE_LOG
struct yas533_temperature_filter {
	int16_t log[YAS533_MAG_TEMPERATURE_LOG];
	int num;
	int idx;
};
#endif
struct yas533_mag_driver {
	int enabled;
	struct yas533_cal_data cal;
	int measure_state;
	int8_t hard_offset[3];
	int overflow;
	uint32_t overflow_time;
	int32_t coef[3];
	int16_t last_data[3];
#if 1 < YAS533_MAG_TEMPERATURE_LOG
	struct yas533_temperature_filter t;
#endif
	struct mutex lock;
};

struct yas533_mod_private_data {
	struct i2c_client *client;
	struct ext_slave_platform_data *pdata;
};

static const int8_t INVALID_OFFSET[] = {0x7f, 0x7f, 0x7f};
static struct yas533_mag_driver yas533_driver;
static struct i2c_client *yas533_i2c_client;

#ifdef CONFIG_OF
static int yas533_parse_dt(struct device *dev, struct ext_slave_platform_data *pdata)
{
	int i, ret;
	unsigned int value;
	int orientation[9];
	enum of_gpio_flags flags;
	struct device_node *np = dev->of_node;
	
	/* parse irq and request gpio */
	value = of_get_named_gpio_flags(np, "yas,int-gpio", 0, &flags);
	if (value < 0) {
		return -EINVAL;
	}
	gpio_request(value, "yas-irq");
	gpio_direction_input(value);

	ret = of_property_read_u32(np, "yas,address",(unsigned int *)&value);
	if (ret) {
		dev_err(dev, "Looking up %s property in node %s failed",
			"yas,address", np->full_name);
		return -ENODEV;
	}
	pdata->address = value;
	ret = of_property_read_u32(np, "yas,adapt-num", (unsigned int *)&value);
	if (ret) {
		dev_err(dev, "Looking up %s property in node %s failed",
			"yas,adapt-num", np->full_name);
		return -ENODEV;
	}
	pdata->adapt_num = value;
	ret = of_property_read_u32(np, "yas,bus", (unsigned int *)&value);
	if (ret) {
		dev_err(dev, "Looking up %s property in node %s failed",
			"yas,bus", np->full_name);
		return -ENODEV;
	}
	pdata->bus = value;
	ret = of_property_read_u32_array(np, "yas,orientation", (unsigned int *)orientation, 9);
	if (ret) {
		dev_err(dev, "Looking up %s property in node %s failed",
			"yas,orientation", np->full_name);
		return -ENODEV;
	}
	for (i  = 0; i < 9; i++) {
		pdata->orientation[i] = (orientation[i]==2) ? -1:orientation[i];
	}
	printk("address = %d\n", pdata->address);
	printk("adapt-num = %d\n", pdata->adapt_num);
	printk("bus = %d\n", pdata->bus);
	printk("orientation = \n \
			%d, %d, %d,\n \
			%d, %d, %d,\n \
			%d, %d, %d,\n", 
			pdata->orientation[0], pdata->orientation[1], pdata->orientation[2], 
			pdata->orientation[3], pdata->orientation[4], pdata->orientation[5],
			pdata->orientation[6], pdata->orientation[7], pdata->orientation[8]);
#if 0
	ret = of_property_read_string(np, "atmel,vcc_i2c_supply", &pdata->ts_vcc_i2c);
	ret = of_property_read_string(np, "atmel,vdd_ana_supply", &pdata->ts_vdd_ana);
	printk("pwr_en=%d, sleep_pwr_en=%d, vcc_i2c=%s, vdd_ana=%s\n", pdata->pwr_en,
			pdata->sleep_pwr_en, pdata->ts_vcc_i2c, pdata->ts_vdd_ana);
#endif
	return 0;	
}
#endif

static int i2c_single_write(uint8_t addr, uint8_t data)
{
	struct yas533_mod_private_data *private_data;
	if (!yas533_i2c_client)
		return -1;
	private_data = i2c_get_clientdata(yas533_i2c_client);
	if (!private_data)
		return -1;
	return inv_serial_single_write(yas533_i2c_client->adapter,
			private_data->pdata->address, addr, data);
}

static int i2c_read(uint8_t addr, uint8_t *data, int len)
{
	struct yas533_mod_private_data *private_data;
	if (!yas533_i2c_client)
		return -1;
	private_data = i2c_get_clientdata(yas533_i2c_client);
	if (!private_data)
		return -1;
	return inv_serial_read(yas533_i2c_client->adapter,
			private_data->pdata->address, addr, len, data);
}

static int get_cal_data_yas533(struct yas533_cal_data *c)
{
	uint8_t data[14];
	int i, ret;
	ret = i2c_read(YAS533_REGADDR_CAL, data, 14);
	if (ret)
		return ret;
	ret = i2c_read(YAS533_REGADDR_CAL, data, 14);
	if (ret)
		return ret;
	c->fxy1y2[0] = (uint8_t)(((data[10]&0x01)<<1) | ((data[11]>>7)&0x01));
	c->rxy1y2[0] = ((int8_t)(((data[10]>>1) & 0x3f)<<2))>>2;
	c->fxy1y2[1] = (uint8_t)(((data[11]&0x01)<<1) | ((data[12]>>7)&0x01));
	c->rxy1y2[1] = ((int8_t)(((data[11]>>1) & 0x3f)<<2))>>2;
	c->fxy1y2[2] = (uint8_t)(((data[12]&0x01)<<1) | ((data[13]>>7)&0x01));
	c->rxy1y2[2] = ((int8_t)(((data[12]>>1) & 0x3f)<<2))>>2;
	c->Cx = data[0] * 10 - 1280;
	c->Cy1 = data[1] * 10 - 1280;
	c->Cy2 = data[2] * 10 - 1280;
	c->a2 = ((data[3]>>2)&0x03f) - 32;
	c->a3 = (uint8_t)(((data[3]<<2) & 0x0c) | ((data[4]>>6) & 0x03)) - 8;
	c->a4 = (uint8_t)(data[4] & 0x3f) - 32;
	c->a5 = ((data[5]>>2) & 0x3f) + 38;
	c->a6 = (uint8_t)(((data[5]<<4) & 0x30) | ((data[6]>>4) & 0x0f)) - 32;
	c->a7 = (uint8_t)(((data[6]<<3) & 0x78) | ((data[7]>>5) & 0x07)) - 64;
	c->a8 = (uint8_t)(((data[7]<<1) & 0x3e) | ((data[8]>>7) & 0x01)) - 32;
	c->a9 = (uint8_t)(((data[8]<<1) & 0xfe) | ((data[9]>>7) & 0x01));
	c->k = (uint8_t)((data[9]>>2) & 0x1f);
	for (i = 0; i < 13; i++)
		if (data[i] != 0)
			return 0;
	if (data[13] & 0x80)
		return 0;
	return -1;
}

static int measure_normal(int *busy, int16_t *t, int16_t *xy1y2, int *ouflow)
{
	uint8_t data[8];
	int i, ret;
	ret = i2c_single_write(YAS533_REGADDR_MEASURE_COMMAND, 0x01);
	if (ret)
		return ret;
	usleep_range(2000, 3000);
	ret = i2c_read(YAS533_REGADDR_MEASURE_DATA, data, 8);
	if (ret)
		return ret;
	*busy = (data[0]>>7) & 0x01;
	*t = (int16_t)((((int32_t)data[0]<<3) & 0x3f8)|((data[1]>>5) & 0x07));
	xy1y2[0] = (int16_t)((((int32_t)data[2]<<6) & 0x1fc0)
			| ((data[3]>>2) & 0x3f));
	xy1y2[1] = (int16_t)((((int32_t)data[4]<<6) & 0x1fc0)
			| ((data[5]>>2) & 0x3f));
	xy1y2[2] = (int16_t)((((int32_t)data[6]<<6) & 0x1fc0)
			| ((data[7]>>2) & 0x3f));
	*ouflow = 0;
	for (i = 0; i < 3; i++) {
		if (xy1y2[i] == YAS533_DATA_OVERFLOW)
			*ouflow |= (1<<(i*2));
		if (xy1y2[i] == YAS533_DATA_UNDERFLOW)
			*ouflow |= (1<<(i*2+1));
	}
	return 0;
}

static int set_offset(const int8_t *offset)
{
	int ret;
	ret = i2c_single_write(YAS533_REGADDR_OFFSET_X, offset[0]);
	if (ret)
		return ret;
	ret = i2c_single_write(YAS533_REGADDR_OFFSET_Y1, offset[1]);
	if (ret)
		return ret;
	ret = i2c_single_write(YAS533_REGADDR_OFFSET_Y2, offset[2]);
	if (ret)
		return ret;
	set_vector(yas533_driver.hard_offset, offset);
	return 0;
}

static int yas533_measure_and_set_offset(void)
{
	static const int correct[5] = {16, 8, 4, 2, 1};
	int8_t hard_offset[3] = {0, 0, 0};
	int16_t t, xy1y2[3];
	int32_t flag[3];
	int i, j, busy, ouflow, ret;
	for (i = 0; i < 5; i++) {
		ret = set_offset(hard_offset);
		if (ret)
			return ret;
		ret = measure_normal(&busy, &t, xy1y2, &ouflow);
		if (ret)
			return ret;
		if (busy)
			return -1;
		for (j = 0; j < 3; j++) {
			if (YAS533_DATA_CENTER == xy1y2[j])
				flag[j] = 0;
			if (YAS533_DATA_CENTER < xy1y2[j])
				flag[j] = 1;
			if (xy1y2[j] < YAS533_DATA_CENTER)
				flag[j] = -1;
		}
		for (j = 0; j < 3; j++)
			if (flag[j])
				hard_offset[j] = (int8_t)(hard_offset[j]
						+ flag[j] * correct[i]);
	}
	return set_offset(hard_offset);
}

static int yas533_get_offset(int8_t *hard_offset)
{
	set_vector(hard_offset, yas533_driver.hard_offset);
	return 0;
}

static int yas533_set_offset(const int8_t *hard_offset)
{
	int ret;
	if (!yas533_driver.enabled) {
		set_vector(yas533_driver.hard_offset, hard_offset);
		return 0;
	}
	ret = i2c_single_write(YAS533_REGADDR_ACTUATE_INIT_COIL, 0);
	if (ret)
		return ret;
	if (is_valid_offset(hard_offset)) {
		ret = set_offset(hard_offset);
		if (ret)
			return ret;
		yas533_driver.measure_state = YAS533_MAG_STATE_NORMAL;
	} else {
		set_vector(yas533_driver.hard_offset, INVALID_OFFSET);
		yas533_driver.measure_state = YAS533_MAG_STATE_MEASURE_OFFSET;
	}
	return 0;
}

static void yas533_init(void)
{
	int i;
	set_vector(yas533_driver.hard_offset, INVALID_OFFSET);
	yas533_driver.overflow = 0;
	yas533_driver.overflow_time = jiffies_to_msecs(jiffies);
	yas533_driver.enabled = 0;
	yas533_driver.coef[0] = YAS533_VERSION_AC_COEF_X;
	yas533_driver.coef[1] = YAS533_VERSION_AC_COEF_Y1;
	yas533_driver.coef[2] = YAS533_VERSION_AC_COEF_Y2;
	yas533_driver.measure_state = YAS533_MAG_STATE_MEASURE_OFFSET;
#if 1 < YAS533_MAG_TEMPERATURE_LOG
	yas533_driver.t.num = yas533_driver.t.idx = 0;
#endif
	for (i = 0; i < 3; i++)
		yas533_driver.last_data[i] = 0;
	mutex_init(&yas533_driver.lock);
}

static int yas533_disable(void)
{
	yas533_driver.enabled = 0;
	return 0;
}

static int yas533_enable(void)
{
	int ret;
	uint8_t data;
	ret = i2c_single_write(YAS533_REGADDR_TEST1, 0x00);
	if (ret)
		return ret;
	ret = i2c_single_write(YAS533_REGADDR_TEST2, 0x00);
	if (ret)
		return ret;
	ret = i2c_read(YAS533_REGADDR_DEVICE_ID, &data, 1);
	if (ret)
		return ret;
	if (data != YAS533_DEVICE_ID)
		return -1;
	ret = get_cal_data_yas533(&yas533_driver.cal);
	if (ret)
		return ret;
	yas533_driver.enabled = 1;
	ret = yas533_set_offset(yas533_driver.hard_offset);
	if (ret) {
		yas533_driver.enabled = 0;
		return ret;
	}
	yas533_driver.overflow = 0;
	yas533_driver.overflow_time = jiffies_to_msecs(jiffies);
#if 1 < YAS533_MAG_TEMPERATURE_LOG
	yas533_driver.t.num = yas533_driver.t.idx = 0;
#endif
	return 0;
}

static int yas533_measure(int16_t *rawfixed)
{
	static const int16_t cval[] = {3721, 3971, 4221, 4471};
	struct yas533_cal_data *c = &yas533_driver.cal;
	int32_t tmp;
	int32_t sx, sy1, sy2, sy, sz, xy1y2_linear[3];
	int32_t xyz[3];
	int16_t xy1y2[3], temperature;
	int ret, i, ouflow, busy;
#if 1 < YAS533_MAG_TEMPERATURE_LOG
	int32_t sum = 0;
#endif
	uint32_t t;
	if (!yas533_driver.enabled) {
		for (i = 0; i < 3; i++)
			rawfixed[i] = yas533_driver.last_data[i];
		return 0;
	}
	switch (yas533_driver.measure_state) {
	case YAS533_MAG_STATE_INIT_COIL:
		t = jiffies_to_msecs(jiffies);
		if (t - yas533_driver.overflow_time
				< YAS533_MAG_INITCOIL_TIMEOUT)
			break;
		yas533_driver.overflow_time = t;
		ret = i2c_single_write(YAS533_REGADDR_ACTUATE_INIT_COIL, 0);
		if (ret)
			return ret;
		if (!yas533_driver.overflow
				&& is_valid_offset(yas533_driver.hard_offset)) {
			yas533_driver.measure_state = YAS533_MAG_STATE_NORMAL;
			break;
		}
		yas533_driver.measure_state = YAS533_MAG_STATE_MEASURE_OFFSET;
		/* FALLTHRU */
	case YAS533_MAG_STATE_MEASURE_OFFSET:
		ret = yas533_measure_and_set_offset();
		if (ret)
			return ret;
		yas533_driver.measure_state = YAS533_MAG_STATE_NORMAL;
		break;
	}

	ret = measure_normal(&busy, &temperature, xy1y2, &ouflow);
	if (ret)
		return ret;
	for (i = 0; i < 3; i++)
		xy1y2_linear[i] = xy1y2[i]
			- cval[yas533_driver.cal.fxy1y2[i]]
			+ (yas533_driver.hard_offset[i]
					- yas533_driver.cal.rxy1y2[i])
			* yas533_driver.coef[i];
#if 1 < YAS533_MAG_TEMPERATURE_LOG
	yas533_driver.t.log[yas533_driver.t.idx++] = temperature;
	if (YAS533_MAG_TEMPERATURE_LOG <= yas533_driver.t.idx)
		yas533_driver.t.idx = 0;
	yas533_driver.t.num++;
	if (YAS533_MAG_TEMPERATURE_LOG <= yas533_driver.t.num)
		yas533_driver.t.num = YAS533_MAG_TEMPERATURE_LOG;
	for (i = 0; i < yas533_driver.t.num; i++)
		sum += yas533_driver.t.log[i];
	tmp = sum * 10 / yas533_driver.t.num;
#else
	tmp = temperature * 10;
#endif
	sx  = xy1y2_linear[0];
	sy1 = xy1y2_linear[1];
	sy2 = xy1y2_linear[2];
	sx  -= (c->Cx  * tmp) / 1000;
	sy1 -= (c->Cy1 * tmp) / 1000;
	sy2 -= (c->Cy2 * tmp) / 1000;
	sy = sy1 - sy2;
	sz = -sy1 - sy2;
	xyz[0] = c->k * ((100   * sx + c->a2 * sy + c->a3 * sz) / 10);
	xyz[1] = c->k * ((c->a4 * sx + c->a5 * sy + c->a6 * sz) / 10);
	xyz[2] = c->k * ((c->a7 * sx + c->a8 * sy + c->a9 * sz) / 10);
	if (busy)
		return -1;
	if (0 < ouflow) {
		if (!yas533_driver.overflow)
			yas533_driver.overflow_time = jiffies_to_msecs(jiffies);
		yas533_driver.overflow = 1;
		yas533_driver.measure_state = YAS533_MAG_STATE_INIT_COIL;
	} else
		yas533_driver.overflow = 0;
	for (i = 0; i < 3; i++) {
		if (ouflow & (1<<(i*2)))
			rawfixed[i] = 0x7fff; /* overflow */
		else if (ouflow & (1<<(i*2+1)))
			rawfixed[i] = 0x8000; /* underflow */
		else
			rawfixed[i] = xyz[i] / 100;
		yas533_driver.last_data[i] = rawfixed[i];
	}
	return 0;
}

static int yas533_suspend(void *mlsl_handle,
		struct ext_slave_descr *slave,
		struct ext_slave_platform_data *pdata)
{
	int ret;
	mutex_lock(&yas533_driver.lock);
	ret = yas533_disable();
	mutex_unlock(&yas533_driver.lock);
	return ret;
}

static int yas533_resume(void *mlsl_handle,
		struct ext_slave_descr *slave,
		struct ext_slave_platform_data *pdata)
{
	int ret;
	mutex_lock(&yas533_driver.lock);
	ret = yas533_enable();
	mutex_unlock(&yas533_driver.lock);
	return ret;
}

static int yas533_read(void *mlsl_handle,
		struct ext_slave_descr *slave,
		struct ext_slave_platform_data *pdata,
		unsigned char *data)
{
	short rawfixed[3];
	int ret;
	mutex_lock(&yas533_driver.lock);
	ret = yas533_measure(rawfixed);
	mutex_unlock(&yas533_driver.lock);
	if (ret)
		return ret;
	data[0] = rawfixed[0] >> 8;
	data[1] = rawfixed[0] & 0xFF;
	data[2] = rawfixed[1] >> 8;
	data[3] = rawfixed[1] & 0xFF;
	data[4] = rawfixed[2] >> 8;
	data[5] = rawfixed[2] & 0xFF;
	return INV_SUCCESS;
}

static struct ext_slave_descr yas533_descr = {
	.init		= NULL,
	.exit		= NULL,
	.suspend	= yas533_suspend,
	.resume		= yas533_resume,
	.read		= yas533_read,
	.config		= NULL,
	.get_config	= NULL,
	.name		= "yas533",
	.type		= EXT_SLAVE_TYPE_COMPASS,
	.id		= COMPASS_ID_YAS533,
	.read_reg	= 0x06,
	.read_len	= 6,
	.endian		= EXT_SLAVE_BIG_ENDIAN,
	.range		= {4915, 2000},
	.trigger	= NULL,
};

static struct ext_slave_descr *yas533_get_slave_descr(void)
{
	return &yas533_descr;
}

static ssize_t yas533_enable_show(struct device_driver *ddri, char *buf)
{
	int enabled;
	mutex_lock(&yas533_driver.lock);
	enabled = yas533_driver.enabled;
	mutex_unlock(&yas533_driver.lock);
	return sprintf(buf, "%d\n", enabled);
}

static ssize_t yas533_enable_store(struct device_driver *ddrv, const char *buf,
		size_t count)
{
	long value;
	if (kstrtol(buf, 10, &value) < 0)
		return -EINVAL;
	mutex_lock(&yas533_driver.lock);
	if (value)
		yas533_enable();
	else
		yas533_disable();
	mutex_unlock(&yas533_driver.lock);
	return count;
}

static ssize_t yas533_data_show(struct device_driver *ddri, char *buf)
{
	int16_t rawfixed[3];
	int ret;
	mutex_lock(&yas533_driver.lock);
	ret = yas533_measure(rawfixed);
	mutex_unlock(&yas533_driver.lock);
	if (ret)
		return -EFAULT;
	return sprintf(buf, "%d %d %d\n", rawfixed[0], rawfixed[1],
			rawfixed[2]);
}

static ssize_t yas533_offset_show(struct device_driver *ddri, char *buf)
{
	int8_t hard_offset[3];
	int ret;
	mutex_lock(&yas533_driver.lock);
	ret = yas533_get_offset(hard_offset);
	mutex_unlock(&yas533_driver.lock);
	if (ret)
		return -EFAULT;
	return sprintf(buf, "%d %d %d\n", hard_offset[0],
			hard_offset[1], hard_offset[2]);
}

static ssize_t yas533_offset_store(struct device_driver *ddrv, const char *buf,
		size_t count)
{
	int32_t tmp[3];
	int8_t hard_offset[3];
	int i, ret;
	sscanf(buf, "%d %d %d", &tmp[0], &tmp[1], &tmp[2]);
	for (i = 0; i < 3; i++)
		hard_offset[i] = (int8_t) tmp[i];
	mutex_lock(&yas533_driver.lock);
	ret = yas533_set_offset(hard_offset);
	mutex_unlock(&yas533_driver.lock);
	if (ret)
		return -EFAULT;
	return count;
}

static DRIVER_ATTR(enable, S_IRUSR|S_IRGRP|S_IWUSR|S_IWGRP,
		yas533_enable_show, yas533_enable_store);
static DRIVER_ATTR(data, S_IRUSR|S_IRGRP, yas533_data_show, NULL);
static DRIVER_ATTR(offset, S_IRUGO|S_IWUGO,
		yas533_offset_show, yas533_offset_store);
static struct driver_attribute *yas533_attr_list[] = {
	&driver_attr_enable,
	&driver_attr_data,
	&driver_attr_offset,
};

static int yas533_create_attr(struct device_driver *driver)
{
	int i, ret = 0;
	int num = ARRAY_SIZE(yas533_attr_list);
	for(i = 0; i < num; i++) {
		ret = driver_create_file(driver, yas533_attr_list[i]);
		if(ret) {
			printk(KERN_ERR "%s: driver_create_file (%s) = %d\n",
					__func__,
					yas533_attr_list[i]->attr.name, ret);
			break;
		}
	}
	return ret;
}

static void yas533_delete_attr(struct device_driver *driver)
{
	int num = ARRAY_SIZE(yas533_attr_list), i;
	for(i = 0; i < num; i++)
		driver_remove_file(driver, yas533_attr_list[i]);
}

static unsigned short normal_i2c[] = { I2C_CLIENT_END };

static int yas533_probe(struct platform_device *pdev);
static int yas533_remove(struct platform_device *pdev);

#ifdef CONFIG_OF
static struct of_device_id yas533_platform_match_table[] = {
	{ .compatible = "yas533",},
	{ },
};
#endif

static struct platform_driver yas533_platform_driver = {
	.probe      = yas533_probe,
	.remove     = yas533_remove,
	.driver     = {
		.name  = "msensor",
		.owner = THIS_MODULE,
	#ifdef CONFIG_OF
      	.of_match_table = yas533_platform_match_table,
	#endif
	}
};

static int yas533_mod_probe(struct i2c_client *client,
		const struct i2c_device_id *devid)
{
	struct ext_slave_platform_data *pdata;
	struct yas533_mod_private_data *private_data;
	int result = 0;
	uint8_t data;
	dev_info(&client->adapter->dev, "%s: %s\n", __func__, devid->name);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		result = -ENODEV;
		goto out_no_free;
	}

#ifdef CONFIG_OF
	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(&client->adapter->dev, "faild to alloc pdata memory\n");
		result = -ENOMEM;
		goto out_no_free;
	}

	result = yas533_parse_dt(&client->dev, pdata);
	if (result) {
		dev_err(&client->adapter->dev, "faild to parse dt\n");
		goto out_free_pdata;
	}
#else
	pdata = client->dev.platform_data;
	if (!pdata) {
		dev_err(&client->adapter->dev,
				"Missing platform data for slave %s\n", devid->name);
		result = -EFAULT;
		goto out_free_pdata;
	}
#endif
	private_data = kzalloc(sizeof(*private_data), GFP_KERNEL);
	if (!private_data) {
		result = -ENOMEM;
		goto out_free_pdata;
	}

	result = yas533_create_attr(&yas533_platform_driver.driver);
	if (result) {
		printk(KERN_ERR "%s: create_attr failed\n", __func__);
		goto out_free_memory;
	}
	private_data->client = client;
	private_data->pdata = pdata;
	i2c_set_clientdata(client, private_data);
	yas533_i2c_client = client;
	//i2c_smbus_read_byte_data(client,);
	result=i2c_read(YAS533_REGADDR_DEVICE_ID, &data, 1);

	if (data != YAS533_DEVICE_ID)
		goto out_remove_group;
	result = inv_mpu_register_slave(THIS_MODULE, client, pdata,
			yas533_get_slave_descr);
	if (result) {
		dev_err(&client->adapter->dev,
				"Slave registration failed: %s, %d\n",
				devid->name, result);
		goto out_remove_group;
	}
	yas533_init();
	return 0;

out_remove_group:
	yas533_delete_attr(&yas533_platform_driver.driver);
out_free_memory:
	kfree(private_data);
out_free_pdata:
#ifdef CONFIG_OF
	kfree(pdata);
#endif
out_no_free:
	dev_err(&client->adapter->dev, "%s failed %d\n", __func__, result);
	i2c_set_clientdata(client, NULL);
	return result;
}

static int yas533_mod_remove(struct i2c_client *client)
{
	struct yas533_mod_private_data *private_data =
		i2c_get_clientdata(client);
	if (private_data) {
		yas533_delete_attr(&yas533_platform_driver.driver);
		inv_mpu_unregister_slave(client, private_data->pdata,
				yas533_get_slave_descr);
		kfree(private_data);
	}
	return 0;
}

static const struct i2c_device_id yas533_mod_id[] = {
	{ "yas533", COMPASS_ID_YAS533 },
	{}
};

MODULE_DEVICE_TABLE(i2c, yas533_mod_id);

#ifdef CONFIG_OF
static struct of_device_id yas533_match_table[] = {
	{ .compatible = "yas533",},
	{ },
};
#endif

static struct i2c_driver yas533_mod_driver = {
	.class = I2C_CLASS_HWMON,
	.probe = yas533_mod_probe,
	.remove = yas533_mod_remove,
	.id_table = yas533_mod_id,
	.driver = {
		.owner = THIS_MODULE,
		.name = "yas533_mod",
	#ifdef CONFIG_OF
      	.of_match_table = yas533_match_table,
	#endif
	},
	.address_list = normal_i2c,
};

static int yas533_probe(struct platform_device *pdev)
{
	int res;
#ifdef CONFIG_LPM_MODE
    extern unsigned int poweroff_charging;
    extern unsigned int recovery_mode;
	if (1 == poweroff_charging || 1 == recovery_mode) {
        printk(KERN_ERR"%s: probe exit, lpm=%d recovery=%d\n", __func__, poweroff_charging, recovery_mode);
		return -ENODEV;
	}
#endif
	pr_info("%s: Probe name %s\n", __func__, pdev->name);
	res = i2c_add_driver(&yas533_mod_driver);
	if (res)
		pr_err("%s failed [%d]\n", __func__, res);
	return res;
}

static int yas533_remove(struct platform_device *pdev)
{
	pr_info("%s: Remove name %s\n", __func__, pdev->name);
	i2c_del_driver(&yas533_mod_driver);
	return 0;
}

static int __init yas533_mod_init(void)
{
	pr_info("%s: module_init name %s\n", __func__, "yas533_mod");
	return platform_driver_register(&yas533_platform_driver);
}

static void __exit yas533_mod_exit(void)
{
	pr_info("%s: module_exit name %s\n", __func__, "yas533_mod");
	platform_driver_unregister(&yas533_platform_driver);
}

module_init(yas533_mod_init);
module_exit(yas533_mod_exit);

MODULE_AUTHOR("Invensense Corporation");
MODULE_DESCRIPTION("Driver to integrate YAS533 sensor with the MPU");
MODULE_LICENSE("GPL");
MODULE_ALIAS("yas533_mod");

/**
 *  @}
 */
