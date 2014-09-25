/* Copyright(C) 2013 laser-Q
 * Copyright(C) 2013 YULONG, Inc.
 * Based on a previous work by Copyright(C) 2013 Texas Insrument,Inc.
 *
 * Copyright(C) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GUN General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
 * GUN General Public License for more details.
 */
#define pr_fmt(fmt)      "%s:" fmt, __func__

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/time.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/of.h>

#include "bq27541_fuelgauger_yl.h"

#define FUELGAUGER_PREFIX      "YL:BQ27541:"

#undef YL_BQ_DEBUG_DBG
//#define YL_BQ_DEBUG_DBG
#ifdef YL_BQ_DEBUG_DBG
#define pr_bq_dbg(fmt, ...)\
	printk(KERN_DEBUG FUELGAUGER_PREFIX pr_fmt(fmt), ##__VA_ARGS__)
#else
#define pr_bq_dbg(fmt, ...)
#endif


#undef YL_BQ_DEBUG_INFO
#define YL_BQ_DEBUG_INFO
#ifdef YL_BQ_DEBUG_INFO
#define pr_bq_info(fmt, ...)\
	printk(KERN_INFO FUELGAUGER_PREFIX pr_fmt(fmt), ##__VA_ARGS__)
#else
#define pr_bq_info(fmt, ...)
#endif

#undef YL_BQ_DEBUG_ERR
#define YL_BQ_DEBUG_ERR
#ifdef YL_BQ_DEBUG_ERR
#define pr_bq_err(fmt, ...)\
	printk(KERN_ERR FUELGAUGER_PREFIX pr_fmt(fmt), ##__VA_ARGS__)
#else
#define pr_bq_err(fmt, ...)
#endif


#define CONFIG_BQ27541_TEST_ENABLE



#define DRIVER_VERSION      "1.0.0"
#define BQ27541_DEV_NAME      "qcom,bq27541"
#define BQ27541_NAME      "bq27541"


/* Bq27541 standard data commands */
#define BQ27541_REG_CNTL		0x00
#define BQ27541_REG_AR			0x02
#define BQ27541_REG_ARTTE		0x04
#define BQ27541_REG_TEMP		0x06
#define BQ27541_REG_VOLT		0x08
#define BQ27541_REG_FLAGS		0x0A
#define BQ27541_REG_NAC			0x0C
#define BQ27541_REG_FAC			0x0e
#define BQ27541_REG_RM			0x10
#define BQ27541_REG_FCC			0x12
#define BQ27541_REG_AI			0x14
#define BQ27541_REG_TTE			0x16
#define BQ27541_REG_TTF			0x18
#define BQ27541_REG_SI			0x1a
#define BQ27541_REG_STTE		0x1c
#define BQ27541_REG_MLI			0x1e
#define BQ27541_REG_MLTTE		0x20
#define BQ27541_REG_AE			0x22
#define BQ27541_REG_AP			0x24
#define BQ27541_REG_TTECP		0x26
#define BQ27541_REG_SOH			0x28
#define BQ27541_REG_SOC			0x2c
#define BQ27541_REG_NIC			0x2e
#define BQ27541_REG_ICR			0x30
#define BQ27541_REG_LOGIDX		0x32
#define BQ27541_REG_LOGBUF		0x34

#define BQ27541_FLAG_DSC		BIT(0)
#define BQ27541_FLAG_FC			BIT(9)

#define BQ27541_CS_DLOGEN		BIT(15)
#define BQ27541_CS_SS		    BIT(13)

/* Control subcommands */
#define BQ27541_SUBCMD_CTNL_STATUS  0x0000
#define BQ27541_SUBCMD_DEVCIE_TYPE  0x0001
#define BQ27541_SUBCMD_FW_VER  0x0002
#define BQ27541_SUBCMD_HW_VER  0x0003
#define BQ27541_SUBCMD_DF_CSUM  0x0004
#define BQ27541_SUBCMD_PREV_MACW   0x0007
#define BQ27541_SUBCMD_CHEM_ID   0x0008
#define BQ27541_SUBCMD_BD_OFFSET   0x0009
#define BQ27541_SUBCMD_INT_OFFSET  0x000a
#define BQ27541_SUBCMD_CC_VER   0x000b
#define BQ27541_SUBCMD_OCV  0x000c
#define BQ27541_SUBCMD_BAT_INS   0x000d
#define BQ27541_SUBCMD_BAT_REM   0x000e
#define BQ27541_SUBCMD_SET_HIB   0x0011
#define BQ27541_SUBCMD_CLR_HIB   0x0012
#define BQ27541_SUBCMD_SET_SLP   0x0013
#define BQ27541_SUBCMD_CLR_SLP   0x0014
#define BQ27541_SUBCMD_FCT_RES   0x0015
#define BQ27541_SUBCMD_ENABLE_DLOG  0x0018
#define BQ27541_SUBCMD_DISABLE_DLOG 0x0019
#define BQ27541_SUBCMD_SEALED   0x0020
#define BQ27541_SUBCMD_ENABLE_IT    0x0021
#define BQ27541_SUBCMD_DISABLE_IT   0x0023
#define BQ27541_SUBCMD_CAL_MODE  0x0040
#define BQ27541_SUBCMD_RESET   0x0041
#define ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN   (-2731)
#define BQ27541_INIT_DELAY   ((HZ)*1)

/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);


struct bq27541_device_info {
	struct device			*dev;
	int				id;
	struct i2c_client		*client;
	/* 300ms delay is needed after bq27541 is powered up
	 * and before any successful I2C transaction
	 */
	struct  delayed_work		hw_config;
};

static struct bq27541_device_info *bq27541_di;


static s32 bq27541_readw_reg(u8 reg, struct bq27541_device_info *di)
{
	s32 val;
	struct i2c_client *client = di->client;

	val = i2c_smbus_read_word_data(client, reg);

	pr_bq_dbg("reg = 0x%02X.val = 0x%X.\n", reg , val);

	return val;
}


/*
 * i2c specific code
 */
static int bq27541_i2c_txsubcmd(u8 reg, unsigned short subcmd,
		struct bq27541_device_info *di)
{
	struct i2c_msg msg;
	unsigned char data[3];
	int ret;

	if (!di->client)
		return -ENODEV;

	memset(data, 0, sizeof(data));
	data[0] = reg;
	data[1] = subcmd & 0x00FF;
	data[2] = (subcmd & 0xFF00) >> 8;

	msg.addr = di->client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	ret = i2c_transfer(di->client->adapter, &msg, 1);
	if (ret < 0) {
		pr_bq_err("transfer fail!\n");
		return -EIO;
	}

	return 0;
}

static void bq27541_cntl_cmd(struct bq27541_device_info *di,
				int subcmd)
{
	bq27541_i2c_txsubcmd(BQ27541_REG_CNTL, subcmd, di);
}



static int bq27541_read_i2c(u8 reg, void *buf, int count, struct bq27541_device_info *di)
{
	struct i2c_client *client = di->client;
	struct i2c_msg msg[1];
	int err=0;

	if (!client->adapter)
	{
		pr_bq_err("adapter pointer null\n");
		return -ENODEV;
	}

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;
	err = i2c_transfer(client->adapter, msg, 1);
	udelay(66);
	if (err < 0)
	{
		pr_bq_err( "transfer fail1\n");
		return err;
	}

	msg[0].addr = client->addr;
	msg[0].flags = I2C_M_RD;
	msg[0].len = count;
	msg[0].buf = buf;

	err = i2c_transfer(client->adapter, msg, 1);
	udelay(66);
	if (err < 0)
	{
		pr_bq_err("transfer fail2\n");
	}

	return err;
}


/*
 * Return the battery temperature in tenths of degree Celsius
 * 
 */
static int bq27541_batt_temperature(struct bq27541_device_info *di)
{
	int temp;
	
	temp = bq27541_readw_reg(BQ27541_REG_TEMP, di);
	pr_bq_dbg(":%d\n", temp + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN);
	return temp + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
}

/*
 * Return the battery Voltage in milivolts
 * 
 */
static int bq27541_batt_voltage(struct bq27541_device_info *di)
{
	int m_volt;

	m_volt = bq27541_readw_reg(BQ27541_REG_VOLT, di);

	pr_bq_dbg(":%d\n", m_volt);
	return m_volt;
}

/*
 * Return the battery current in ma
 * negative number has been defined charging,
 */
static int bq27541_batt_current(struct bq27541_device_info *di)
{
	s16 ret;
	s16 curr;
	
	ret = bq27541_read_i2c(BQ27541_REG_AI, &curr, 2, di);
	curr = -1*curr;
	pr_bq_dbg(":%d\n", curr);
	
	//ret = bq27541_readw_reg(BQ27541_REG_AI, di);
	
	return curr;
}

/*
 * Return the battery capacity in d%
 * 
 */
static int bq27541_batt_capacity(struct bq27541_device_info *di)
{
	int capa = 0;
	capa = bq27541_readw_reg(BQ27541_REG_SOC, di);	
	pr_bq_dbg(":%d\n", capa);
	return capa;
}

/*
 * Return the battery capacity remaining in mAH
 * 
 */
static int bq27541_batt_rm_capacity(struct bq27541_device_info *di)
{
	int rem_capa = 0;
	rem_capa = bq27541_readw_reg(BQ27541_REG_RM, di);	
	pr_bq_dbg(":%d\n", rem_capa);
	return rem_capa;
}

/*
 * Return the battery capacity when battery fully charged  in mAH
 * 
 */
static int bq27541_batt_full_capacity(struct bq27541_device_info *di)
{
	int full_capa = 0;
	full_capa = bq27541_readw_reg(BQ27541_REG_FCC, di);	
	pr_bq_dbg(":%d\n", full_capa);
	return full_capa;
}



static int bq27541_get_batt_mvolts(void)
{
	if (!bq27541_di) {
		pr_bq_err("bq27541_di is unavaliable\n");
		return -ENODEV;
	}
	
	return bq27541_batt_voltage(bq27541_di);
}

static int bq27541_get_batt_temperature(void)
{
	if (!bq27541_di) {
		pr_bq_err("bq27541_di is unavaliable\n");
		return -ENODEV;
	}
	
	return bq27541_batt_temperature(bq27541_di);
}

static int bq27541_get_batt_current(void)
{
	if (!bq27541_di) {
		pr_bq_err("bq27541_di is unavaliable\n");
		return -ENODEV;
	}
	
	return bq27541_batt_current(bq27541_di);
}

static int bq27541_get_batt_capacity(void)
{
	if (!bq27541_di) {
		pr_bq_err("bq27541_di is unavaliable\n");
		return -ENODEV;
	}
	
	return bq27541_batt_capacity(bq27541_di);
}

static int bq27541_get_batt_full_capacity(void)
{
	if (!bq27541_di) {
		pr_bq_err("bq27541_di is unavaliable\n");
		return -ENODEV;
	}
	
	//return bq27541_batt_full_capacity(bq27541_di);
	return 4000;
}

static int bq27541_get_batt_rm_capacity(void)
{
	if (!bq27541_di) {
		pr_bq_err("bq27541_di is unavaliable\n");
		return -ENODEV;
	}
	
	return bq27541_batt_rm_capacity(bq27541_di);
}



static int bq27541_get_batt_status(void)
{
	return 1;
}


static int bq27541_is_batt_present(void)
{
	return 1;
}
static int bq27541_is_batt_temp_within_range(void)
{
	return 1;
}
static int bq27541_is_batt_id_valid(void)
{
	return 1;
}
static int bq27541_check_batt_present(void)
{
	int flags = 0, ret = 0;
	
	if (!bq27541_di) {
		pr_bq_err("bq27541_di is unavaliable\n");
		return -ENODEV;
	}

	bq27541_cntl_cmd(bq27541_di, BQ27541_SUBCMD_CTNL_STATUS);
	udelay(66);
	ret = bq27541_read_i2c(BQ27541_REG_CNTL, &flags, 2, bq27541_di);
	if (ret < 0) {
		pr_bq_err("error reading register %02x ret = %d\n",
			 BQ27541_REG_CNTL, ret);
		return ret;
	}

	pr_bq_dbg("BQ27541_SUBCMD_CTNL_STATUS :0x%x\n", flags);
	return 1;
}

static struct yl_battery_gauge bq27541_batt_gauge = {
	.name = BQ27541_NAME,
	.get_batt_mvolts		= bq27541_get_batt_mvolts,
	.get_batt_temperature	= bq27541_get_batt_temperature,
	.get_batt_current      = bq27541_get_batt_current,
	.get_batt_capacity       = bq27541_get_batt_capacity,
	.get_batt_status      = bq27541_get_batt_status,
	.is_batt_present		= bq27541_is_batt_present,
	.is_batt_temp_within_range	= bq27541_is_batt_temp_within_range,
	.is_batt_id_valid		= bq27541_is_batt_id_valid,
	.check_batt_present = bq27541_check_batt_present,
	.get_batt_full_capacity = bq27541_get_batt_full_capacity,
	.get_batt_rm_capacity = bq27541_get_batt_rm_capacity,
};


static int bq27541_chip_config(struct bq27541_device_info *di)
{
	int flags = 0, ret = 0;

	bq27541_cntl_cmd(di, BQ27541_SUBCMD_CTNL_STATUS);
	udelay(66);
	ret = bq27541_read_i2c(BQ27541_REG_CNTL, &flags, 2, di);
	if (ret < 0) {
		dev_err(di->dev, "error reading register %02x ret = %d\n",
			 BQ27541_REG_CNTL, ret);
		return ret;
	}
	udelay(66);

	bq27541_cntl_cmd(di, BQ27541_SUBCMD_ENABLE_IT);
	udelay(66);

	if (!(flags & BQ27541_CS_DLOGEN)) {
		bq27541_cntl_cmd(di, BQ27541_SUBCMD_ENABLE_DLOG);
		udelay(66);
	}

	return 0;
}

static void bq27541_hw_config(struct work_struct *work)
{
	int ret = 0;
	int flags = 0;
	int type = 0;
	int fw_ver = 0;
	int full_capa = 0;
	struct bq27541_device_info *di;

	di  = container_of(work, struct bq27541_device_info, hw_config.work);
	
	ret = bq27541_chip_config(di);
	if (ret) {
		dev_err(di->dev, "Failed to config Bq27541\n");
		return;
	}

	bq27541_cntl_cmd(di, BQ27541_SUBCMD_CTNL_STATUS);
	udelay(66);
	bq27541_read_i2c(BQ27541_REG_CNTL, &flags, 2, di);
	
	bq27541_cntl_cmd(di, BQ27541_SUBCMD_DEVCIE_TYPE);
	udelay(66);
	bq27541_read_i2c(BQ27541_REG_CNTL, &type, 2, di);
	
	bq27541_cntl_cmd(di, BQ27541_SUBCMD_FW_VER);
	udelay(66);
	bq27541_read_i2c(BQ27541_REG_CNTL, &fw_ver, 2, di);

	/* this indicated battery has been connected, if bq27541_batt_gauge has been registered  */
	yl_battery_gauge_register(&bq27541_batt_gauge);

	full_capa = bq27541_batt_full_capacity(di);
		
	pr_bq_info("DEVICE_TYPE is 0x%02X, FIRMWARE_VERSION is 0x%02X, battery full capacity = %dmAH\n",
			type, fw_ver, full_capa);
	pr_bq_info("Complete bq27541 configuration 0x%02X\n", flags);
}


#ifdef CONFIG_BQ27541_TEST_ENABLE
static int reg;
static int subcmd;
static ssize_t bq27541_read_stdcmd(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;
	int temp = 0;
	struct platform_device *client;
	struct bq27541_device_info *di;

	client = to_platform_device(dev);
	di = platform_get_drvdata(client);

	if (reg <= BQ27541_REG_ICR && reg > 0x00) {
		ret = bq27541_read_i2c(reg, &temp, 2, di);
		if (ret)
			ret = snprintf(buf, PAGE_SIZE, "Read Error!\n");
		else
			ret = snprintf(buf, PAGE_SIZE, "0x%02x\n", temp);
	} else
		ret = snprintf(buf, PAGE_SIZE, "Register Error!\n");

	return ret;
}

static ssize_t bq27541_write_stdcmd(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int cmd;

	sscanf(buf, "%x", &cmd);
	reg = cmd;
	return ret;
}

static ssize_t bq27541_read_subcmd(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;
	int temp = 0;
	struct platform_device *client;
	struct bq27541_device_info *di;

	client = to_platform_device(dev);
	di = platform_get_drvdata(client);

	if (subcmd == BQ27541_SUBCMD_DEVCIE_TYPE ||
		 subcmd == BQ27541_SUBCMD_FW_VER ||
		 subcmd == BQ27541_SUBCMD_HW_VER ||
		 subcmd == BQ27541_SUBCMD_CHEM_ID) {

		bq27541_cntl_cmd(di, subcmd); /* Retrieve Chip status */
		udelay(66);
		ret = bq27541_read_i2c(BQ27541_REG_CNTL, &temp, 2, di);
		if (ret)
			ret = snprintf(buf, PAGE_SIZE, "Read Error!\n");
		else
			ret = snprintf(buf, PAGE_SIZE, "0x%02x\n", temp);
	} else
		ret = snprintf(buf, PAGE_SIZE, "Register Error!\n");

	return ret;
}

static ssize_t bq27541_write_subcmd(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int cmd;

	sscanf(buf, "%x", &cmd);
	subcmd = cmd;
	return ret;
}

static DEVICE_ATTR(std_cmd, S_IRUGO|S_IWUGO, bq27541_read_stdcmd,
	bq27541_write_stdcmd);
static DEVICE_ATTR(sub_cmd, S_IRUGO|S_IWUGO, bq27541_read_subcmd,
	bq27541_write_subcmd);
static struct attribute *fs_attrs[] = {
	&dev_attr_std_cmd.attr,
	&dev_attr_sub_cmd.attr,
	NULL,
};
static struct attribute_group fs_attr_group = {
	.attrs = fs_attrs,
};

static struct platform_device this_device = {
	.name			= "bq27541-test",
	.id			= -1,
	.dev.platform_data	= NULL,
};
#endif

static int bq27541_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	char *name;
	struct bq27541_device_info *di;
	int num;
	int retval = 0;

	pr_bq_dbg("ENTER===\n"); 
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	/* Get new ID for the new battery device */
	retval = idr_pre_get(&battery_id, GFP_KERNEL);
	if (retval == 0)
		return -ENOMEM;
	mutex_lock(&battery_mutex);
	retval = idr_get_new(&battery_id, client, &num);
	mutex_unlock(&battery_mutex);
	if (retval < 0)
		return retval;

	name = kasprintf(GFP_KERNEL, "%s-%d", id->name, num);
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}
	di->id = num;

	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	di->client = client;

#ifdef CONFIG_BQ27541_TEST_ENABLE
	platform_set_drvdata(&this_device, di);
	retval = platform_device_register(&this_device);
	if (!retval) {
		retval = sysfs_create_group(&this_device.dev.kobj,
			 &fs_attr_group);
		if (retval)
			goto batt_failed_3;
	} else
		goto batt_failed_3;
#endif

	if (retval) {
		dev_err(&client->dev, "failed to setup bq27541\n");
		goto batt_failed_3;
	}

	if (retval) {
		dev_err(&client->dev, "failed to powerup bq27541\n");
		goto batt_failed_3;
	}

	bq27541_di = di;
	INIT_DELAYED_WORK(&di->hw_config, bq27541_hw_config);
	schedule_delayed_work(&di->hw_config, BQ27541_INIT_DELAY);
	
	return 0;

batt_failed_3:
	kfree(di);
batt_failed_2:
	kfree(name);
batt_failed_1:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	return retval;
}

static int bq27541_battery_remove(struct i2c_client *client)
{
	struct bq27541_device_info *di = i2c_get_clientdata(client);

	yl_battery_gauge_unregister(&bq27541_batt_gauge);
	bq27541_cntl_cmd(di, BQ27541_SUBCMD_DISABLE_DLOG);
	udelay(66);
	bq27541_cntl_cmd(di, BQ27541_SUBCMD_DISABLE_IT);
	cancel_delayed_work_sync(&di->hw_config);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	kfree(di);
	return 0;
}

static const struct i2c_device_id bq27541_id[] = {
	{ BQ27541_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, BQ27541_id);

static struct of_device_id bq27541_match_table[] = {
	{.compatible = BQ27541_DEV_NAME,},
	{},
};


static struct i2c_driver bq27541_fuelgauger_yl_driver = {
	.driver		= {
			.name = BQ27541_NAME,
			.owner = THIS_MODULE,
			.of_match_table = bq27541_match_table,
	},
	.probe		= bq27541_battery_probe,
	.remove		= bq27541_battery_remove,
	.id_table	= bq27541_id,
};

static int __init bq27541_battery_init(void)
{
	int ret;

	pr_bq_dbg("ENTER===\n"); 
	
	ret = i2c_add_driver(&bq27541_fuelgauger_yl_driver);
	if (ret)
		pr_bq_err("Unable to register BQ27541 driver\n");

	return ret;
}
module_init(bq27541_battery_init);

static void __exit bq27541_battery_exit(void)
{
	i2c_del_driver(&bq27541_fuelgauger_yl_driver);
}
module_exit(bq27541_battery_exit);



MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("YULONG INC");
MODULE_DESCRIPTION("BQ27541 FUELGAUGER IC DRIVER FOR YULONG 9976");

 
