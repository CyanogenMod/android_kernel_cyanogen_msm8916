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

#include "lc709203_fuelgauger_yl.h"
#include <linux/qpnp/qpnp-adc.h>
#include <linux/types.h>
#include "yl_pm8916_vbus.h"


#define FUELGAUGER_PREFIX      "YL:LC709203:"


#define YL_LC_DEBUG_DBG
#ifdef YL_LC_DEBUG_DBG
#define pr_lc_dbg(fmt, ...)\
	pr_debug(FUELGAUGER_PREFIX pr_fmt(fmt), ##__VA_ARGS__)
#else
#define pr_lc_dbg(fmt, ...)
#endif


#undef YL_LC_DEBUG_INFO
#define YL_LC_DEBUG_INFO
#ifdef YL_LC_DEBUG_INFO
#define pr_lc_info(fmt, ...)\
	printk(KERN_INFO FUELGAUGER_PREFIX pr_fmt(fmt), ##__VA_ARGS__)
#else
#define pr_lc_info(fmt, ...)
#endif

#undef YL_LC_DEBUG_ERR
#define YL_LC_DEBUG_ERR
#ifdef YL_LC_DEBUG_ERR
#define pr_lc_err(fmt, ...)\
	printk(KERN_ERR FUELGAUGER_PREFIX pr_fmt(fmt), ##__VA_ARGS__)
#else
#define pr_lc_err(fmt, ...)
#endif

#define DRIVER_VERSION      "1.0.0"
#define LC709203_DEV_NAME      "yl,lc709203"
#define LC709203_NAME      "lc709203"

/*MACRO for CRC */
#define	dPOLYNOMIAL8			0x8380

#define LC709203_INIT_DELAY   ((HZ)*1)

/* If the system has several batteries we need a different name for each
 * of them...
 */
/*static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);*/


struct lc709203_device_info {
	struct device			*dev;
	int				id;
	struct i2c_client		*client;
       // struct mutex                    i2c_mutex;
	struct  delayed_work		hw_config;
	struct qpnp_vadc_chip		*vadc_dev;
};

static struct lc709203_device_info *lc709203_di;


/*lc709203 rw must need crc */
static unsigned char u1_CRC_8_u1u1( unsigned char u1ArgBeforeData , unsigned char u1ArgAfterData)
{
	unsigned char	u1TmpLooper = 0;
	unsigned char	u1TmpOutData = 0;
	unsigned short	u2TmpValue = 0;

	u2TmpValue = (unsigned short)(u1ArgBeforeData ^ u1ArgAfterData);
	u2TmpValue <<= 8;

	for( u1TmpLooper = 0 ; u1TmpLooper < 8 ; u1TmpLooper++ ){
		if( u2TmpValue & 0x8000 ){
			u2TmpValue ^= dPOLYNOMIAL8;
		}
		u2TmpValue <<= 1;
	}

	u1TmpOutData = (unsigned char)(u2TmpValue >> 8);

	return( u1TmpOutData );
}

/*lc709203  CRC func end*/


/*
 * i2c specific code
 */
static int lc709203_write_i2c(u8 reg, unsigned short subcmd,
		struct lc709203_device_info *di)
{
	struct i2c_msg msg;
	unsigned char data[4];
	int ret;
	/*var for crc */
	static unsigned char	u1Debug = 0;
	static unsigned char	u1CRC8 = 0;

	if (!di->client)
		return -ENODEV;

	memset(data, 0, sizeof(data));
	data[0] = reg;
	data[1] = subcmd & 0x00FF;
	data[2] = (subcmd & 0xFF00) >> 8;
	/* crc Write Word Protocol*/
	u1Debug = u1_CRC_8_u1u1( 0x00 , 0x16 );		// Address
	u1Debug = u1_CRC_8_u1u1( u1Debug , data[0] );	// Command
	u1Debug = u1_CRC_8_u1u1( u1Debug , data[1]);	// Data
	u1CRC8 = u1_CRC_8_u1u1( u1Debug , data[2]);	// Data
	data[3]=u1CRC8;

	msg.addr = di->client->addr;
	msg.flags = 0;
	msg.len = 4;
	msg.buf = data;

	ret = i2c_transfer(di->client->adapter, &msg, 1);
	if (ret < 0) {
		pr_lc_err(" write transfer fail!\n");
		return -EIO;
	}

	return ret;
}


static int lc709203_read_i2c(u8 reg, void *buf, int count, struct lc709203_device_info *di)
{
	struct i2c_client *client = di->client;
	struct i2c_msg msgs[2];
	int err=0;

	/*var for crc */
	static unsigned char	u1Debug = 0;
	static unsigned char	u1CRC8 = 0;
	unsigned char data[3];
       
	if (!client->adapter)
	{
		pr_lc_err("adapter pointer null\n");
		return -ENODEV;
	}

	memset(data, 0, sizeof(data));
	
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &reg;

	msgs[1].addr = client->addr;
	msgs[1].flags=I2C_M_RD;
	msgs[1].len = count;
	msgs[1].buf = data;
	err = i2c_transfer(client->adapter, msgs, 2);
	udelay(66);
	if (err < 0)
	{
		pr_lc_err( "===transfer fail===\n");
		return err;
	}
	else
	{
	       
		/*crc Read Word Protocol*/
		u1Debug = u1_CRC_8_u1u1( 0x00 , 0x16 );		// Address
		u1Debug = u1_CRC_8_u1u1( u1Debug ,reg);	// Command
		u1Debug = u1_CRC_8_u1u1( u1Debug , 0x17 );	// Address
		u1Debug = u1_CRC_8_u1u1( u1Debug , data[0] );	// Data
		u1CRC8 = u1_CRC_8_u1u1( u1Debug , data[1] );	// Data
		if(u1CRC8 == data[2])
		{
		memcpy(buf,data,sizeof(data));
		pr_lc_err("read success\n");
			
		}
	}
      return err;
}

static s16 lc709203_read_reg(u8 reg, struct lc709203_device_info *di)
{
	s32 val;
	s16 val_final;
	s16 err = 0;
	u8 data[3];

	err = lc709203_read_i2c(reg,&val,3, di);
	if(err < 0)
	{
		pr_lc_err("cmd 0x%02X  read fail \n", reg);
		return err;
	}
	data[0] = (val&0x00FF);
	data[1] = (val&0xFF00)>>8;
	val_final = (data[1]<<8)|data[0];	
	pr_lc_err("reg = 0x%02X.val = 0x%X.\n", reg , val_final);

	return val_final;
}
static void lc709203_reg_write(u8 reg,int subcmd,struct lc709203_device_info *di	)
{
	int err=0;
	err = lc709203_write_i2c(reg, (unsigned short)subcmd, di);
	if(err < 0)
	{
		pr_lc_err("reg = 0x%02X write fail \n", reg );	
	}else
	pr_lc_err("reg = 0x%02X.val = 0x%X. wtite success \n", reg , subcmd);
		
}

/*
 * Return the battery temperature in tenths of degree Celsius
 * 
 */
static int lc709203_batt_temperature(struct lc709203_device_info *di)
{
	int  temp;
	
	temp = lc709203_read_reg(LC709203_REG_CELL_TEMP, di);
	temp = temp -0xAAC;
	
	pr_lc_err(": batt temp  %d\n", temp);
	return temp;
}

/*
 * Return the battery Voltage in milivolts
 * 
 */
static int lc709203_batt_voltage(struct lc709203_device_info *di)
{
	int  m_volt;
		

	m_volt = lc709203_read_reg(LC709203_REG_CELL_VOLT, di);
	pr_lc_err(": batt volt %d\n", m_volt);
	
	return m_volt;
}

/*
 * Return the battery capacity in d%
 * 
 */
static int lc709203_batt_capacity(struct lc709203_device_info *di)
{
	int capa = 0;
	capa = lc709203_read_reg(LC709203_REG_RSOC, di);	
	pr_lc_info(":batt RSOC %d\n", capa);
	return capa;
}


static int lc709203_get_batt_mvolts(void)
{
	if (!lc709203_di) {
		pr_lc_err("lc709203_di is unavaliable\n");
		return -ENODEV;
	}
	
	return lc709203_batt_voltage(lc709203_di);
}

static int lc709203_get_batt_temperature(void)
{
	if (!lc709203_di) {
		pr_lc_err("lc709203_di is unavaliable\n");
		return -ENODEV;
	}
	
	return lc709203_batt_temperature(lc709203_di);
}

static int lc709203_get_batt_capacity(void)
{
	if (!lc709203_di) {
		pr_lc_err("lc709203_di is unavaliable\n");
		return -ENODEV;
	}
	
	return lc709203_batt_capacity(lc709203_di);
}
int lc709203_get_batt_property(unsigned char command, int *data)
{
	int ret;

	if(!lc709203_di)
	{
		printk(KERN_ERR"%s:lc709203_di ==null\n", __func__);
		return -ENODEV;
	}
	
	switch(command)
	{

//0.1K
		case POWER_SUPPLY_PROP_TEMP:
			ret = lc709203_get_batt_temperature();
			if (ret < 0) 
			{
				pr_lc_err( "%s: error reading temp %d\n",__func__, ret);
				return -1;
			}
			*data = ret;
			pr_lc_info("%s: yl_temp=%d\n", __func__,ret);
			break;

//uV			
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			ret = lc709203_get_batt_mvolts();
			if (ret < 0) 
			{
				pr_lc_err( "%s: error reading voltage(now) %d\n",__func__, ret);
				return -1;
			}

			*data = ret;
			pr_lc_info("%s: yl_Voltage=%d\n", __func__,ret);
			break;	
//%
		case POWER_SUPPLY_PROP_CAPACITY:
			ret = lc709203_get_batt_capacity();
			if(ret>100 || ret< 0)
			{
				pr_lc_err( "%s: capacity data error,capacity = [%d]\n",__func__,ret);
				return -1;
			}

			*data = ret;
			pr_lc_info("%s: yl_capacity=%d\n", __func__,ret);
			break;

		default:
		    pr_lc_err("%s: unkonwn command [0x%x]\n", __func__,command);
			return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(lc709203_get_batt_property);

#ifndef CONFIG_CHARGER_HAVE_LC709203_FUEL
static int64_t read_battery_id(struct lc709203_device_info *di)//addded by Eric.zhao for battery compatable 20140715
{
	int rc;
	struct qpnp_vadc_result result;

	rc = qpnp_vadc_read(di->vadc_dev, LR_MUX2_BAT_ID, &result);
	if (rc) {
		pr_lc_err("error reading batt id channel = %d, rc = %d\n",
					LR_MUX2_BAT_ID, rc);
		return rc;
	}

	return result.physical;
}
#endif
static int lc709203_chip_config(struct lc709203_device_info *di)
{
	udelay(66); 
	lc709203_reg_write(LC709203_REG_PW_Mode,0x0001,di);//IC PowerMode
	udelay(66);
#ifdef CONFIG_CHARGER_HAVE_LC709203_FUEL
	//lc709203_reg_write(LC709203_REG_INIT_RSOC,0xAA55,di);//init SOC
	//udelay(66);
#else
	lc709203_reg_write(LC709203_REG_INIT_RSOC,0xAA55,di);//init SOC
	udelay(66);
#endif
	return 0;
}

#ifndef CONFIG_CHARGER_HAVE_LC709203_FUEL
static void lc709203_hw_config(struct work_struct *work)
{
	int ret = 0;
      int64_t battery_id_temp=0;
	s16 ic_ver = 0;
	struct lc709203_device_info *di;

	di  = container_of(work, struct lc709203_device_info, hw_config.work);
	battery_id_temp = read_battery_id(di);
	if(battery_id_temp<200000)//Guang YU Battery 20140715
		{
			pr_lc_info("===Guang Yu Battery====\n");
			lc709203_reg_write(LC709203_REG_ADJ_APPLI,0x00BF,di);
			
		}
	else //Li Shen Battery 20140715
		{
			pr_lc_info("===Li Shen Battery====\n");
			lc709203_reg_write(LC709203_REG_ADJ_APPLI,0x00BE,di);
		}
	
	ret = lc709203_chip_config(di);
	if (ret) {
		dev_err(di->dev, "Failed to config LC709203\n");
		return;
	}

	ic_ver = lc709203_read_reg(LC709203_REG_IC_VER, di);
	udelay(66);
	pr_lc_info("IC_VERSION is 0x%02X\n",ic_ver);
	pr_lc_info("Complete lc709203 configuration end\n");
	return;
}
#else
void lc709203_hw_init_by_battery(void)
{
    int64_t battery_id_temp=0;
	int ret = 0;
	s16 ic_ver = 0;
	
	battery_id_temp = read_yl_pm8916_batt_id();
	pr_lc_info("battery_id_temp = %lld\n",battery_id_temp);
	if(battery_id_temp<200000){//CPCC Battery (19000)
			pr_lc_info("===CPCC Battery====\n");
			lc709203_reg_write(LC709203_REG_ADJ_APPLI,0x00BF,lc709203_di);		
	}
	else if(battery_id_temp<600000){//CPBB Battery (574000)
			pr_lc_info("===CPBB Battery====\n");
			lc709203_reg_write(LC709203_REG_ADJ_APPLI,0x00BE,lc709203_di);
	}else if(battery_id_temp<1000000){//CPMK Battery (898000)
			pr_lc_info("===CPMK Battery====\n");
			lc709203_reg_write(LC709203_REG_ADJ_APPLI,0x00BE,lc709203_di);
	}else{		//CPLS Battery (1236000)
			pr_lc_info("===CPLS Battery====\n");
			lc709203_reg_write(LC709203_REG_ADJ_APPLI,0x00BE,lc709203_di);
		}
	
	ret = lc709203_chip_config(lc709203_di);
	if (ret) {
		dev_err(lc709203_di->dev, "Failed to config LC709203\n");
		return;
	}

	ic_ver = lc709203_read_reg(LC709203_REG_IC_VER, lc709203_di);
	udelay(66);
	pr_lc_info("IC_VERSION is 0x%02X\n",ic_ver);
	pr_lc_info("Complete lc709203 init configuration end\n");
}
#endif
#define BATTERY_CAPACITY_FIFTEEN  15
static ssize_t fuel_gauge_capacity_store(struct device_driver *drv, const char * buf, size_t count)
{
	char *after;
	unsigned long data = 0;
	data = simple_strtoul(buf, &after, 10);
	switch(data)
		{
			case BATTERY_CAPACITY_FIFTEEN:
				 lc709203_reg_write(LC709203_REG_RSOC,0x000F,lc709203_di);
		       break;
			 default:
			 break;
			 	
		}
	return count;
}

static ssize_t fuel_gauge_capacity_show(struct device_driver *drv, char *buf)
{
    long int val=0;
    u8 data[3];
    int val_final;
     s16 ret;
     ret= lc709203_read_i2c(LC709203_REG_RSOC,&val,3,lc709203_di);
     if(ret<0)
     	{
     		printk(KERN_ERR "====read cpacity error ====\n");
		return -22;
     	}
       //val_final = (val & 0x0000) >>16;
	//val_final = ((val_final & 0xFF00)>>8) | ((val_final & 0x00FF) << 8); 
	data[0] = (val&0x00FF);
	data[1] = (val&0xFF00)>>8;
	val_final = (data[1]<<8)|data[0];
	 pr_lc_err( "====data[0] = %d====\n",data[0]);
	 pr_lc_err( "====data[1] = %d====\n",data[1]);
     return snprintf(buf, PAGE_SIZE, "%d\n", val_final);
}

static DRIVER_ATTR(batt_capacity, 0666, fuel_gauge_capacity_show, fuel_gauge_capacity_store);


static int lc709203_fuelgauger_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	//char *name;
	struct lc709203_device_info *di;
	int retval = 0;

	pr_lc_info("===lc709203 probe ENTER===\n"); 
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	di = kzalloc(sizeof(struct lc709203_device_info), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}
       memset(di,0,sizeof(struct lc709203_device_info));
	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	di->client = client;

#ifdef CONFIG_LC709203_TEST_ENABLE
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
	lc709203_di = di;
#ifndef CONFIG_CHARGER_HAVE_LC709203_FUEL 
	INIT_DELAYED_WORK(&di->hw_config, lc709203_hw_config);
	schedule_delayed_work(&di->hw_config, LC709203_INIT_DELAY);
#endif	
        pr_lc_info("===lc709203 probe exit===\n");
	
	return 0;

batt_failed_1:
	kfree(di);
       return retval;
}

static int lc709203_fuelgauger_remove(struct i2c_client *client)
{
	struct lc709203_device_info *di = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&di->hw_config);
	
	/*
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);
	*/	

	kfree(di);
	return 0;
}

static const struct i2c_device_id lc709203_id[] = {
	{ LC709203_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, lc709203_id);

static struct of_device_id lc709203_match_table[] = {
	{.compatible = LC709203_DEV_NAME,},
	{},
};


static struct i2c_driver lc709203_fuelgauger_yl_driver = {
	.driver		= {
			.name = LC709203_NAME,
			.owner = THIS_MODULE,
			.of_match_table = lc709203_match_table,
	},
	.probe		= lc709203_fuelgauger_probe,
	.remove		= lc709203_fuelgauger_remove,
	.id_table	= lc709203_id,
};

static int __init lc709203_fuelgauger_init(void)
{
	int ret;

	pr_lc_dbg("ENTER===\n"); 
	
	ret = i2c_add_driver(&lc709203_fuelgauger_yl_driver);
	if (ret)
		pr_lc_err("Unable to register lc709203 driver\n");
        ret = driver_create_file(&lc709203_fuelgauger_yl_driver.driver, &driver_attr_batt_capacity);
	if (ret)
		printk(KERN_ERR"%s: driver ceate file error! \n", __func__);

	return ret;
}

#ifdef CONFIG_CHARGER_HAVE_LC709203_FUEL
	subsys_initcall(lc709203_fuelgauger_init);
#else
	module_init(lc709203_fuelgauger_init);
#endif


static void __exit lc709203_fuelgauger_exit(void)
{
	i2c_del_driver(&lc709203_fuelgauger_yl_driver);
}
module_exit(lc709203_fuelgauger_exit);



MODULE_LICENSE("GPL v1");
MODULE_AUTHOR("YULONG INC");
MODULE_DESCRIPTION("LC709203 FUELGAUGER IC DRIVER FOR YULONG Y1");

 
