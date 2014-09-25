/****************************************************************************/
/*                                                                       	*/
/* Copyright (c) 2010-2020  YULONG Company             ??????????????     	*/
/*                 ¨®?¨¢¨²?????¨²¨ª¡§D?????¡ê¡§¨¦??¨²¡ê?¨®D?T1???  ¡ã?¨¨¡§?¨´¨®D 2010-2020  	*/
/*                                                                          */
/* PROPRIETARY RIGHTS of YULONG Company are involved in the  ????????????	*/
/* subject matter of this material.  All manufacturing, reproduction, use, 	*/
/* and sales rights pertaining to this subject matter are governed by the 	*/
/* license agreement.  The recipient of this software implicitly accepts 	*/ 
/* the terms of the license.                                             	*/
/* ¡À?¨¨¨ª?t??¦Ì¦Ì¡Á¨º¨¢?¨º?¨®?¨¢¨²1???¦Ì?¡Á¨º2¨²,¨¨?o?¨¨?¨º????¨¢o¨ª¨º1¨®?¡À?¡Á¨º¨¢?¡À?D???¦Ì?   		*/
/* ?¨¤¨®|¦Ì?¨º¨¦??¨º¨²¨¨¡§,3D¦Ì¡ê¡À¡ê?¨¹?e¨¨?o¨ª?¨®¨º¨¹?¨¤¨®|¦Ì??¡§?¨¦??¨º?.                      	*/
/*                                                                       	*/
/****************************************************************************/
 
/**************************************************************************
**  Copyright (C), 2012, Yulong Tech. Co., Ltd.
**  FileName:          tps80032.h
**  Author:            zhangzhe
**  Version :          1.00
**  Date:              2012-08-10
**  Description:       main battery driver for dual battery, based on 
**		       BQ27541 fuel gauge           
**  History:         
**  <author>	  <date>	<version> 	<desc>
**  zhangzhe	2010-08-10	  1.00 		create                                       
**************************************************************************/


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
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/err.h>

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <linux/of.h>
#endif

//------------------------------------
//zhangzhe add
#include <linux/delay.h>
#include "bq27541.h"
#include "fuel_gauge_m.h"
//#include "log_ctrl.h"
#define DRIVER_VERSION			"1.1.0"
//------------------------------------


/*************************************************
define parameter
define some static parameter,such as:
IDR
MUTEX
battery present
bus connection

*************************************************/
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);


/*************************************************
struct bq27541_device_info
define device struct, and a static pointer.

*************************************************/
struct bq27541_device_info {
	struct device			*dev;
	struct i2c_client		*client;
	struct mutex			i2c_mutex;
	struct delayed_work		hw_config;
	struct wake_lock 		fule_gauge_wake_lock;
	int				id;
	int				suspend;
};
static struct bq27541_device_info *bq27541_info_p;


/*************************************************
_bq27541_read_i2c

basic I2C operation, to read several Byte from 
BQ27541.

*************************************************/
static int _bq27541_read_i2c(u8 reg, void *buf, int count,struct i2c_client *client)
{
	struct i2c_msg msg[1];
	int err=0;

	mutex_lock(&bq27541_info_p->i2c_mutex);

	if (!client->adapter)
	{
		printk( "FUEL_M:YLLOG%s(): adapter pointer null\n",__func__);
		mutex_unlock(&bq27541_info_p->i2c_mutex);
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
		printk( "FUEL_M:YLLOG%s(): transfer fail1\n",__func__);
		mutex_unlock(&bq27541_info_p->i2c_mutex);
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
		printk("FUEL_M:YLLOG%s(): transfer fail2\n",__func__);
	}

	mutex_unlock(&bq27541_info_p->i2c_mutex);

	return err;
}

/*************************************************
_bq27541_write_i2c

basic I2C operation, to write several Byte into 
BQ27541.

history:
date		version		android
2012.05.15	1.1		zhangzhe		
*************************************************/
static int _bq27541_write_i2c(u8 reg, void *buf, int count,struct i2c_client *client)
{
	struct i2c_msg msg[1];
	char data[64];
	int err=0;

	mutex_lock(&bq27541_info_p->i2c_mutex);

	if (!client->adapter)
	{
		printk("FUEL_M:YLLOG%s(): adapter pointer null\n",__func__);
		mutex_unlock(&bq27541_info_p->i2c_mutex);
		return -ENODEV;
	}

	if (count >= 64)
	{
		printk("FUEL_M:YLLOG%s(): buffer over range\n",__func__);
		mutex_unlock(&bq27541_info_p->i2c_mutex);
		return -E2BIG;
	}

	data[0] = reg;
	memcpy(data + 1, buf, count);
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = count + 1;
	msg[0].buf = data;

	err = i2c_transfer(client->adapter, msg, 1);
	udelay(66);
	if (err < 0)
	{
		printk( "FUEL_M:YLLOG%s(): transfer fail1\n",__func__);
	}

	mutex_unlock(&bq27541_info_p->i2c_mutex);

	return err;
}


/*static int bq27541_read_std(u8 reg, int *rt_value)
{
	int ret;

//	printk(LOG_HED"%s()\n",__func__);
	
	if (!bq27541_info_p->suspend)
		ret = _bq27541_read_i2c(reg, (void*)rt_value, 2, bq27541_info_p->client);
	else
		ret = -EAGAIN;

	return ret;
}  */


/*************************************************
bq27541_m_read_sub

While it's used to send command to the main 
fuel-gauge , and receive its answer.

history:
date		version		android
2012.05.15	1.1		zhangzhe		
*************************************************/
static int bq27541_m_read_sub(unsigned short subcmd, int *rt_value)
{
	int ret;

	if (!bq27541_info_p->suspend)
		ret = _bq27541_write_i2c(BQ27541_REG_CNTL, &subcmd, 2, bq27541_info_p->client);
	else
		ret = -EAGAIN;
	if (ret >= 0) {
		ret = _bq27541_read_i2c(BQ27541_REG_CNTL, (void*)rt_value, 2, bq27541_info_p->client);
	}

	return ret;
}


static int bq27541_m_bat_bus_connection(void)
{
	int ret = 0, flags = 0;

	if (bq27541_m_read_sub(BQ27541_SUBCMD_DEVCIE_TYPE, &flags) < 0) 
	{
		ret = 0;	
		printk( "FUEL_M:YLLOGreading error , mbat might be dead!\n");
	}
	else
	{
		if(flags==0x0541)
		{

			ret = 1;
			printk( "FUEL_M:YLLOGmbat connect well !\n");	
		}
		else
		{
			ret = 0;
			printk("FUEL_M:YLLOGunknown device\n");
		}
	}	
	
	return ret;
}


/*************************************************
bq27541_m_get_proprity
 
It's used to get the proprities of main bettery.
The func has been export to kernel, so the baterry
driver could easily call it.
main proprity include:
temp
voltage(now)
current(AVG in latest 1 second)
capacity
(maybe a 'battery present')

*************************************************/
int bq27541_m_get_proprity(unsigned char command, int *data)
{
	short temp = 0;
	int ret = 0;

	if(!bq27541_info_p)	
	{
		printk( "FUEL_M:YLLOG%s(): null pointer, bq27541_info_p\n",__func__);
		return -1;
	}

	if(!data)	
	{
		printk("FUEL_M:YLLOG%s(): null pointer, data\n",__func__);
		return -2;
	}


//	printk(LOG_HED"%s(): command(%d)\n",__func__,command);

	switch(command)
	{

//bool
		case POWER_SUPPLY_PROP_PRESENT:
			*data = bq27541_m_bat_bus_connection();
			break;

//0.1K
		case POWER_SUPPLY_PROP_TEMP:

			ret = _bq27541_read_i2c(BQ27541_REG_TEMP, &temp, 2, bq27541_info_p->client);
//			temp &=0xFFFF;
			if (ret < 0) 
			{
				printk("FUEL_M:YLLOG%s: error reading tempority %d\n",__func__, ret);
				return -1;
			}
			*data = temp-2731;

			break;

//uV			
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:

			ret = _bq27541_read_i2c(BQ27541_REG_VOLT, &temp, 2, bq27541_info_p->client);
//			temp &=0xFFFF;
			if (ret < 0) 
			{
				printk("FUEL_M:YLLOG%s: error reading voltage(now) %d\n",__func__, ret);
				return -1;
			}
			if(temp>6000 || temp<0)
			{
				printk("FUEL_M:YLLOG%s: voltage data overflow, temp = [%d] \n",__func__, temp);
				return -1;
			}
			*data = temp*1000;//Android need uV
			break;

//mA
		case POWER_SUPPLY_PROP_CURRENT_AVG:

			ret = _bq27541_read_i2c(BQ27541_REG_AI, &temp, 2, bq27541_info_p->client);
//			temp &=0xFFFF;
			if (ret < 0) 
			{
				printk("FUEL_M:YLLOG%s: error reading current(avg) %d\n",__func__, ret);
				return -1;
			}
			*data = temp;
			break;

//%
		case POWER_SUPPLY_PROP_CAPACITY:

			ret = _bq27541_read_i2c(BQ27541_REG_SOC, &temp, 2, bq27541_info_p->client);
//			temp &=0xFFFF;
			if (ret < 0) 
			{
				printk("FUEL_M:YLLOG%s: error reading capacity %d\n",__func__, ret);
				return -1;
			}
			if(temp >100 || temp < 0)
			{
				printk("FUEL_M:YLLOG%s: capacity data error,temp = [%d]\n",__func__, temp);
				return -1;
			}
			*data = temp;
			break;

		case 0xAA:

			ret = _bq27541_read_i2c(BQ27541_REG_NAC, &temp, 2, bq27541_info_p->client);
			temp &=0xFFFF;
			if (ret < 0) 
			{
				printk( "FUEL_M:YLLOG%s: error reading capacity %d\n",__func__, ret);
				return -1;
			}
			*data = temp;
			break;

		case 0xAB:

			ret = _bq27541_read_i2c(BQ27541_REG_FAC, &temp, 2, bq27541_info_p->client);
//			temp &=0xFFFF;
			if (ret < 0) 
			{
				printk( "FUEL_M:YLLOG%s: error reading capacity %d\n",__func__, ret);
				return -1;
			}
			*data = temp;
			break;

		case 0xAC:

			ret = _bq27541_read_i2c(BQ27541_REG_RM, &temp, 2, bq27541_info_p->client);
//			temp &=0xFFFF;
			if (ret < 0) 
			{
				printk( "FUEL_M:YLLOG%s: error reading capacity %d\n",__func__, ret);
				return -1;
			}
			*data = temp;
			break;

		case 0xAD:

			ret = _bq27541_read_i2c(BQ27541_REG_FCC, &temp, 2, bq27541_info_p->client);
//			temp &=0xFFFF;
			if (ret < 0) 
			{
				printk( "FUEL_M:YLLOG%s: error reading capacity %d\n",__func__, ret);
				return -1;
			}
			*data = temp;
			break;

		 case 0xAF:

			ret = _bq27541_read_i2c(BQ27541_REG_CNTL, &temp, 2, bq27541_info_p->client);
//			temp &=0xFFFF;
			if (ret < 0) 
			{
				printk( "FUEL_M:YLLOG%s: error reading capacity %d\n",__func__, ret);
				return -1;
			}
			*data = temp;
			break;			

		default:
		    	printk("%s: unkonwn command [%d]\n", __func__,command);
			break;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(bq27541_m_get_proprity);


/*************************************************
bq27541_m_hw_config

It's used to config the main fuel-gauge, while just
read some config information from BQ27541 now.

change history:
date		version		android
2012.05.15	1.1		zhangzhe		
*************************************************/
static void bq27541_m_hw_config(struct work_struct *work)
{
	int ret = 0, flags = 0, type = 0, fw_ver = 0;

	printk("%s()\n",__func__);

	ret = bq27541_m_read_sub(BQ27541_SUBCMD_CTNL_STATUS, &flags);
	if (ret < 0) 
	{
		printk( "FUEL_M:YLLOG%s: error reading register %02x ret = %d\n",__func__, BQ27541_REG_CNTL, ret);
		printk("FUEL_M:YLLOG%s: Failed to config Bq27541, The battery might be depleted\n",__func__);
		return;
	}

	bq27541_m_read_sub(BQ27541_SUBCMD_DEVCIE_TYPE, 	&type);
	bq27541_m_read_sub(BQ27541_SUBCMD_FW_VER, 	&fw_ver	);

	printk( "%s: Complete bq27541 configuration 0x%02X\n\n",__func__, flags);
	printk( "%s: DEVICE_TYPE is 0x%02X, FIRMWARE_VERSION is 0x%02X\n\n",__func__,type, fw_ver);
}


/*************************************************
bq27541_m_battery_probe

Just a probe ...

change history:
date		version		android
2012.05.15	1.1		zhangzhe		
*************************************************/
static int bq27541_m_battery_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	char *name;
	struct bq27541_device_info *mp;
	//struct i2c_adapter adapter;
	int num,ret = 0;
	//short temp = 0;
	int temp_dat = 0;

	printk("%s()\n",__func__);

//1. check func
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk("FUEL_M:YLLOG%s: I2C functionality fault !\n", __func__);	
		return -ENODEV;
	}


//2. 
	//get I2C ID, (into 'num')
	ret = idr_pre_get(&battery_id, GFP_KERNEL);
	if (ret == 0)
		return -ENOMEM;
	//to protect idr
	mutex_lock(&battery_mutex);
	ret = idr_get_new(&battery_id, client, &num);
	mutex_unlock(&battery_mutex);
	if (ret < 0)
		return ret;
	// print inform
	name = kasprintf(GFP_KERNEL, "%s-%d", id->name, num);
	if (!name) 
	{
		printk("FUEL_M:YLLOG%s: failed to allocate device name\n",__func__);
		ret = -ENOMEM;
		goto batt_failed_1;
	}

//3. alloc memory, and init it

	//alloc memory
	mp = kzalloc(sizeof(struct bq27541_device_info), GFP_KERNEL);
	if (!mp) 
	{
		printk("FUEL_M:YLLOG%s: failed to allocate device info data\n",__func__);
		ret = -ENOMEM;
		goto batt_failed_2;
	}
	memset(mp,0,sizeof(struct bq27541_device_info));
	//init memory
	mp->id = num;
	i2c_set_clientdata(client, mp);
	mp->dev = &client->dev;
	mp->client = client;
	if(!mp->client) 
	{
		printk("FUEL_M:YLLOG%s: failed to setup bq27541\n",__func__);
		goto batt_failed_3;
	}
	//set bq27541_info_p point to this memory
	bq27541_info_p = mp;

	//init i2c_mutex
	mutex_init(&mp->i2c_mutex);

	//it's active now
	bq27541_info_p->suspend = 0;

//4. init work
	INIT_DELAYED_WORK(&mp->hw_config, bq27541_m_hw_config);
	schedule_delayed_work(&mp->hw_config, BQ27541_INIT_DELAY);

	printk("%s(): success\n",__func__);


	bq27541_m_get_proprity(POWER_SUPPLY_PROP_VOLTAGE_NOW,&temp_dat);

	return 0;

batt_failed_3:
	kfree(mp);
batt_failed_2:
	kfree(name);
batt_failed_1:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	printk("FUEL_M:YLLOG%s(): fail\n",__func__);

	return ret;
}

/*************************************************
bq27541_m_battery_remove

And a remove ...

by zhangzhe, 2012.05.15

change history:
date		based version	android
2012.05.15	1.1		zhangzhe		
*************************************************/
static int bq27541_m_battery_remove(struct i2c_client *client)
{
	struct bq27541_device_info *di = i2c_get_clientdata(client);

	printk("%s()\n",__func__);

	mutex_lock(&battery_mutex);
	bq27541_info_p->suspend = 1;
	mutex_unlock(&battery_mutex);

	cancel_delayed_work_sync(&di->hw_config);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	kfree(di);
	return 0;
}


/*************************************************
bq27541_m_battery_suspend

suspend, maybe no use ...

history:
date		version		android
2012.05.15	1.1		zhangzhe		
*************************************************/
static int bq27541_m_battery_suspend(struct i2c_client *client, pm_message_t mesg)
{
	printk("%s() ++\n",__func__);

	mutex_lock(&battery_mutex);
	bq27541_info_p->suspend = 1;
	mutex_unlock(&battery_mutex);

	return 0;
}


/*************************************************
bq27541_m_battery_resume

resume

history:
date		version		android
2012.05.15	1.1		zhangzhe		
*************************************************/
static int bq27541_m_battery_resume(struct i2c_client *client)
{
	printk("%s() --\n",__func__);

	mutex_lock(&battery_mutex);
	bq27541_info_p->suspend = 0;
	mutex_unlock(&battery_mutex);

	return 0;
}
/* add by peihonggang  */
#ifdef CONFIG_OF
static struct of_device_id bq27541_match_table[] = {
     	{  .compatible = "qcom,bq27541",},
     	{},
};
#else
#define bq27541_match_table NULL
#endif
/* end by phg */

static const struct i2c_device_id bq27541_id[] = {
	{ "bq27541", 1 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq27541_id);

/*************************************************
bq27541_m_battery_driver

history:
date		version		android
2012.05.15	1.1		zhangzhe		
*************************************************/
static struct i2c_driver bq27541_m_battery_driver = {
	.driver		= {
			.name = "bq27541",
			.owner = THIS_MODULE,
			.of_match_table = bq27541_match_table,
	},
	.probe		= bq27541_m_battery_probe,
	.remove		= bq27541_m_battery_remove,
	.suspend	       = bq27541_m_battery_suspend,
	.resume		= bq27541_m_battery_resume,
	.id_table	= bq27541_id,
};


/*************************************************
bq27541_m_battery_init

to register bq27541 driver for main battery

history:
date		version		android
2012.05.15	1.1		zhangzhe		
*************************************************/
static int __init bq27541_m_battery_init(void)
{
	int ret;

	printk("%s(): for tps80032\n",__func__);

	ret = i2c_add_driver(&bq27541_m_battery_driver);
	if (ret)
		printk("FUEL_M:YLLOG%s: Unable to register BQ27541 based - main-battery driver\n",__func__);

	return ret;
}

/*************************************************
bq27541_m_battery_exit

to unregister bq27541 driver for main battery

history:
date		version		android
2012.05.15	1.1		zhangzhe		
*************************************************/
static void __exit bq27541_m_battery_exit(void)
{
	i2c_del_driver(&bq27541_m_battery_driver);
}

module_init(bq27541_m_battery_init);
module_exit(bq27541_m_battery_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("WALL <zhangzhe@yulong.com>");
MODULE_DESCRIPTION("BQ27541 based - main-battery driver");
