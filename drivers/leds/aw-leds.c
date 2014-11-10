#include	<linux/err.h>
#include	<linux/errno.h>
#include	<linux/delay.h>
#include	<linux/fs.h>
#include	<linux/i2c.h>
#include	<linux/input.h>
#include	<linux/uaccess.h>
#include	<linux/workqueue.h>
#include	<linux/irq.h>
#include	<linux/gpio.h>
#include	<linux/interrupt.h>
#include	<linux/slab.h>

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/regmap.h>
#include <linux/power_supply.h>
#include<linux/mutex.h>
//#include <linux/mfd/88pm80x.h>
#include <linux/leds.h> //added by zht on 2013.10.14
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES			5
#define AW_LEDS_DEV_NAME        "aw_leds"
#define DEBUG_TRICOLOR_LED 	1

//#define LT_TEST 1

#define AW_RSTR               	0x00
#define AW_GCR				0x01
#define AW_STATUS			0x02
#define AW_LEDE				0x30
#define AW_LED0_CTRL		0x31
#define AW_LED1_CTRL		0x32
#define AW_LED2_CTRL		0x33
#define AW_PWM0			0x34
#define AW_PWM1			0x35
#define AW_PWM2			0x36
#define AW_LED0_T0			0x37
#define AW_LED0_T1			0x38
#define AW_LED0_T2			0x39
#define AW_LED1_T0			0x3A
#define AW_LED1_T1			0x3B
#define AW_LED1_T2			0x3C
#define AW_LED2_T0			0x3D
#define AW_LED2_T1			0x3E
#define AW_LED2_T2			0x3F
#define AW_ASR				0x77H

enum aw_led_color {
	LED_COLOR_RED,
	LED_COLOR_GREEN,
	LED_COLOR_BLUE,
	LED_COLOR_DEFAULT,
};



#define Imax          0x73
#define Rise_time   0x00
#define Hold_time   0x01 //holdtime 0.26s
#define Fall_time     0x00
#define Off_time      0x04//2.08s
#define Delay_time   0x20  // 0.26S
#define Period_Num  0x00

struct aw_leds_data {
	struct i2c_client *client;
	struct mutex lock;
	unsigned int ldo_ctl_flag;
	unsigned int led_flag[6];
	int led_data[3];
       struct led_classdev leds[3];//added by zht for aw-leds
       struct aw_leds_platform_data *pdata;
	struct regulator *vdd_ana;
};

struct aw_leds_data *aw_data=NULL;

//add by taokai 2014-7-22
struct aw_leds_platform_data {
       unsigned short led_red_sw;
	unsigned short aw_red_on;
	unsigned short aw_red_off ;
	unsigned short AW_RED_CTRL;
	unsigned short AW_RED_PWM ;
	unsigned short AW_RED_T0;
	unsigned short AW_RED_T1;
	unsigned short AW_RED_T2;

	unsigned short aw_green_on;
	unsigned short aw_green_off;
	unsigned short AW_GREEN_CTRL;
	unsigned short AW_GREEN_PWM;
	unsigned short AW_GREEN_T0 ;
	unsigned short AW_GREEN_T1;
	unsigned short AW_GREEN_T2  ;

	unsigned short aw_blue_on;
	unsigned short aw_blue_off ;
	unsigned short AW_BLUE_CTRL;
	unsigned short AW_BLUE_PWM;
	unsigned short AW_BLUE_T0;
	unsigned short AW_BLUE_T1;
	unsigned short AW_BLUE_T2;
};
//add end

/****************************************************I2C**********************************************************/
#if 0
static int aw_leds_i2c_read_reg(u8 regnum)
{
	int val = -1;
	int tries = 0;


	/*
	 * i2c_smbus_read_byte_data - SMBus "read byte" protocol
	 * @client: Handle to slave device
	 * @command: Byte interpreted by slave
	 *
	 * This executes the SMBus "read byte" protocol, returning negative errno
	 * else a data byte received from the device.
	 */


	do{
		val = i2c_smbus_read_byte_data(aw_data->client, regnum);

		if(val < 0){
			printk(KERN_ERR"[aw_leds]:error:i2c_read failed in %s \n", __func__);
			msleep_interruptible(I2C_RETRY_DELAY);//continue;
		}
	}while((val < 0) && (++tries < I2C_RETRIES));

	return val;


}
#endif


static int aw_leds_i2c_write_reg(u8 regnum, u8 value)
{
        int32_t ret = -1;
	uint8_t tries = 0;

	/*
	 * i2c_smbus_write_byte_data - SMBus "write byte" protocol
	 * @client: Handle to slave device
	 * @command: Byte interpreted by slave
	 * @value: Byte being written
	 *
	 * This executes the SMBus "write byte" protocol, returning negative errno
	 * else zero on success.
	 */

	do{
		ret = i2c_smbus_write_byte_data(aw_data->client, regnum, value);

		if(ret){
			printk(KERN_ERR"[aw_leds]:error:i2c_write failed in %s \n", __func__);
			msleep_interruptible(I2C_RETRY_DELAY);//continue;
		}
	}while((ret != 0) && (++tries < I2C_RETRIES));

    return ret;

}
/*
static int aw_led_power_on(struct aw_leds_data *aw_data)
{
	int ret;
	ret = pm800_extern_read(PM80X_POWER_PAGE,0X52);
	ret |= (1<<1);
	ret = pm800_extern_write(PM80X_POWER_PAGE,0x52,ret);
	return 0;
}*/

/*
static int aw_led_power_off(struct aw_leds_data *aw_data)
{
	int ret;

	ret = pm800_extern_read(PM80X_POWER_PAGE,0X52);
	ret &= (~(1<<1));
	ret = pm800_extern_write(PM80X_POWER_PAGE,0x52,ret);
	return 0;
}*/

static void aw_leds_dev_init(struct aw_leds_data *data)
{
	//int ret;
	//int ledgain;
#if 0
        printk(KERN_INFO"aw_leds:%s ",__func__);
	aw_leds_i2c_write_reg(AW_GCR,0x01);

	aw_leds_i2c_write_reg(AW_LEDE,0x00);    //green 0x01  blue 0x02 red 0x04

	aw_leds_i2c_write_reg(AW_LED0_CTRL,0x01);
//	aw_leds_i2c_write_reg(AW_LED1_CTRL,Imax);
	aw_leds_i2c_write_reg(AW_LED1_CTRL,0x01);
	aw_leds_i2c_write_reg(AW_LED2_CTRL,0x02);
	aw_leds_i2c_write_reg(AW_PWM0,0xff);
	aw_leds_i2c_write_reg(AW_PWM1,0xff);
	aw_leds_i2c_write_reg(AW_PWM2,0xff);

	aw_leds_i2c_write_reg(AW_LED0_T0,Rise_time<<4 | Hold_time);
	aw_leds_i2c_write_reg(AW_LED0_T1,Fall_time<<4 | Off_time);
	aw_leds_i2c_write_reg(AW_LED0_T2,Delay_time<<4| Period_Num);

	aw_leds_i2c_write_reg(AW_LED1_T0,Rise_time<<4 | Hold_time);
	aw_leds_i2c_write_reg(AW_LED1_T1,Fall_time<<4 | Off_time);
	aw_leds_i2c_write_reg(AW_LED1_T2,Delay_time<<4| Period_Num);

	aw_leds_i2c_write_reg(AW_LED2_T0,Rise_time<<4 | Hold_time);
	aw_leds_i2c_write_reg(AW_LED2_T1,Fall_time<<4 | Off_time);
	aw_leds_i2c_write_reg(AW_LED2_T2,Delay_time<<4| Period_Num);
#endif
	msleep(20);
}

#if 0
static ssize_t blue_led_set_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 val;
	val = aw_leds_i2c_read_reg(AW_LED2_CTRL);
	return sprintf(buf, "%x\n", val);


}


void blue_led(int on){

	if(on){
		//aw_led_power_on(aw_data);
		aw_leds_i2c_write_reg(AW_GCR,0x01);
		aw_leds_i2c_write_reg(AW_LEDE,0x04);
		aw_leds_i2c_write_reg(AW_LED2_CTRL,0x02);
		aw_leds_i2c_write_reg(AW_PWM2,0xff);
		aw_leds_i2c_write_reg(AW_LED2_T1,Fall_time<<4 | Off_time);
	}else{
	//	aw_led_power_off(aw_data);

		aw_leds_i2c_write_reg(AW_LEDE,0x00);
		aw_leds_i2c_write_reg(AW_PWM2,0x00);
	}

}

static ssize_t blue_led_set_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	unsigned long enable = simple_strtoul(buf,NULL, 10);
	mutex_lock(&aw_data->lock);   //test

	aw_data->ldo_ctl_flag = enable;
	if(enable)
	{
//		aw_leds_i2c_write_reg(AW_LEDE,0x00);
//		aw_leds_i2c_write_reg(AW_PWM0,0x00);
		blue_led(1);
	}else{
//		aw_leds_i2c_write_reg(AW_LEDE,0x01);
//		aw_leds_i2c_write_reg(AW_LED0_CTRL,0x03);
//		aw_leds_i2c_write_reg(AW_PWM0,0xff);
		blue_led(0);
	}
	mutex_unlock(&aw_data->lock);
	return count;
}

static ssize_t blue_blink_set_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 val;
	val = aw_leds_i2c_read_reg(AW_LED2_CTRL);
	return sprintf(buf, "%x\n", val);
}

static ssize_t blue_blink_set_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long enable = simple_strtoul(buf,NULL, 10);
	mutex_lock(&aw_data->lock);
	aw_data->ldo_ctl_flag = enable;
	if(enable)
	{
	//	aw_led_power_on(aw_data);
		aw_leds_i2c_write_reg(AW_GCR,0x01);
		aw_leds_i2c_write_reg(AW_LEDE,0x04);
		aw_leds_i2c_write_reg(AW_LED2_CTRL,0x12);
		aw_leds_i2c_write_reg(AW_PWM2,0xff);
		aw_leds_i2c_write_reg(AW_LED2_T1,Fall_time<<4 | Off_time);

	}else{
	//	aw_led_power_off(aw_data);
		aw_leds_i2c_write_reg(AW_LEDE,0x00);
		aw_leds_i2c_write_reg(AW_PWM2,0x00);


	}
	mutex_unlock(&aw_data->lock);
	return count;
}

static ssize_t red_led_set_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 val;
	val = aw_leds_i2c_read_reg(AW_LED0_CTRL);
	return sprintf(buf, "%x\n", val);
}

static ssize_t red_led_set_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long enable = simple_strtoul(buf,NULL, 10);
	mutex_lock(&aw_data->lock);
	aw_data->ldo_ctl_flag = enable;
	if(enable)
	{
		//aw_led_power_on(aw_data);
		aw_leds_i2c_write_reg(AW_GCR,0x01);
		aw_leds_i2c_write_reg(AW_LEDE,0x01);
		aw_leds_i2c_write_reg(AW_LED0_CTRL,0x01);
		aw_leds_i2c_write_reg(AW_PWM0,0xff);
		aw_leds_i2c_write_reg(AW_LED0_T1,Fall_time<<4 | Off_time);
	}else{
		//aw_led_power_off(aw_data);
		aw_leds_i2c_write_reg(AW_LEDE,0x00);
		aw_leds_i2c_write_reg(AW_PWM0,0x00);

	}
	mutex_unlock(&aw_data->lock);
	return count;
}

static ssize_t red_blink_set_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 val;
	val = aw_leds_i2c_read_reg(AW_LED0_CTRL);
	return sprintf(buf, "%x\n", val);

}

static ssize_t red_blink_set_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	unsigned long enable = simple_strtoul(buf,NULL, 10);
	mutex_lock(&aw_data->lock);
	aw_data->ldo_ctl_flag = enable;
	if(enable)
	{
		//aw_led_power_on(aw_data);
		aw_leds_i2c_write_reg(AW_GCR,0x01);
		aw_leds_i2c_write_reg(AW_LEDE,0x01);
		aw_leds_i2c_write_reg(AW_LED0_CTRL,0x11);
		aw_leds_i2c_write_reg(AW_PWM0,0xff);
		aw_leds_i2c_write_reg(AW_LED0_T1,Fall_time<<4 | Off_time);
	}else{
		//aw_led_power_off(aw_data);
		aw_leds_i2c_write_reg(AW_LEDE,0x00);
		aw_leds_i2c_write_reg(AW_PWM0,0x00);

	}
	mutex_unlock(&aw_data->lock);
	return count;
}

static ssize_t green_led_set_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 val;
	val = aw_leds_i2c_read_reg(AW_LED1_CTRL);
	return sprintf(buf, "%x\n", val);
}

static ssize_t green_led_set_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long enable = simple_strtoul(buf,NULL, 10);
	mutex_lock(&aw_data->lock);
	aw_data->ldo_ctl_flag = enable;
	if(enable)
	{
		//aw_led_power_on(aw_data);
		aw_leds_i2c_write_reg(AW_GCR,0x01);
		aw_leds_i2c_write_reg(AW_LEDE,0x02);
		aw_leds_i2c_write_reg(AW_LED1_CTRL,0x01);
		aw_leds_i2c_write_reg(AW_PWM1,0xff);
		aw_leds_i2c_write_reg(AW_LED1_T1,Fall_time<<4 | Off_time);
	}else{
		//aw_led_power_off(aw_data);
		aw_leds_i2c_write_reg(AW_LEDE,0x00);
		aw_leds_i2c_write_reg(AW_PWM1,0x00);

	}
	mutex_unlock(&aw_data->lock);
	return count;
}

static ssize_t green_blink_set_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 val;
	val = aw_leds_i2c_read_reg(AW_LED1_CTRL);
	return sprintf(buf, "%x\n", val);
}

static ssize_t green_blink_set_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long enable = simple_strtoul(buf,NULL, 10);
	mutex_lock(&aw_data->lock);
	aw_data->ldo_ctl_flag = enable;
	if(enable)
	{
		//aw_led_power_on(aw_data);
		aw_leds_i2c_write_reg(AW_GCR,0x01);
		aw_leds_i2c_write_reg(AW_LEDE,0x02);
		aw_leds_i2c_write_reg(AW_LED1_CTRL,0x11);
		aw_leds_i2c_write_reg(AW_PWM1,0xff);
		aw_leds_i2c_write_reg(AW_LED1_T1,Fall_time<<4 | Off_time);
	}else{
		//aw_led_power_off(aw_data);
		aw_leds_i2c_write_reg(AW_LEDE,0x00);
		aw_leds_i2c_write_reg(AW_PWM1,0x00);

	}
	mutex_unlock(&aw_data->lock);
	return count;
}
#endif
//add by taokai 2014.7.22
static int aw_leds_parse_configs(struct device *dev, char *name, u32 *array) {
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
	dev_info(dev, "aw_leds_parse end\n");
	return rc;
}

static int aw_leds_parse_dt(struct device *dev, struct aw_leds_platform_data *pdata)
{
	int rc = 0;
	int index = 0;
	u32 array[22];
	struct device_node *np = dev->of_node;

	/* general_reg */
	rc = aw_leds_parse_configs(dev, "aw_leds,cfgs", array);
	if (rc) {
		dev_err(dev, "Looking up %s property in node %s failed", "aw_leds,cfgs", np->full_name);
		return -ENODEV;
	}

	pdata->led_red_sw               = array[index++];
	pdata->aw_red_on                = array[index++];
	pdata->aw_red_off                = array[index++];
	pdata->AW_RED_CTRL           = array[index++];
	pdata->AW_RED_PWM           = array[index++];
	pdata->AW_RED_T0              = array[index++];
	pdata->AW_RED_T1              = array[index++];
	pdata->AW_RED_T2              = array[index++];
	pdata->aw_green_on             = array[index++];
	pdata->aw_green_off             = array[index++];
	pdata->AW_GREEN_CTRL       = array[index++];
	pdata->AW_GREEN_PWM       = array[index++];
	pdata->AW_GREEN_T0          = array[index++];
	pdata->AW_GREEN_T1          = array[index++];
	pdata->AW_GREEN_T2          = array[index++];
	pdata->aw_blue_on              = array[index++];
	pdata->aw_blue_off              = array[index++];
	pdata->AW_BLUE_CTRL         = array[index++];
	pdata->AW_BLUE_PWM         = array[index++];
	pdata->AW_BLUE_T0             = array[index++];
	pdata->AW_BLUE_T1             = array[index++];
	pdata->AW_BLUE_T2             = array[index++];
	return rc;
}
//add end
static void red_led_blink(struct aw_leds_data *aw_data)
{
	int ret = -1;

	ret = aw_leds_i2c_write_reg(AW_GCR,0x01);
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, AW_GCR);
		return;
	}

	ret = aw_leds_i2c_write_reg(AW_LEDE,aw_data->pdata->aw_red_on);
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, AW_LEDE);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_RED_CTRL,0x11);//打开闪烁模式、电流5mA
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_RED_CTRL);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_RED_PWM,0xff);
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_RED_PWM);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_RED_T0,Rise_time<<4 | Hold_time);//亮度保持0.26s
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_RED_T0);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_RED_T1,Fall_time<<4 | Off_time);//灯灭2.08s
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_RED_T1);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_RED_T2,Delay_time);//延迟0.26s。周期2.86s
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_RED_T2);
		return;
	}

}
static void red_led_off(struct aw_leds_data *aw_data)
{
	int ret = -1;

	ret = aw_leds_i2c_write_reg(AW_LEDE,aw_data->pdata->aw_red_off);//只关闭红色灯
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, AW_LEDE);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_RED_PWM,0x00);
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_RED_PWM);
		return;
	}
}

static void green_led_blink(struct aw_leds_data *aw_data)
{
	int ret = -1;

	ret = aw_leds_i2c_write_reg(AW_GCR,0x01);
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, AW_GCR);
		return;
	}

	ret = aw_leds_i2c_write_reg(AW_LEDE,aw_data->pdata->aw_green_on);
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, AW_LEDE);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_GREEN_CTRL,0x13);//打开闪烁模式、绿灯电流15mA
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_GREEN_CTRL);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_GREEN_PWM,0xff);
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_GREEN_PWM);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_GREEN_T0,Rise_time<<4 | Hold_time);//亮度保持0.26s
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_GREEN_T0);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_GREEN_T1,Fall_time<<4 | Off_time);//灯灭2.08s
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_GREEN_T1);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_GREEN_T2,Delay_time);//延迟0.26s。周期2.86s
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_GREEN_T2);
		return;
	}
}

static void green_led_off(struct aw_leds_data *aw_data)
{
	int ret = -1;

	ret = aw_leds_i2c_write_reg(AW_LEDE,aw_data->pdata->aw_green_off);//只关闭绿色灯
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, AW_LEDE);
		return;
	}

    ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_GREEN_PWM,0x00);
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_GREEN_PWM);
		return;
	}
}

static void blue_led_blink(struct aw_leds_data *aw_data)
{
	int ret = -1;

	ret = aw_leds_i2c_write_reg(AW_GCR,0x01);
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, AW_GCR);
		return;
	}

	ret = aw_leds_i2c_write_reg(AW_LEDE,aw_data->pdata->aw_blue_on);
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, AW_LEDE);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_BLUE_CTRL,0x13);//打开闪烁模式、蓝灯电流15mA
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_BLUE_CTRL);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_BLUE_PWM,0xff);
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_BLUE_PWM);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_BLUE_T0,Rise_time<<4 | Hold_time);//亮度保持0.26s
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_BLUE_T0);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_BLUE_T1,Fall_time<<4 | Off_time);//灯灭2.08s
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_BLUE_T1);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_BLUE_T2,Delay_time);//延迟0.26s。周期2.86s
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_BLUE_T2);
		return;
	}
}

static void blue_led_off(struct aw_leds_data *aw_data)
{
	int ret = -1;

	ret = aw_leds_i2c_write_reg(AW_LEDE,aw_data->pdata->aw_blue_off);//只关闭蓝色灯
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, AW_LEDE);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_BLUE_PWM,0x00);
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_BLUE_PWM);
		return;
	}
}

static void red_led_on(struct aw_leds_data *aw_data)
{
	int ret = -1;

	ret = aw_leds_i2c_write_reg(AW_GCR,0x01);//打开全局控制寄存器，使能LED子模块
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, AW_GCR);
		return;
	}

	ret = aw_leds_i2c_write_reg(AW_LEDE,aw_data->pdata->aw_red_on);//红灯
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, AW_LEDE);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_RED_CTRL,0x03);//电流配置5mA
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_RED_CTRL);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_RED_PWM,0xff);
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_RED_PWM);
		return;
	}

	ret= aw_leds_i2c_write_reg(aw_data->pdata->AW_RED_T0,Rise_time<<4 | Hold_time);//亮度保持0.26s
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_RED_T0);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_RED_T1,Fall_time<<4 | Off_time);//灯灭2.08s
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_RED_T1);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_RED_T2,Delay_time);//延迟0.26s。周期2.86s
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_RED_T2);
		return;
	}
}
static void green_led_on(struct aw_leds_data *aw_data)
{
	int ret = -1;

	ret = aw_leds_i2c_write_reg(AW_GCR,0x01);
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, AW_GCR);
		return;
	}

	ret = aw_leds_i2c_write_reg(AW_LEDE,aw_data->pdata->aw_green_on);
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, AW_LEDE);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_GREEN_CTRL,0x03);//15ma
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_GREEN_CTRL);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_GREEN_PWM,0xff);
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_GREEN_PWM);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_GREEN_T0,Rise_time<<4 | Hold_time);//亮度保持0.26s
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_GREEN_T0);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_GREEN_T1,Fall_time<<4 | Off_time);
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_GREEN_T1);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_GREEN_T2,Delay_time);//延迟0.26s。周期2.86s
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_GREEN_T2);
		return;
	}
}
static void blue_led_on(struct aw_leds_data *aw_data)
{
	int ret = -1;

	ret = aw_leds_i2c_write_reg(AW_GCR,0x01);
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, AW_GCR);
		return;
	}

	ret = aw_leds_i2c_write_reg(AW_LEDE,aw_data->pdata->aw_blue_on);
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, AW_LEDE);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_BLUE_CTRL,0x03);//15ma
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_BLUE_CTRL);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_BLUE_PWM,0xff);
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_BLUE_PWM);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_BLUE_T0,Rise_time<<4 | Hold_time);//亮度保持0.26s
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_BLUE_T0);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_BLUE_T1,Fall_time<<4 | Off_time);
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_BLUE_T1);
		return;
	}

	ret = aw_leds_i2c_write_reg(aw_data->pdata->AW_BLUE_T2,Delay_time);//延迟0.26s。周期2.86s
	if (ret) {
		printk(KERN_ERR "%s: write_reg 0x%x failed\n", __func__, aw_data->pdata->AW_BLUE_T2);
		return;
	}
}

static ssize_t led_blink_solid_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	unsigned long enable=0;
	enum aw_led_color color = LED_COLOR_DEFAULT;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw_leds_data *aw_led_data = NULL;
	if (!strcmp(led_cdev->name, "red")) {
		color = LED_COLOR_RED;
	} else if (!strcmp(led_cdev->name, "green")) {
		color = LED_COLOR_GREEN;
	} else if (!strcmp(led_cdev->name, "blue")) {
		color = LED_COLOR_BLUE;
	}
	aw_led_data = container_of(led_cdev, struct aw_leds_data, leds[color] );
	if(!aw_led_data)
		printk(KERN_ERR "%s aw_led_data is NULL ",__func__);
	//sscanf(buf, "%d", &blink);
	 enable= simple_strtoul(buf,NULL, 10);
#if DEBUG_TRICOLOR_LED
	printk("tricolor %s is %ld\n",led_cdev->name, enable);
#endif
	mutex_lock(&aw_led_data->lock);
	if(enable){
		switch(color)
			{
			   case LED_COLOR_RED:
			   	     aw_data->led_flag[0]=1;
			   	     red_led_blink(aw_data);
				     break;
			   case LED_COLOR_GREEN:
			   	     aw_data->led_flag[1]=1;
			   	     green_led_blink(aw_data);
                     break;
				case LED_COLOR_BLUE:
			   	     aw_data->led_flag[4]=1;
			   	     blue_led_blink(aw_data);
                     break;
			    default:
			             break;
			}
		}
	else
		{
			switch(color)
			{
			   case LED_COLOR_RED:
			   	     aw_data->led_flag[0]=0;
			   	     red_led_off(aw_data);
				     break;
			   case LED_COLOR_GREEN:
			   	     aw_data->led_flag[1]=0;
			   	     green_led_off(aw_data);
                     break;
			  case LED_COLOR_BLUE:
			   	     aw_data->led_flag[4]=0;
			   	     blue_led_off(aw_data);
                     break;
			   default:
			   	     break;
			}
		}
	aw_led_data->led_data[color]=enable;
	mutex_unlock(&aw_led_data->lock);
	return size;
}
static ssize_t led_blink_solid_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
       ssize_t ret = 0;
	enum aw_led_color color = LED_COLOR_DEFAULT;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw_leds_data *aw_led_data = NULL;
	if (!strcmp(led_cdev->name, "red")) {
		color = LED_COLOR_RED;
	} else if (!strcmp(led_cdev->name, "green")) {
		color = LED_COLOR_GREEN;
	} else if (!strcmp(led_cdev->name, "blue")) {
		color = LED_COLOR_BLUE;
	}
	aw_led_data = container_of(led_cdev, struct aw_leds_data, leds[color]);
	if(!aw_led_data)
		printk(KERN_ERR "%s tricolor_led is NULL ",__func__);
	ret = sprintf(buf, "%u\n", aw_led_data->led_data[color]);
	return ret;
}

static void led_brightness_set_tricolor(struct led_classdev *led_cdev,
			       enum led_brightness brightness)
{
	struct aw_leds_data *aw_led_data = NULL;
	enum aw_led_color color = LED_COLOR_DEFAULT;

//	printk(KERN_ERR "%s ******start******* \n",__func__);
  //     printk(KERN_ERR "brightness value is %d \n",brightness);
	if (!strcmp(led_cdev->name, "red")) {
		color = LED_COLOR_RED;
	} else if (!strcmp(led_cdev->name, "green")) {
		color = LED_COLOR_GREEN;
	} else if (!strcmp(led_cdev->name, "blue")) {
		color = LED_COLOR_BLUE;
	}
	aw_led_data = container_of(led_cdev, struct aw_leds_data, leds[color]);
	if(!aw_led_data)
		printk(KERN_ERR "%s tricolor_led is NULL \n",__func__);
	mutex_lock(&aw_led_data->lock);
	if(brightness){
		switch(color) {
			case LED_COLOR_RED:
				printk(KERN_ERR "*****RED  OPEN *****\n");
				aw_data->led_flag[2]=1;
				red_led_on(aw_data);
				break;
			case LED_COLOR_GREEN:
				printk(KERN_ERR "*****GREEN OPEN*****\n");
				aw_data->led_flag[3]=1;
				green_led_on(aw_data);
				break;
			case LED_COLOR_BLUE:
				printk(KERN_ERR "*****BLUE OPEN*****\n");
				aw_data->led_flag[5]=1;
				blue_led_on(aw_data);
				break;
			default:
				break;
		}
	} else {
		switch(color) {
			case LED_COLOR_RED:
			printk(KERN_ERR "*****RED  CLOSED*****\n");
			       aw_data->led_flag[2]=0;
				red_led_off(aw_data);
				break;
			case LED_COLOR_GREEN:
			printk(KERN_ERR "*****GREEN  CLOSED*****\n");
			 	 aw_data->led_flag[3]=0;
				green_led_off(aw_data);
				break;
			case LED_COLOR_BLUE:
				printk(KERN_ERR "*****BLUE CLOSED*****\n");
				aw_data->led_flag[5]=0;
				blue_led_off(aw_data);
				break;
			default:
				break;
		}
	}
    //  printk(KERN_ERR " ******end******* \n");
       mutex_unlock(&aw_led_data->lock);
}


int aw_led_set_charge(unsigned int led_color,unsigned int on_off)
{

	unsigned int enable = on_off;
	unsigned int color = led_color;


	mutex_lock(&aw_data->lock);
	switch(color)
		{
			case 1:
			{
				if(enable)
				{
					//aw_led_power_on(aw_data);
					aw_leds_i2c_write_reg(AW_GCR,0x01);
					aw_leds_i2c_write_reg(AW_LEDE,0x01);
					aw_leds_i2c_write_reg(AW_LED0_CTRL,0x01);
					aw_leds_i2c_write_reg(AW_PWM0,0xff);
					aw_leds_i2c_write_reg(AW_LED0_T1,Fall_time<<4 | Off_time);
				}
				else
				{
					//aw_led_power_off(aw_data);
		            aw_leds_i2c_write_reg(AW_LEDE,0x00);
		            aw_leds_i2c_write_reg(AW_PWM0,0x00);

				}
			}
				break;
			case 2:
			{
				if(enable)
				{
					//aw_led_power_on(aw_data);
					aw_leds_i2c_write_reg(AW_GCR,0x01);
					aw_leds_i2c_write_reg(AW_LEDE,0x02);
					aw_leds_i2c_write_reg(AW_LED1_CTRL,0x01);
					aw_leds_i2c_write_reg(AW_PWM1,0xff);
					aw_leds_i2c_write_reg(AW_LED1_T1,Fall_time<<4 | Off_time);
				}
				else
				{
					//aw_led_power_off(aw_data);
		            aw_leds_i2c_write_reg(AW_LEDE,0x00);
		            aw_leds_i2c_write_reg(AW_PWM0,0x00);

				}
			}
				break;
			case 3:
			{
				if(enable)
				{
					//aw_led_power_on(aw_data);
					aw_leds_i2c_write_reg(AW_GCR,0x01);
					aw_leds_i2c_write_reg(AW_LEDE,0x04);//blue add by hugh
					aw_leds_i2c_write_reg(AW_LED2_CTRL,0x01);
					aw_leds_i2c_write_reg(AW_PWM2,0xff);
					aw_leds_i2c_write_reg(AW_LED2_T1,Fall_time<<4 | Off_time);
				}
				else
				{
					//aw_led_power_off(aw_data);
		            aw_leds_i2c_write_reg(AW_LEDE,0x00);
		            aw_leds_i2c_write_reg(AW_PWM0,0x00);

				}
			}
				break;
			default:
				break;

		}
	mutex_unlock(&aw_data->lock);
	return 0;
}
EXPORT_SYMBOL_GPL(aw_led_set_charge);


/*
static DEVICE_ATTR(red_led_set, 0664, red_led_set_show, red_led_set_store);
static DEVICE_ATTR(red_blink_set, 0664, red_blink_set_show, red_blink_set_store);
static DEVICE_ATTR(green_led_set, 0664, green_led_set_show, green_led_set_store);
static DEVICE_ATTR(green_blink_set, 0664, green_blink_set_show, green_blink_set_store);
static DEVICE_ATTR(blue_led_set, 0664, blue_led_set_show, blue_led_set_store);
static DEVICE_ATTR(blue_blink_set, 0664, blue_blink_set_show, blue_blink_set_store);
*/
static DEVICE_ATTR(blink, 0644, led_blink_solid_show,led_blink_solid_store);



/*
static struct attribute *aw_leds_attribute[] = {
	&dev_attr_red_led_set.attr,
	&dev_attr_red_blink_set.attr,
	&dev_attr_green_led_set.attr,
	&dev_attr_green_blink_set.attr,
	&dev_attr_blue_led_set.attr,
	&dev_attr_blue_blink_set.attr,
	NULL

};

static struct attribute_group aw_attribute_group = {
	.attrs = aw_leds_attribute

};*/




static int aw_led_resume(struct i2c_client *client)
{
#if 0
	int flag;
	flag = aw_data->ldo_ctl_flag;
	aw_led_power_on(aw_data);
	aw_leds_dev_init(aw_data);
#endif
	return 0;
}


static int aw_led_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	int i;
	for( i = 0; i < 6; i++ )
	{
		if( aw_data->led_flag[i] )
			return 0;
	}

	ret = aw_leds_i2c_write_reg(AW_GCR,0x00);//关闭所有LED子模块,低功耗模式
	if(ret)
		{
			printk(KERN_ERR "aw_led_suspend:aw_led suspend error\n");
	    		return -1;
		}
	return ret;
  }


static int aw_leds_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	int ret = 0;
	int i,j;
	struct aw_leds_platform_data *pdata;
	printk("[aw_leds]...func:%s...probe start....\n",__func__);
	if(!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WRITE_BYTE | I2C_FUNC_SMBUS_READ_BYTE_DATA))
	{
		printk(KERN_ALERT "%s: AW_LEDS functionality check failed.\n", __func__);
		ret = -EIO;
		goto exit;
	}
        printk(KERN_DEBUG"aw_leds i2c check success\n");
//add by taokai 2014.7.22
	if (client->dev.of_node) {
		pdata= devm_kzalloc(&client->dev, sizeof(struct aw_leds_platform_data), GFP_KERNEL);
		if (pdata == NULL) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			ret = -ENOMEM;
			goto out;
		}
		ret = aw_leds_parse_dt(&client->dev, pdata);
		if (ret) {
	 		dev_err(&client->dev, "Get pdata failed from Device Tree\n");
			return ret;
		}
	} else {
		pdata = client->dev.platform_data;
		if (pdata == NULL) {
			dev_err(&client->dev, "pdata is NULL\n");
			ret = -ENOMEM;
			goto out;
		}
	}
//add end
	aw_data = kzalloc(sizeof(struct aw_leds_data),GFP_KERNEL);
	if (aw_data == NULL) {
		printk(KERN_ALERT "%s: AW_LEDS kzalloc failed.\n", __func__);
		ret = -ENOMEM;
		goto out;
	}
//add by hhr 2014.7.22
	ret = gpio_request(pdata->led_red_sw, "red_leds");
	if (ret){
		pr_err("Failed to request GPIO \n");
		return ret;
	}
//add end
	aw_data->vdd_ana = regulator_get(&client->dev, "vdd_ana");
	if (IS_ERR(aw_data->vdd_ana)) {
		ret = PTR_ERR(aw_data->vdd_ana);
		pr_err("[LED]:Regulator get vdd_ana failed\n");
		goto reg_vdd_ana_put;
	}
	ret = regulator_enable(aw_data->vdd_ana);
	if (ret) {
		pr_err("[LED]:Regulator vdd_ana enable failed\n");
		return ret;
	}

	mutex_init(&aw_data->lock);
	aw_data->ldo_ctl_flag = 0;
	aw_data->client = client;
	aw_data->pdata=pdata;//add by taokai 2014.7.22
	i2c_set_clientdata(client, aw_data);
	aw_leds_dev_init(aw_data);
	aw_led_set_charge(0,0);	//close led
	gpio_direction_output(pdata->led_red_sw, 1); //added by taokai 2014.7.30
       //ret = sysfs_create_group(&client->dev.kobj,&aw_attribute_group);
    aw_data->leds[0].name = "red";
	aw_data->leds[0].brightness_set = led_brightness_set_tricolor;
    aw_data->leds[1].name = "green";
	aw_data->leds[1].brightness_set = led_brightness_set_tricolor;
	aw_data->leds[2].name = "blue";
	aw_data->leds[2].brightness_set = led_brightness_set_tricolor;

	for (i = 0; i < 3; i++) {	/* red, green,blue*/
		#ifdef CONFIG_SINK_LED_CONTRL // for charge red led by fuzicheng
		if (i == 0)
			continue;
		#endif
		ret = led_classdev_register(&client->dev, &aw_data->leds[i]);
		if (ret<0) {
			printk(KERN_ERR
			       "aw_leds: led_classdev_register failed\n");
			goto err_led_classdev_register_failed;
		}
	}

	for (i = 0; i < 3; i++) {
		#ifdef CONFIG_SINK_LED_CONTRL // for charge red led by fuzicheng
		if (i == 0)
			continue;
		#endif
				ret = device_create_file(aw_data->leds[i].dev, &dev_attr_blink);
				if (ret<0) {
					printk(KERN_ERR
						   "tricolor_led: device_create_file failed\n");
					goto err_out_attr_blink;
				}
			}

	printk("[aw_leds]...func:%s...probe end....\n",__func__);
      goto exit;

err_out_attr_blink:
	  for (j = 0; j < i; j++)
                device_remove_file(aw_data->leds[j].dev, &dev_attr_blink);

err_led_classdev_register_failed:
	for (j = 0; j < i; j++)
		led_classdev_unregister(&aw_data->leds[j]);

out:
        kfree(aw_data);
reg_vdd_ana_put:
	regulator_put(aw_data->vdd_ana);
exit:
	 return ret;
}

static int /*__devexit */aw_leds_remove(struct i2c_client *client)
{
    return 0;
}

static const struct i2c_device_id aw_leds_id[]={ { AW_LEDS_DEV_NAME, 0 },{ }, };


MODULE_DEVICE_TABLE(i2c,aw_leds_id);

#ifdef CONFIG_OF
static const struct of_device_id aw_of_match[] = {
        { .compatible = "aw-leds", },
        { },
};
MODULE_DEVICE_TABLE(of, aw_of_match);
#else
#define aw_of_match NULL
#endif
static struct i2c_driver aw_leds_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = AW_LEDS_DEV_NAME,
		#ifdef CONFIG_OF
			.of_match_table = aw_of_match,
		#endif
		  },
	.probe = aw_leds_probe,
	.remove = /*__devexit_p*/(aw_leds_remove),
	.suspend = aw_led_suspend,
	.resume = aw_led_resume,
	.id_table = aw_leds_id,

};

static int __init aw_leds_init(void)
{

	return i2c_add_driver(&aw_leds_driver);

}
static void __exit aw_leds_exit(void)
{
	i2c_del_driver(&aw_leds_driver);
	return;

}

late_initcall(aw_leds_init);
module_exit(aw_leds_exit);

MODULE_DESCRIPTION("AW three color leds sysfs driver");
MODULE_AUTHOR("yulong");
MODULE_LICENSE("GPL");
