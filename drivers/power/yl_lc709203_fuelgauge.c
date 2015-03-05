/* Copyright(C) 2013 laser-Q
 * Copyright(C) 2013 YULONG, Inc.
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
#define pr_fmt(fmt)      "LC709203:%s:" fmt, __func__

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
#include <linux/qpnp/qpnp-adc.h>
#include <linux/types.h>
#include "yl_pm8916_vbus.h"


#define DRIVER_VERSION      "1.0.0"
#define LC709203_DEV_NAME      "yl,yl_lc709203"
#define LC709203_NAME      "yl_lc709203"


/* If the system has several batteries we need a different name for each
 * of them...
 */
/*static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);*/

#define LC709203_MAX_BATT_ID_NUM      10
struct LC709203_batt_ids {
	int kohm[LC709203_MAX_BATT_ID_NUM];
	int num;
};

struct spec_battery_data {
	int                  batt_full_capa;
	int                  batt_id_kohm;
	int                  battery_appli;
	const char	*battery_type;
};
	
struct lc709203_battery_data {
	int      rpull_up_kohm;
	int      vref_batt_id_uv;
	struct spec_battery_data *spec_battery_data;
};


struct lc709203_chip {
	struct device			*dev;
	int				id;
	struct i2c_client		*client;
	// struct mutex                    i2c_mutex;
	struct  delayed_work		hw_config;
	struct qpnp_vadc_chip		*vadc_dev;
	
	struct lc709203_battery_data      *battery_data;      

	/* add for i2c error when resume*/
	bool				resume_completed;
	struct mutex			r_completed_lock;
	
	int soc_high_scaled;
	int soc_low_scaled;

	const char     *lc709203_psy_name;
	const char     *charge_psy_name;
	struct power_supply     lc709203_batt_psy;
	struct power_supply     *charge_psy;
};

/* LC709203 standard data commands */
/*add by sunxiaogang@yulong.com 2015.01.16 for the capacity jump when insert battery*/
#define LC709203_REG_SET_BEFORE_RSOC    0x04
#define LC709203_REG_SET_THERMB	0x06
#define LC709203_REG_INIT_RSOC		0x07
#define LC709203_REG_CELL_TEMP		0x08
#define LC709203_REG_CELL_VOLT		0x09
#define LC709203_REG_ADJ_APPLI		0x0B
#define LC709203_REG_ADJ_THERM		0x0C
#define LC709203_REG_RSOC			0x0D
#define LC709203_REG_RTIME			0x0F
#define LC709203_REG_IC_VER			0x11
#define LC709203_REG_RSOC_ALM	0x13
#define LC709203_REG_LOW_CELL_VOL_ALM   0x14
#define LC709203_REG_PW_Mode	0x15
#define LC709203_REG_EN_NTC			0x16

/*MACRO for CRC */
#define	dPOLYNOMIAL8			0x8380

#define LC709203_INIT_DELAY   ((HZ)*1)

static struct lc709203_chip *this_chip;


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
		struct lc709203_chip *chip)
{
	struct i2c_msg msg;
	unsigned char data[4];
	int ret;
	/*var for crc */
	static unsigned char	u1Debug = 0;
	static unsigned char	u1CRC8 = 0;

	if (!chip->client)
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

	msg.addr = chip->client->addr;
	msg.flags = 0;
	msg.len = 4;
	msg.buf = data;

	ret = i2c_transfer(chip->client->adapter, &msg, 1);
	if (ret < 0) {
		pr_err(" write transfer fail!\n");
		return -EIO;
	}

	return ret;
}


static int lc709203_read_i2c(u8 reg, void *buf, int count, struct lc709203_chip *chip)
{
	struct i2c_client *client = chip->client;
	struct i2c_msg msgs[2];
	int err=0;

	/*var for crc */
	static unsigned char	u1Debug = 0;
	static unsigned char	u1CRC8 = 0;
	unsigned char data[3];

	if (!client->adapter)
	{
		pr_err("adapter pointer null\n");
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
		pr_err( "transfer fail\n");
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
			pr_debug("read CRC verify success......\n");

		}
	}
	return err;
}

static s16 lc709203_read_reg(u8 reg, struct lc709203_chip *chip)
{
	s32 val;
	s16 val_final;
	s16 err = 0;
	u8 data[3];

	err = lc709203_read_i2c(reg,&val,3, chip);
	if(err < 0)
	{
		pr_err("cmd 0x%02X  read fail \n", reg);
		return err;
	}
	data[0] = (val&0x00FF);
	data[1] = (val&0xFF00)>>8;
	val_final = (data[1]<<8)|data[0];	
	pr_debug("read reg = 0x%02X.val = 0x%02X.\n", reg , val_final);

	return val_final;
}
static void lc709203_reg_write(u8 reg,int subcmd,struct lc709203_chip *chip	)
{
	int err=0;
	err = lc709203_write_i2c(reg, (unsigned short)subcmd, chip);
	if(err < 0)
	{
		pr_err("reg = 0x%02X write fail \n", reg );	
	}else
		pr_debug("write reg = 0x%02X.val = 0x%02X.  success \n", reg , subcmd);

}

/*
 * write the battery temperature to the lc709203 chip
 * 
 */
static void update_batt_temp_to_lc709203(struct lc709203_chip *chip, int batt_temp)
{

	batt_temp = batt_temp + 0xAAC;
	lc709203_reg_write(LC709203_REG_SET_THERMB, batt_temp, chip);
	udelay(66);
	pr_debug("batt_temp = %d  batt_temp:%x\n", batt_temp,batt_temp);
}


/*
 * Return the battery temperature in tenths of degree Celsius
 * 
 */
static int lc709203_batt_temperature(struct lc709203_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;
	if (!this_chip) {
		pr_err("this_chip is unavailable\n");
		return -EINVAL;
	}
	
	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM, &results);
	if (rc) {
		pr_err("unable to read batt temp rc = %d \n", rc);
		return 0;
	}
	
	pr_debug("read batt temp= %lld \n", results.physical);
	return (int)results.physical;	
}

/*
 * Return the battery Voltage in milivolts
 * 
 */
static int lc709203_batt_voltage(struct lc709203_chip *chip)
{
	int  m_volt;		

	m_volt = lc709203_read_reg(LC709203_REG_CELL_VOLT, chip);
	pr_debug(": batt volt %d\n", m_volt);

	return m_volt;
}

static int bound_soc(int soc)
{
	soc = max(0, soc);
	soc = min(100, soc);

	return soc;
}

static int soc_scaled(struct lc709203_chip *chip, int soc)
{
	int new_rsoc = 0;
	pr_debug("soc_high_scaled = %d soc_low_scaled = %d \n", chip->soc_high_scaled, chip->soc_low_scaled);    
    soc = 10*soc;
    if(soc >= chip->soc_high_scaled)
		return 100;
	else if(soc < chip->soc_low_scaled)
		return 0;

	new_rsoc = 1000*(soc - chip->soc_low_scaled)/(chip->soc_high_scaled - chip->soc_low_scaled);
	
	if((new_rsoc%10) >= 5){
		new_rsoc = new_rsoc/10 + 1;
	}else{
		new_rsoc = new_rsoc/10;
	}
	return new_rsoc;
}
/*
 * Return the battery capacity in d%
 * 
 */
static int lc709203_batt_capacity(struct lc709203_chip *chip)
{
	int capa = 0;
	int new_rsoc = 0;

	capa = lc709203_read_reg(LC709203_REG_RSOC, chip);	
	pr_debug("capa = %d  \n", capa);
	capa = bound_soc(capa);
	new_rsoc = soc_scaled(chip, capa);

	pr_debug("capa = %d new_rsoc = %d \n", capa, new_rsoc);
	new_rsoc = bound_soc(new_rsoc);
	pr_debug("new_rsoc = %d  \n", new_rsoc);
	return new_rsoc;
}

/* power supply segment */
static enum power_supply_property lc709203_batt_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

static int lc709203_batt_get_property(struct power_supply *psy, 
		enum power_supply_property prop,
		union power_supply_propval *val)
{
	struct lc709203_chip *chip = container_of(psy, struct lc709203_chip, lc709203_batt_psy);
	int batt_temp = 0;
	
	mutex_lock(&chip->r_completed_lock);
	if (!chip->resume_completed) {
		dev_dbg(chip->dev, "read LC709203 before device-resume\n");
		mutex_unlock(&chip->r_completed_lock);
		return 0;
	}
	mutex_unlock(&chip->r_completed_lock);
	
	switch (prop) {
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = 1;
			break;	
		case POWER_SUPPLY_PROP_CAPACITY:
			batt_temp = lc709203_batt_temperature(chip);
			update_batt_temp_to_lc709203(chip,batt_temp);
			val->intval = lc709203_batt_capacity(chip);
			break;
		case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
			val->intval = 2500;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
			val->intval = 4350000;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval = lc709203_batt_voltage(chip);
			break;
		case POWER_SUPPLY_PROP_TEMP:
			val->intval = lc709203_batt_temperature(chip);
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static int lc709203_batt_is_writeable(struct power_supply *psy,
		enum power_supply_property prop)
{
	int rc;

	switch (prop) {
		case POWER_SUPPLY_PROP_CAPACITY:
		case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		case POWER_SUPPLY_PROP_CHARGE_FULL:
			rc = 1;
			break;
		default:
			rc = 0;
			break;
	}
	return rc;
}

static int lc709203_batt_set_property(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	//struct lc709203_chip *chip = container_of(psy,
	//			struct lc709203_chip, lc709203_batt_psy);

	switch (prop) {
		case POWER_SUPPLY_PROP_CAPACITY:
			break;
		case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
			break;
		case POWER_SUPPLY_PROP_CHARGE_FULL:	
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static int64_t of_batterydata_convert_battery_id_kohm(int batt_id_uv,
				int rpull_up, int vadc_vdd)
{
	int64_t resistor_value_kohm, denom;

	if (batt_id_uv == 0) {
		/* vadc not correct or batt id line grounded, report 0 kohms */
		return 0;
	}
	/* calculate the battery id resistance reported via ADC */
	denom = div64_s64(vadc_vdd * 1000000LL, batt_id_uv) - 1000000LL;

	if (denom == 0) {
		/* batt id connector might be open, return 0 kohms */
		return 0;
	}
	resistor_value_kohm = div64_s64(rpull_up * 1000000LL + denom/2, denom);

	pr_debug("batt id voltage = %d, resistor value = %lld\n", batt_id_uv, resistor_value_kohm);

	return resistor_value_kohm;
}

static int64_t read_battery_id_uv(struct lc709203_chip *chip)
{
	int rc;
	struct qpnp_vadc_result result;

	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX2_BAT_ID, &result);
	if (rc) {
		pr_err("error reading batt id channel = %d, rc = %d\n",
				LR_MUX2_BAT_ID, rc);
		return rc;
	}

	return result.physical;
}

static int set_battery_data(struct lc709203_chip *chip)
{
	struct lc709203_battery_data *batt_data;
	struct device_node *batt_node;
	struct device_node *best_batt_node;
	struct device_node *node;
	struct LC709203_batt_ids batt_ids;
//	const char	*battery_type=NULL;
	int64_t battery_id_uv;
	int rc = 0;
	int delta, batt_id_kohm, best_delta, best_id_kohm, batt_num, i;


	batt_node = of_find_node_by_name( chip->dev->of_node,
					"yl,battery-data");
	if (!batt_node) {
			pr_err("No available batterydata\n");
			return -EINVAL;
	}
	
	battery_id_uv = read_battery_id_uv(chip);
	if (battery_id_uv < 0) {
		pr_err("cannot read battery id err = %lld\n", battery_id_uv);
		return battery_id_uv;
	}

	batt_data = devm_kzalloc(chip->dev,
			sizeof(struct lc709203_battery_data), GFP_KERNEL);
	if (!batt_data) {
		pr_err("Could not alloc battery data\n");
		return -EINVAL;
	}

	batt_data->spec_battery_data = devm_kzalloc(chip->dev,
		sizeof(struct spec_battery_data), GFP_KERNEL);

	rc = of_property_read_u32(batt_node, "yl,rpull-up-kohm", &batt_data->rpull_up_kohm);
	if (rc){ 
		dev_err(chip->dev, "Failed to read battery rpull_up_kohm\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(batt_node, "yl,vref-batt-id-uv", &batt_data->vref_batt_id_uv);
	if (rc){ 
		dev_err(chip->dev, "Failed to read battery vref_batt_id_uv\n");
		return -EINVAL;
	}

	batt_id_kohm = of_batterydata_convert_battery_id_kohm(battery_id_uv,
					batt_data->rpull_up_kohm, batt_data->vref_batt_id_uv);

	/*
	 * Find the battery data with a battery id resistor closest to this one
	 */
	node = batt_node;
	best_batt_node = NULL;
	best_delta = 0;
	best_id_kohm = 0;
	batt_num = 0;
	
	for_each_child_of_node(batt_node, node) {
		rc = of_property_read_u32(node, "yl,batt-id-kohm", &batt_ids.kohm[batt_num]);
		if (rc){ 
			dev_err(chip->dev, "Failed to read battery batt_id_kohm\n");
			return -EINVAL;
		}

		batt_num++;
		if (LC709203_MAX_BATT_ID_NUM <= batt_num) {
			pr_err("Too many battery id resistors\n");
			break;
		}
		batt_ids.num = batt_num;
		
		if (LC709203_MAX_BATT_ID_NUM < batt_ids.num) {
			pr_err("Too many battery id resistors\n");
			break;
		}
		
		for (i = 0; i < batt_ids.num; i++) {
			delta = abs(batt_ids.kohm[i] - batt_id_kohm);
			if (delta < best_delta || !best_batt_node) {
				best_batt_node = node;
				best_delta = delta;
				best_id_kohm = batt_ids.kohm[i];
			}
		}
	}

	if (best_batt_node == NULL) {
		pr_err("No battery data found\n");
		return -ENODATA;
	}

	rc = of_property_read_string(best_batt_node, "yl,battery-type",
							&batt_data->spec_battery_data->battery_type);
	if (rc){ 
		dev_err(chip->dev, "Failed to read battery battery_type\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(best_batt_node, "yl,batt-id-kohm", &batt_data->spec_battery_data->batt_id_kohm);
	if (rc){ 
		dev_err(chip->dev, "Failed to read battery batt_id_kohm\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(best_batt_node, "yl,batt-full-capa", &batt_data->spec_battery_data->batt_full_capa);
	if (rc){ 
		dev_err(chip->dev, "Failed to read battery batt_full_capa\n");
		batt_data->spec_battery_data->batt_full_capa = 2500;
		rc = 0;
	}

	rc = of_property_read_u32(best_batt_node, "yl,battery-appli", &batt_data->spec_battery_data->battery_appli);
	if (rc){ 
		dev_err(chip->dev, "Failed to read battery battery_appli\n");
		batt_data->spec_battery_data->battery_appli = 0x0025;
		rc = 0;
	}

	pr_info("battery %s loaded , appli:0x%04X \n", batt_data->spec_battery_data->battery_type, 
		batt_data->spec_battery_data->battery_appli);	

	chip->battery_data = batt_data;
	
	return 0;
}

static int lc709203_parse_dt(struct lc709203_chip *chip)
{
	int rc;
	struct device_node *node = chip->dev->of_node;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing \n");
		return -EINVAL;
	}

	rc = of_property_read_string(node, "yl,lc709203-psy-name", &chip->lc709203_psy_name);
	if (rc) { 
		dev_err(chip->dev, "Failed to read lc709203_psy_name\n");
		chip->lc709203_psy_name = "lc709203_battery";
		rc = 0;
	}

	rc = of_property_read_string(node, "yl,charge-psy-name", &chip->charge_psy_name);
	if (rc){ 
		dev_err(chip->dev, "Failed to read charge_psy_name\n");
		chip->charge_psy_name = "battery";
		rc = 0;
	}

	rc = of_property_read_u32(node, "yl,soc-high-scaled", &chip->soc_high_scaled);
	if (rc){ 
		dev_err(chip->dev, "Failed to read soc_high_scaled\n");
		chip->soc_high_scaled = 100;
		rc = 0;
	}


	rc = of_property_read_u32(node, "yl,soc-low-scaled", &chip->soc_low_scaled);
	if (rc){ 
		dev_err(chip->dev, "Failed to read soc_low_scaled\n");
		chip->soc_low_scaled = 0;
		rc = 0;
	}
	pr_info(" soc_high_scaled = %d soc_low_scaled = %d\n", chip->soc_high_scaled,chip->soc_low_scaled);

	return rc;
}

static int lc709203_hw_init_by_battery(struct lc709203_chip *chip)
{
	s16 ic_ver = 0;
        int rc = 0;     //add by sunxiaogagn@yulong.com 2015.01.16 for the capacity jump when isert battery

	set_battery_data(chip);
        /*add begin by sunxiaogagn@yulong.com 2015.01.16 for the capacity jump when isert battery*/
        rc = lc709203_read_reg(LC709203_REG_ADJ_APPLI, chip);
        pr_info("warm reset register value = 0x%x\n",rc);
        if(chip->battery_data->spec_battery_data->battery_appli != rc)
        {
                lc709203_reg_write(LC709203_REG_SET_BEFORE_RSOC, 0xAA55, chip);
                pr_info("LC709203_REG_SET_BEFORE_RSOC is set\n");
                udelay(100);
        }
        /*add end by sunxiaogagn@yulong.com 2015.01.16*/

	lc709203_reg_write(LC709203_REG_ADJ_APPLI, chip->battery_data->spec_battery_data->battery_appli, chip);

	ic_ver = lc709203_read_reg(LC709203_REG_IC_VER, chip);
	udelay(66);
	pr_info("IC_VERSION is 0x%02X\n",ic_ver);
	return 0;
}

#define BATTERY_CAPACITY_FIFTEEN  15
static ssize_t fuel_gauge_capacity_store(struct device_driver *drv, const char * buf, size_t count)
{
	char *after;
	unsigned long data = 0;
	data = simple_strtoul(buf, &after, 10);

	if (NULL == this_chip) {
		pr_err( "lc709203_chip is NULL ERROR \n");
		return -EINVAL;
	}

	switch(data)
	{
		case BATTERY_CAPACITY_FIFTEEN:
			lc709203_reg_write(LC709203_REG_RSOC,0x000F,this_chip);
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

	if (NULL == this_chip) {
		pr_err( "lc709203_chip is NULL ERROR \n");
		return -EINVAL;
	}
	ret= lc709203_read_i2c(LC709203_REG_RSOC,&val,3,this_chip);
	if(ret<0)
	{
		pr_err( "read cpacity error \n");
		return -22;
	}
	//val_final = (val & 0x0000) >>16;
	//val_final = ((val_final & 0xFF00)>>8) | ((val_final & 0x00FF) << 8); 
	data[0] = (val&0x00FF);
	data[1] = (val&0xFF00)>>8;
	val_final = (data[1]<<8)|data[0];
	pr_info( "data[0] = %d, data[1] = %d\n",data[0], data[1]);

	return snprintf(buf, PAGE_SIZE, "%d\n", val_final);
}

static DRIVER_ATTR(batt_capacity, 0664, fuel_gauge_capacity_show, fuel_gauge_capacity_store);


static int lc709203_fuelgauger_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	//char *name;
	struct lc709203_chip *chip;
	int retval = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	chip = kzalloc(sizeof(struct lc709203_chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}
	memset(chip,0,sizeof(struct lc709203_chip));
	i2c_set_clientdata(client, chip);
	chip->dev = &client->dev;
	chip->client = client;
	retval = lc709203_parse_dt(chip);
	if (retval < 0) {
		dev_err(&client->dev, "lc709203 Unable to parse DT nodes\n");
		//return rc;
	}

	lc709203_reg_write(LC709203_REG_PW_Mode,0x0001,chip);
	udelay(100);
	lc709203_reg_write(LC709203_REG_PW_Mode,0x0001,chip);
	
	chip->resume_completed = true;
	mutex_init(&chip->r_completed_lock);
	chip->vadc_dev = qpnp_get_vadc(chip->dev, "lc");
	if (IS_ERR(chip->vadc_dev)) {
		retval = PTR_ERR(chip->vadc_dev); 
		pr_info("vadc_dev prop missing retval = %d \n", retval);
		kfree(chip);
		return retval;
	}
	
	lc709203_hw_init_by_battery(chip);

	/* register lc709203 battery power supply */
	chip->lc709203_batt_psy.name		= chip->lc709203_psy_name;
	chip->lc709203_batt_psy.type		= POWER_SUPPLY_TYPE_YL_BATTERY;
	chip->lc709203_batt_psy.get_property	= lc709203_batt_get_property;
	chip->lc709203_batt_psy.property_is_writeable = lc709203_batt_is_writeable;
	chip->lc709203_batt_psy.set_property	= lc709203_batt_set_property;
	chip->lc709203_batt_psy.properties	= lc709203_batt_properties;
	chip->lc709203_batt_psy.num_properties  = ARRAY_SIZE(lc709203_batt_properties);

	retval = power_supply_register(chip->dev, &chip->lc709203_batt_psy);
	if (retval < 0) {
		dev_err(chip->dev, "Unable to register lc709203_batt_psy retval = %d\n", retval);
		goto unregister_batt_psy;
	}

	this_chip = chip;

	return 0;

batt_failed_1:
	kfree(chip);

unregister_batt_psy:
	return retval;
}

static int lc709203_fuelgauger_remove(struct i2c_client *client)
{
	struct lc709203_chip *chip = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&chip->hw_config);

	/*
	   mutex_lock(&battery_mutex);
	   idr_remove(&battery_id, chip->id);
	   mutex_unlock(&battery_mutex);
	 */	
	mutex_destroy(&chip->r_completed_lock);

	kfree(chip);
	return 0;
}

static const struct i2c_device_id lc709203_id[] = {
	{ LC709203_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, lc709203_id);

static int lc709203_suspend(struct device *dev)
{
	int rc = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct lc709203_chip *chip = i2c_get_clientdata(client);
	
	/* add active */
	mutex_lock(&chip->r_completed_lock);
	chip->resume_completed = false;
	mutex_unlock(&chip->r_completed_lock);
	
	return rc;
}


static int lc709203_resume(struct device *dev)
{
	int rc = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct lc709203_chip *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->r_completed_lock);
	chip->resume_completed = true;
	mutex_unlock(&chip->r_completed_lock);

	return rc;
}

static const struct dev_pm_ops lc709203_pm_ops = {
	.resume = lc709203_resume,
	.suspend = lc709203_suspend,
};


static struct of_device_id lc709203_match_table[] = {
	{.compatible = LC709203_DEV_NAME,},
	{},
};


static struct i2c_driver lc709203_fuelgauger_yl_driver = {
	.driver		= {
		.name = LC709203_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lc709203_match_table,
		.pm = &lc709203_pm_ops,
	},
	.probe		= lc709203_fuelgauger_probe,
	.remove		= lc709203_fuelgauger_remove,
	.id_table	= lc709203_id,
};

static int __init lc709203_fuelgauger_init(void)
{
	int ret;

	ret = i2c_add_driver(&lc709203_fuelgauger_yl_driver);
	if (ret)
		pr_err("Unable to register lc709203 driver\n");

	ret = driver_create_file(&lc709203_fuelgauger_yl_driver.driver, &driver_attr_batt_capacity);
	if (ret)
		pr_err("%s: driver ceate file error! \n", __func__);

	return ret;
}

#ifdef CONFIG_PRIORITIZE_LOAD_LC709203
/* make sure prioritized load LC709203 driver */
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

