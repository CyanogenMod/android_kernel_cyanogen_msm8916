/* Copyright (c) 2013-2014 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 *FAN5405 charge IC driver file, sm means switch mode
 *
 */

#define pr_fmt(fmt) "FAN5405: %s: " fmt, __func__

#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/bitops.h>
#include <linux/alarmtimer.h>
#include <linux/wakelock.h>
#include <linux/qpnp/power-on.h>
#include <linux/boot_params.h>
#include "yl_pm8916_vbus.h"
#include "lc709203_fuelgauger_yl.h"

extern int yl_get_lcd_status;
extern int yl_get_bk_level;

struct fan5405_chip {
	struct device         *dev;
	struct i2c_client      *client;

	unsigned en_gpio;
	unsigned irq_gpio;
	u32      irq_flag;
	int      irq;	
	/* configuration data - charge */
	bool      charging_disabled;
	bool      enable_te;
	int         vbus_curr_max;
	int         weak_batt_vol;
	int         chg_vol_max;
	int         chg_curr_max;
	int         chg_curr_now;
	int         iterm_ma;
	int         vsp_vol;
	int         safe_curr;
	int         safe_vol;

	bool	     shutdown_soc_invalid;
	bool      vbus_present;
	/* battery status tracking */
	bool      batt_present;
	bool      batt_too_cold;
	bool      batt_too_hot;
	int        batt_temp;
	int        batt_temp_status;
	int        batt_curr;
	int        batt_volt;
	int        batt_crude_volt;
	int        batt_status;
	int        batt_crude_capa;
	int        pon_volt;
	int        pon_volt_delta;
	int 	    shutdown_ocv;
	int 	    shutdown_soc;
	int 	    warm_reset;

	int        charge_stat;
	int        to_sys_capa;
	int        last_to_sys_capa;
	int        report_soc;

	int        dischg_batt_mvolt_delta;
	/*  */
	int        set_ivbus_max;

	/* add for i2c error when resume*/
	bool				resume_completed;
	bool				update_heartbeat_waiting;
	bool				update_t32s_waiting;
	bool				external_change_waiting;
	struct mutex			r_completed_lock;
	
	/* add alarm for batt info report */
	struct wake_lock          batt_info_alarm_wlock;
	struct work_struct batt_info_alarm_work;
	struct alarm report_batt_info_alarm;

	struct power_supply     *usb_psy;
	struct power_supply     batt_psy;
	
	//struct wake_lock          fan5405_wlock;
	struct delayed_work     update_t32s_work;
	struct delayed_work     update_heartbeat_work;

	struct mutex			reg_rw_lock;
	
	struct pinctrl *int_pinctrl;
	struct pinctrl_state *int_state_active;
	struct pinctrl_state *int_state_suspend;

};

struct fan5405_batt_capa_data {
	int capa;
	int volt;
};

static struct fan5405_chip *this_chip;
#define CONFIG_BATTERY_LC709203   //ADDED for LC709203

/* define registers macro segment */
#define FAN5405_CONTROL0        0x00
#define FAN5405_CONTROL1        0x01
#define FAN5405_OREG                0x02
#define FAN5405_IC_INFO           0x03
#define FAN5405_IBAT                 0x04
#define FAN5405_SP_CHARGER    0x05
#define FAN5405_SAFETY             0x06
#define FAN5405_MONITOR          0x10

/* register 0x00 mask*/
#define TMR_RST_MASK                BIT(7)
#define EN_STAT_MASK                BIT(6)
/* register 0x01 mask*/
#define INPUT_CURR_LIMIT_MASK     0xC0
#define WEAK_BATT_VOL_MASK        0x30
#define TERM_EN_MASK              BIT(3)
#define CAHRGE_EN_MASK            BIT(2)
#define HZ_MODE_MASK              BIT(1)
#define OPA_MODE_MASK             BIT(0)
/* register 0x02 mask*/
/* set charge output voltge */
#define OREG_MASK                      0xFC
#define OTG_PL                             BIT(1)
#define OTG_EN                             BIT(0)
/* register 0x03 mask*/
/* register 0x04 mask*/
#define RESET_MASK        BIT(7)
/* maximum charge current */
#define IOCHARGE_MASK     0x70
#define ITEM_MASK         0x07
/* register 0x05 mask*/
#define DIS_VREG_MASK      BIT(6)
#define IO_LEVEL           BIT(5)
#define VSP_MASK           0x07
/* register 0x05 mask*/
#define ISAFE_MASK         0x70
#define VSAGE_MASK         0x0F


enum {
	UNKNOWN_CHG_TYPE = 0,
	PC_USB_CHG_TYPE,
	WALL_CHARGER_CHG_TYPE,
	UNSTANDARD_CHG_TYPE,
};

enum {
	DEFUALT_CHARGING_MODE = 0x00, /* NORMAL_CHARGING_MODE */
	USB_CHARGING_MODE = 0x01,         /* charging current max = 500mA */
	NORMAL_CHARGING_MODE = 0x02,  /* charging current max = 1000mA */
	FAST_CHARGING_MODE = 0x0F,        /* charging current max = 1500mA */
	DIS_CHARGING_MODE = 0xFF,          /* disable charge */
	UNKOWN_CHARGING_MODE,            /* NORMAL_CHARGING_MODE */
};


enum batt_temp_status {
	BATT_TEMP_STATUS_UNKOWN = 0,
	BATT_TEMP_TOO_COLD, /* battery temp < 0 celsius*/
	BATT_TEMP_COLD,                 /* 0 celsius <= battery temp < 10 celsius */
	BATT_TEMP_COOL,                 /* 10 celsius <= battery temp < 20 celsius */
	BATT_TEMP_NUM,                   /* 20 celsius <= battery temp <= 45 celsius */
	BATT_TEMP_HOT,                   /* 45 celsius < battery temp < 60 celsius */
	BATT_TEMP_TOO_HOT,         /* 60 celsius <= battery temp */
};

enum batt_temp_threshold {
	BATT_TEMP_TOO_COLD_THRESHOLD = 0,           /*  0 celsius*/
	BATT_TEMP_COLD_THRESHOLD = 100,                 /* 10 celsius */
	BATT_TEMP_COOL_THRESHOLD = 200,                 /* 20 celsius */
	BATT_TEMP_NUM_THRESHOLD = 480,                   /* 45 celsius */
	BATT_TEMP_HOT_THRESHOLD = 600,                   /* 60 celsius */
};

#define BATT_FULL_CAPA   2500

/* limit current is 0.1C */
#define CHG_CURR_LIMIT_COLD                        (BATT_FULL_CAPA / 10)
/* limit current is 0.3C */
#define CHG_CURR_LIMIT_COOL_4P0V             (3*BATT_FULL_CAPA /10)
/* limit current is 0.1C */
#define CHG_CURR_LIMIT_COOL_4P35V           (BATT_FULL_CAPA /10)
/* limit current is 0.7C */
#define CHG_CURR_LIMIT_NUM                          (7*BATT_FULL_CAPA /10)
/* limit current is 0.3C */
#define CHG_CURR_LIMIT_HOT_4P0V                (3*BATT_FULL_CAPA /10)



#define PON_VOLT_DELTA_DEFAULT 50
/* 50mV */
#define DISCHG_BATT_MVOLT_DELTA_DEFAULT 50
#define CHARGING_BATT_MVOLT_DELTA_DEFAULT 50
#define SOC_UNINITIALIZED         -99

static struct fan5405_batt_capa_data batt_capa_data[] = {
	{100, 4319},
	{95,  4257},
	{90,  4201},
	{85,  4146},
	{80,  4094},
	{75,  4050},
	{70,  3983}, 
	{65,  3942}, 
	{60,  3897}, 
	{55,  3864}, 
	{50,  3837}, 
	{45,  3815}, 
	{40,  3797}, 
	{35,  3779}, 
	{30,  3762}, 
	{25,  3746}, 
	{20,  3728}, 
	{15,  3705}, 
	{10,  3686}, 
	{9,   3684}, 
	{8,   3680}, 
	{7,   3671}, 
	{6,   3649}, 
	{5,   3612}, 
	{4,   3566}, 
	{3,   3508}, 
	{2,   3430}, 
	{1,   3313}, 
	{0,   3000}, 
};


/* i2c operation API segment */
static int __fan5405_read(struct fan5405_chip *chip, int reg,
				u8 *val)
{
	s32 ret = 0;

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		dev_err(chip->dev,
			"i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} 
	
	*val = ret;
	pr_debug("Read 0x%02x=0x%02x\n", reg, *val);
	
	return 0;
}

static int __fan5405_write(struct fan5405_chip *chip, int reg,
						u8 val)
{
	s32 ret = 0;

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		dev_err(chip->dev,
			"i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}
	
	pr_debug("Writing 0x%02x=0x%02x\n", reg, val);
	
	return 0;
}

static int fan5405_read(struct fan5405_chip *chip, int reg,
				u8 *val)
{
	int rc = 0;

	mutex_lock(&chip->reg_rw_lock);
	rc = __fan5405_read(chip, reg, val);
	mutex_unlock(&chip->reg_rw_lock);

	return rc;
}

static int fan5405_write(struct fan5405_chip *chip, int reg,
						u8 val)
{
	int rc = 0;

	mutex_lock(&chip->reg_rw_lock);
	rc = __fan5405_write(chip, reg, val);
	mutex_unlock(&chip->reg_rw_lock);

	return rc;
}

#if 0
static int fan5405_read_bytes(struct fan5405_chip *chip, int reg,
						u8 *val, u8 bytes)
{
	s32 rc;

	mutex_lock(&chip->reg_rw_lock);
	rc = i2c_smbus_read_i2c_block_data(chip->client, reg, bytes, val);
	if (rc < 0)
		dev_err(chip->dev,
			"i2c read fail: can't read %d bytes from %02x: %d\n",
							bytes, reg, rc);
	mutex_unlock(&chip->reg_rw_lock);

	return (rc < 0) ? rc : 0;
}
#endif

static int fan5405_masked_write(struct fan5405_chip *chip, int reg,
						u8 mask, u8 val)
{
	s32 rc = 0;
	u8 temp = 0;

	mutex_lock(&chip->reg_rw_lock);
	rc = __fan5405_read(chip, reg, &temp);
	if (rc < 0) {
		dev_err(chip->dev, "read failed: reg=%02x, rc=%d\n", reg, rc);
		goto out;
	}
	temp &= ~mask;
	temp |= val & mask;
	pr_debug("write: reg=0x%02x, temp=0x%02x\n", reg, temp);
	rc = __fan5405_write(chip, reg, temp);
	if (rc < 0) {
		dev_err(chip->dev,
			"write failed: reg=%02x, rc=%d\n", reg, rc);
	}
out:
	mutex_unlock(&chip->reg_rw_lock);
	return rc;
}

#if 0
/* operate FAN5405 API segment */
static int fan5405_32s_tmr_rst(struct fan5405_chip *chip)
{
	return fan5405_masked_write(chip, FAN5405_CONTROL0,
				TMR_RST_MASK, TMR_RST_MASK);

}

static int fan5405_enable_stat(struct fan5405_chip *chip, bool enable)
{

	u8 temp = 0;

	if ( enable )
		temp = EN_STAT_MASK;
	
	return fan5405_masked_write(chip, FAN5405_CONTROL0,
				EN_STAT_MASK, temp);
}
#endif

static int fan5405_set_ivbus_max(struct fan5405_chip *chip, int current_ma)
{
	u8 temp = 0;
	u8 val = 0;
	
	//if (current_ma <= 100)
	//	temp = 0x00;
	//else if (current_ma <= 500)
	if (current_ma <= 500)
		temp = 0x01;
	else if (current_ma <= 800)
		temp = 0x02;
	else
		temp = 0x03;

	val = (temp) << 6;
		
	pr_debug("Writing 0x%02x (7-6bit) = 0x%02x, current = %d\n", FAN5405_CONTROL1, val, current_ma);
	
	return fan5405_masked_write(chip, FAN5405_CONTROL1,
				INPUT_CURR_LIMIT_MASK, val);

}

#define WEAK_BATT_VOL_MIN      3400
#define WEAK_BATT_VOL_MAX      3700
#define WEAK_BATT_VOL_STEP       100
static int fan5405_set_weak_batt_vol(struct fan5405_chip *chip, int vol_mv)
{
	u8 temp = 0;

	if (vol_mv < WEAK_BATT_VOL_MIN ||vol_mv > WEAK_BATT_VOL_MAX) {
		dev_err(chip->dev, "set the weak battery threshold value error.\n");
		return -EINVAL;
	}
		
		
	temp = (vol_mv - WEAK_BATT_VOL_MIN) / WEAK_BATT_VOL_STEP;
	temp = (temp) << 4;
		
	pr_debug("Writing 0x%02x (5-4bit) = 0x%02x\n", FAN5405_CONTROL1, temp);
	
	return fan5405_masked_write(chip, FAN5405_CONTROL1,
				WEAK_BATT_VOL_MASK, temp);

}

static int fan5405_enable_te(struct fan5405_chip *chip, bool enable)
{
	u8 temp = 0;

	if (enable)
		temp = TERM_EN_MASK;
	
	return fan5405_masked_write(chip, FAN5405_CONTROL1,
				TERM_EN_MASK, temp);
}

static int fan5405_enable_chg(struct fan5405_chip *chip, bool enable)
{
	u8 temp = 0;

	if ( !enable )
		temp = CAHRGE_EN_MASK;
	
	return fan5405_masked_write(chip, FAN5405_CONTROL1,
				CAHRGE_EN_MASK, temp);
}

/* enable choose High impedance mode, disable choose not High impedance mode */
static int fan5405_enable_HZ_mode(struct fan5405_chip *chip, bool enable)
{
	u8 temp = 0;

	if ( enable )
		temp = HZ_MODE_MASK;
	
	return fan5405_masked_write(chip, FAN5405_CONTROL1,
				HZ_MODE_MASK, temp);
}

/* enable choose boost mode for otg function, disable choose charge mode */
static int fan5405_enable_OPA_mode(struct fan5405_chip *chip, bool enable)
{
	u8 temp = 0;

	if ( enable )
		temp = OPA_MODE_MASK;
	
	return fan5405_masked_write(chip, FAN5405_CONTROL1,
				OPA_MODE_MASK, temp);
}

#define CHG_VOL_MIN      3500
#define CHG_VOL_MAX      4440
#define CHG_VOL_STEP     20
static int fan5405_set_chg_vol_max(struct fan5405_chip *chip, int vol_mv)
{
	u8 temp = 0;
	
	if (vol_mv < CHG_VOL_MIN ||vol_mv > CHG_VOL_MAX) {
		dev_err(chip->dev, "set the weak battery threshold value error.\n");
		return -EINVAL;
	}
		
		
	temp = (vol_mv - CHG_VOL_MIN) / CHG_VOL_STEP;
	temp = (temp) << 2;
		
	pr_debug("Writing 0x%02x (7-2bit) = 0x%02x vol_mv=%d\n", FAN5405_OREG, temp, vol_mv);
	
	return fan5405_masked_write(chip, FAN5405_OREG,
				OREG_MASK, temp);
}

static int fan5405_set_otg_pl(struct fan5405_chip *chip, bool level)
{
	u8 temp = 0;

	if ( level )
		temp = OTG_PL;
	
	return fan5405_masked_write(chip, FAN5405_OREG,
				OTG_PL, temp);
}

static int fan5405_enable_otg(struct fan5405_chip *chip, bool enable)
{
	u8 temp = 0;

	if ( enable )
		temp = OTG_EN;
	
	return fan5405_masked_write(chip, FAN5405_OREG,
				OTG_EN, temp);
}

/* when Rsense = 68 */
#define CHARGE_CURR_MIN      550
#define CHARGE_CURR_MAX      1250
#define CHARGE_CURR_STEP     50
static const int charge_curr_limit[] = {550, 650, 750, 850, 950, 1050, 1150, 1250,};
/* when Rsense = 68 */
#define I_TERM_MIN      49
#define I_TERM_MAX      388
#define I_TERM_STEP      388
static const int term_curr_limit[] = {49, 97, 146, 194, 243, 291, 340, 388,};
static int fan5405_set_iterm_ichg(struct fan5405_chip *chip, int current_ma, int term_ma)
{
	u8 i = 0;
	u8 temp = 0;
	u8 val = 0;

	pr_debug("current_ma = %d , term_ma = %d\n", current_ma, term_ma);
	for (i = ARRAY_SIZE(charge_curr_limit) - 1; i > 0; i--) {
		if (charge_curr_limit[i] <= current_ma)
			break;
	}
	temp = i << 4;

	
	for (i = 0; i < ARRAY_SIZE(term_curr_limit) - 1; i++) {
		if (term_curr_limit[i] >= term_ma)
			break;
	}
	

	val = temp | i ;

	return fan5405_write(chip, FAN5405_IBAT, val);
}


#if 0
static int fan5405_reset_reg(struct fan5405_chip *chip)
{
	return fan5405_masked_write(chip, FAN5405_IBAT,
				RESET_MASK, RESET_MASK);
}
#endif

static int fan5405_set_dis_verg(struct fan5405_chip *chip, bool on)
{
	u8 temp = 0;
	if (!on) {
		temp = DIS_VREG_MASK;
	}
	
	pr_debug("Writing 0x%02x dis_verg = 0x%02x\n", FAN5405_SP_CHARGER, temp);
	return fan5405_masked_write(chip, FAN5405_SP_CHARGER,
				DIS_VREG_MASK, temp);
}

static int fan5405_set_io_level(struct fan5405_chip *chip, bool level)
{
	u8 temp = 0;
	
	if (level) {
		temp = IO_LEVEL;
	}

	pr_debug("Writing 0x%02x io = 0x%02x\n", FAN5405_SP_CHARGER, temp);
	return fan5405_masked_write(chip, FAN5405_SP_CHARGER,
				IO_LEVEL, temp);
}

#define SPECIAL_CHG_INPUT_VOL_MIN      4213
#define SPECIAL_CHG_INPUT_VOL_MAX      4773
#define SPECIAL_CHG_INPUT_VOL_STEP      80
static const int special_chg_input_vol[] = {4213, 4293, 4373, 4453, 4533, 4613, 4693, 4773,};
static int fan5405_set_vsp_vol(struct fan5405_chip *chip, int vol_mv)
{
	u8 i = 0;
	for (i = ARRAY_SIZE(special_chg_input_vol) - 1; i >= 0; i--) {
		if (special_chg_input_vol[i] <= vol_mv)
			break;
	}
	
	pr_debug("Writing 0x%02x (2-0bit) = 0x%02x\n", FAN5405_SP_CHARGER, i);
	return fan5405_masked_write(chip, FAN5405_SP_CHARGER,
				VSP_MASK, i);
}

/* when Rsense = 68 */
static const int isafe_limit[] = {550, 650, 750, 850, 950, 1050, 1150, 1250,};
#define VSAFE_MIN      4200
#define VSAFE_MAX      4440
#define VSAFE_step      20

static int  fan5405_set_safe_reg(struct fan5405_chip *chip, int current_ma, int vol_mv)
{
	int safe_vol = 0;
	int rc = 0;
	u8 temp =0;
	u8 temp_curr = 0;
	u8 temp_vol = 0;
	u8 reg_val = 0;
	u8 i = 0;

	if (vol_mv < VSAFE_MIN)
		safe_vol = VSAFE_MIN;
	else if (vol_mv > VSAFE_MAX)
		safe_vol = VSAFE_MAX;
	else
		safe_vol = vol_mv;
	
	temp_vol = (safe_vol - VSAFE_MIN) / VSAFE_step;

	
	for (i = ARRAY_SIZE(isafe_limit) - 1; i >= 0; i--) {
		if (isafe_limit[i] <= current_ma)
			break;
	}
	temp_curr = i << 4;

	temp = temp_curr | temp_vol;

	/* In the first time, we must to write the safety register, and ensure the vuale is right */
	for (i = 0; i < 10; i++) {
		rc = fan5405_write(chip, FAN5405_SAFETY, temp);
		if (rc < 0) {
			dev_err(chip->dev,"fail to set  safety register, rc = %d\n", rc);
		}
		rc = fan5405_read(chip, FAN5405_SAFETY, &reg_val);
		if (rc < 0) {
			dev_err(chip->dev,"fail to read safety register, rc = %d\n", rc);
		}
		
		if (reg_val == temp)
			break;
		
		udelay(100);
	}
	
	pr_info("0x%02x  Writing  0x%02x :  Reading 0x%02x\n", FAN5405_SAFETY, temp, reg_val);
	
	return rc;
	
}

enum fan5405_stat {
	FAN5405_STAT_READY = 0,
	FAN5405_STAT_CHARGING,
	FAN5405_STAT_DONE,
	FAN5405_STAT_FAULT,	
};
static int fan5405_get_stat(struct fan5405_chip *chip)
{
	u8 reg_val = 0;
	u8 mask = 0;
	int rc = 0;

	mask = 0x30;
	rc = fan5405_read(chip, FAN5405_CONTROL0, &reg_val);
	if (rc < 0) {
		dev_err(chip->dev,"fail to read safety register, rc = %d\n", rc);
	}
	reg_val = reg_val & mask;
	pr_debug("register 0x00 = 0x%02x \n",reg_val);
	return (reg_val >> 4);
	
}

static void fan5405_force_en_charging(struct fan5405_chip *chip, bool enable)
{
	if (enable)
		gpio_set_value(chip->en_gpio, 0);
	else
		gpio_set_value(chip->en_gpio, 1);
	
}

static int fan5405_dump_registers(struct fan5405_chip *chip)
{
	int rc = 0;
	u8 reg_val = 0;
	u8 i = 0;

	for (i = 0; i <= 6; i++) {
		rc = fan5405_read(chip, i, &reg_val);
		if (rc < 0) {
			dev_err(chip->dev,"fail to read register : 0x%02x, rc = %d\n", i, rc);
		}
		pr_debug("register 0x%02x = 0x%02x \n", i, reg_val);
	}
	
	rc = fan5405_read(chip, FAN5405_MONITOR, &reg_val);
	if (rc < 0) {
		dev_err(chip->dev,"fail to read register : 0x%02x, rc = %d\n", FAN5405_MONITOR, rc);
	}
	pr_debug("register 0x%02x = 0x%02x \n", FAN5405_MONITOR, reg_val);

	return rc;
}

/* set registers value */
static int fan5405_set_chg_reg(struct fan5405_chip *chip)
{
	int rc = 0;
#if 0
	//1.update T32 timer
	rc = fan5405_32s_tmr_rst(chip);
	if (rc < 0) {
			dev_err(chip->dev,"fail to update T32 timer, rc = %d\n", rc);
			//return rc;
	}

	//2. enable STAT pin LOW when IC is charging
	rc = fan5405_enable_stat(chip, true);
	if (rc < 0) {
			dev_err(chip->dev,"fail to enable STAT, rc = %d\n", rc);
			//return rc;
	}
#endif
	//1. 1SET  REG 0x00, update T32 timer and enable STAT pin LOW when IC is charging 
	rc = fan5405_masked_write(chip, FAN5405_CONTROL0,
				TMR_RST_MASK |EN_STAT_MASK, TMR_RST_MASK | EN_STAT_MASK);

	//1. 2 SET REG 0x01
	rc = fan5405_set_ivbus_max(chip, chip->set_ivbus_max); //VBUS CURRENT

	
	if (!chip->weak_batt_vol)
		chip->weak_batt_vol = 3400;
	rc = fan5405_set_weak_batt_vol(chip, chip->weak_batt_vol);

	rc = fan5405_enable_te(chip, true);
	rc = fan5405_enable_chg(chip, true);
	rc = fan5405_enable_HZ_mode(chip, false);
	rc = fan5405_enable_OPA_mode(chip, false);

	//1. 3 SET REG 0x02
	if (!chip->chg_vol_max)
		chip->chg_vol_max = 4200;
	rc = fan5405_set_chg_vol_max(chip, chip->chg_vol_max);
	rc = fan5405_set_otg_pl(chip, false);
	rc = fan5405_enable_otg(chip, false);
	
	//1. 4 SET REG 0x04

	if (!chip->chg_curr_now)
		chip->chg_curr_now = 1000;

	if (!chip->iterm_ma)
		chip->iterm_ma = 100;
	
	rc = fan5405_set_iterm_ichg(chip, chip->chg_curr_now, chip->iterm_ma);
	if (rc < 0) {
			dev_err(chip->dev,"fail to set 0x04 register value, rc = %d\n", rc);
			//return rc;
	}

	//1. 5 SET REG 0x05
	rc = fan5405_set_dis_verg(chip, true);
	rc = fan5405_set_io_level(chip, false);
	if (!chip->vsp_vol)
		chip->vsp_vol = 4500;
	rc = fan5405_set_vsp_vol(chip, chip->vsp_vol);

	return rc;
}

static int fan5405_batt_volt_to_capa(int volt)
{
	int crude_capa = 88;
	int loop_max_size = 0;
	int loop = 0;
	loop_max_size = sizeof(batt_capa_data)/sizeof(struct fan5405_batt_capa_data) - 1;
	
	for (loop = 0; loop < loop_max_size; loop ++) {
		if (volt >= batt_capa_data[loop].volt)
			break;
	}
	
	
	if (loop == loop_max_size || loop == 0)
		crude_capa = batt_capa_data[loop].capa;
	else {
		pr_debug("loop caps = %d, loop-1 capa = %d,loop-1.volt = %d \n", batt_capa_data[loop].capa, 
				batt_capa_data[loop-1].capa, batt_capa_data[loop-1].volt);
		crude_capa = batt_capa_data[loop - 1].capa - ((batt_capa_data[loop - 1].capa - batt_capa_data[loop].capa) * 
				(batt_capa_data[loop - 1].volt - volt)/(batt_capa_data[loop - 1].volt - batt_capa_data[loop].volt));
	}	
	
	return crude_capa;	
}

#define LOWER      (0)
#define HIGER      (98)
#define CALIBRATE_CAPA(x)      ((100)*(x - LOWER))/(HIGER-LOWER)
//#define CALIBRATE_CAPA(x)      ((100+HIGER)*((x)-LOWER)/(100-LOWER))    ;;LOWER = 2 HIGER =2

static long long get_wall_time(void)
{
	struct timespec     wall_time;
	getnstimeofday(&wall_time);
	return wall_time.tv_sec;
}

static int fan5405_get_batt_ave_mvolt(struct fan5405_chip *chip)
{
	int loop_num = 0;
	int mvolt = 0;
	int mvolt_min = 0;
	int mvolt_max = 0;
	int mvolt_sum = 0;

	mvolt = get_yl_pm8916_batt_mvol();
	mvolt_min = mvolt;
	mvolt_max = mvolt;
	mvolt_sum += mvolt;
	for (loop_num = 0; loop_num < 5; loop_num++) {
		mvolt = get_yl_pm8916_batt_mvol();
		mvolt_sum += mvolt;
		if (mvolt_max < mvolt)
			mvolt_max = mvolt;
		if (mvolt_min > mvolt)
			mvolt_min = mvolt;	
	}
	mvolt = (mvolt_sum - mvolt_min - mvolt_max) / 4;
	chip->batt_crude_volt = mvolt;
	pr_err("batt_volt_mv = %d, yl_get_bk_level = %d, yl_get_lcd_status = %d\n", chip->batt_crude_volt, yl_get_bk_level, yl_get_lcd_status);
	pr_debug("battery average voltage : %d mvolt sum  = %d , mvolt max = %d , mvolt min = %d \n", mvolt, mvolt_sum, mvolt_max, mvolt_min);
	return mvolt;	
}

#define LCD_OPEN_BATT_MVOLT_DELTA      50
#define LCD_CLOSE_BATT_MVOLT_DELTA      30
#define BK_LEVEL_MVOLT_OFFSET(x)      (x/5)
static int fan5405_evaluate_batt_mvolt(struct fan5405_chip *chip)
{
	int evaluate_mvolt = 0;
	int mvolt = 0;
	int batt_mvolt_offset = 0;
	
	if (FAN5405_STAT_CHARGING != chip->charge_stat) {
		mvolt = fan5405_get_batt_ave_mvolt(chip);
	} else { 
		fan5405_force_en_charging(chip, false);
		//msleep(10);
		mvolt = fan5405_get_batt_ave_mvolt(chip);
		fan5405_force_en_charging(chip, !chip->charging_disabled);
	}

	if (yl_get_lcd_status) {
	/* LCD display is open */
		batt_mvolt_offset = LCD_OPEN_BATT_MVOLT_DELTA + BK_LEVEL_MVOLT_OFFSET(yl_get_bk_level);
	} else {
	/* LCD display is close */
		//batt_mvolt_offset = chip->dischg_batt_mvolt_delta;
		batt_mvolt_offset = LCD_CLOSE_BATT_MVOLT_DELTA;
	}

	evaluate_mvolt = mvolt + batt_mvolt_offset;
	pr_info("evaluate_mvolt = %d, batt_mvolt_offset = %d, BK_LEVEL_MVOLT_OFFSET(yl_get_bk_level) = %d\n", evaluate_mvolt, batt_mvolt_offset, BK_LEVEL_MVOLT_OFFSET(yl_get_bk_level));
	return evaluate_mvolt;	  
}

#define BATT_CUTOFF_MVOLT       2600
#define SHUTDOWN_V_PON_SOC_STEP       10
static int initialized_to_sys_capa(struct fan5405_chip *chip)
{
	int read_ocv;
	int read_soc;
	int calculate_soc;
	int rc;

	rc = yl_read_shutdown_ocv_soc(&read_ocv, &read_soc);
	if(rc < 0)
		chip->shutdown_soc_invalid = true;

	chip->shutdown_soc = read_soc;
	calculate_soc = chip->to_sys_capa;

	if(chip->warm_reset){
		if(!chip->shutdown_soc_invalid){
			/* pon soc > shutdown soc + 10 and usb is present (is charging) */
			if (((calculate_soc - read_soc) > SHUTDOWN_V_PON_SOC_STEP) && (true == chip->vbus_present)) {
				chip->to_sys_capa = read_soc + 1;
				/* shutdown soc > pon soc +10 and usb is absent (is discharging) */
			} else if (((read_soc - calculate_soc) > SHUTDOWN_V_PON_SOC_STEP) && (false == chip->vbus_present)) {
				chip->to_sys_capa = read_soc - 1;
			} else {
				chip->to_sys_capa = read_soc;
			}
			chip->batt_volt = read_ocv;
			pr_err("warm_reset: using shutdown SOC\n");
		} else {
			pr_err("using PON SOC\n");
		}
	} else {
		if(!chip->shutdown_soc_invalid &&
				(abs(read_soc - calculate_soc) < SHUTDOWN_V_PON_SOC_STEP)){
			chip->to_sys_capa = read_soc;
			chip->batt_volt = read_ocv;
			pr_err("using shutdown SOC\n");
		} else {
			pr_err("using PON SOC\n");
		}
	}

	pr_info("shutdown_soc_invalid = %d read_ocv = %d read_soc = %d calculate_soc = %d\n", 
			chip->shutdown_soc_invalid, read_ocv, read_soc, calculate_soc);


	return chip->to_sys_capa;
}

#define SOC_TOO_DIFF        20
#define EXPIRE_TIME         120 /* 120 secs */
#define SOC_LOW_EXPIRE_TIME         60 /* 60 secs */
#define BATT_CRUDE_SOC_LOW      20
#define BATT_MVOLT_TO_LOW        3100 /* 3100MV */
#define BATT_MVOLT_DEFAULT       3800 /* 3800MV */
static int calculated_to_sys_capa(struct fan5405_chip *chip)
{
	static int first_time = 0;
	static long long last_update_tm_sec = 0;
	static int charging_need_to_decrease = 0;
	long long the_past_time = 0;
	long long now_tm_sec = 0;
	long long expire_time_sec = SOC_LOW_EXPIRE_TIME;
	int time_to_soc = 0;
	int soc_temp = 0;

	chip->vbus_present = (1 == get_yl_pm8916_vbus_status());
	if ((BATT_CRUDE_SOC_LOW < chip->batt_crude_capa) && (chip->vbus_present == false))
		expire_time_sec = EXPIRE_TIME;
	
	chip->batt_volt  = fan5405_evaluate_batt_mvolt(chip);
	if (chip->batt_volt < 0) {
		pr_err("failt fo read battery voltage : %d \n", chip->batt_volt);
		return 50;
	}

	now_tm_sec = get_wall_time();
	if (first_time == 0) {
		first_time = 1;
		last_update_tm_sec = now_tm_sec + expire_time_sec;
		chip->batt_crude_capa = initialized_to_sys_capa(chip);
		if (SOC_UNINITIALIZED == chip->to_sys_capa) {
			chip->to_sys_capa = fan5405_batt_volt_to_capa(chip->batt_volt);
			chip->batt_crude_capa = chip->to_sys_capa;
		}
	} else {
		if (3 > first_time) {
			first_time++;
			last_update_tm_sec = now_tm_sec + expire_time_sec;
		}
		chip->batt_crude_capa = fan5405_batt_volt_to_capa(chip->batt_volt);
	}

	//pr_info("before to_sys_capa = %d, batt_crude_capa = %d\n", chip->to_sys_capa, chip->batt_crude_capa);
	if (now_tm_sec > last_update_tm_sec ) {
		the_past_time = now_tm_sec -last_update_tm_sec + expire_time_sec;
		last_update_tm_sec = now_tm_sec + expire_time_sec; 
		
		
		if (chip->vbus_present == false){
		/* charger unplugged */
			time_to_soc = chip->to_sys_capa - ((int)the_past_time/(int)expire_time_sec);
			soc_temp = max(time_to_soc, chip->batt_crude_capa);

			if (chip->to_sys_capa > soc_temp) {
				chip->to_sys_capa = soc_temp;
				if ((chip->to_sys_capa - chip->batt_crude_capa) > SOC_TOO_DIFF)
					chip->to_sys_capa--;
			}
		
		} else {
		/* charger plugged */
			time_to_soc = chip->to_sys_capa + ((int)the_past_time/(int)expire_time_sec);
			soc_temp = min(time_to_soc, chip->batt_crude_capa);

			if (chip->to_sys_capa > soc_temp) {
				++charging_need_to_decrease;
				if (charging_need_to_decrease > 3)
					chip->to_sys_capa--;
	
				if ((chip->to_sys_capa - chip->batt_crude_capa) > SOC_TOO_DIFF)
					chip->to_sys_capa--;
				
			} else if(chip->to_sys_capa < soc_temp){
				charging_need_to_decrease = 0;
				chip->to_sys_capa = soc_temp;
				if ((chip->batt_crude_capa - chip->to_sys_capa) > SOC_TOO_DIFF)
					chip->to_sys_capa++;
			}

			if (FAN5405_STAT_DONE == chip->charge_stat && (100 > chip->to_sys_capa))
				chip->to_sys_capa++;

		}
	} else if (now_tm_sec < last_update_tm_sec - expire_time_sec) {
		last_update_tm_sec = now_tm_sec + expire_time_sec; 
	}

	if (BATT_MVOLT_TO_LOW > chip->batt_crude_volt) {
		chip->batt_crude_volt = fan5405_get_batt_ave_mvolt(chip);
		if (BATT_MVOLT_TO_LOW > chip->batt_crude_volt) 
			chip->to_sys_capa = 0;		
	}
	
	if (chip->last_to_sys_capa != chip->to_sys_capa) {
		yl_backup_ocv_soc(chip->batt_volt, chip->to_sys_capa);
		chip->last_to_sys_capa = chip->to_sys_capa;
		pr_info("backup_soc chip->last_to_sys_capa = %d, chip->to_sys_capa = %d\n", chip->last_to_sys_capa, chip->to_sys_capa);
	}
	pr_debug("now_tm_sec = %lld, last_update_tm_sec = %lld, the_past_time = %lld\n", now_tm_sec, last_update_tm_sec, the_past_time);
	pr_info("after last_to_sys_capa = %d, to_sys_capa = %d, batt_crude_capa = %d, the_past_time = %lld\n", 
			chip->last_to_sys_capa,chip->to_sys_capa, chip->batt_crude_capa, the_past_time);
	return chip->to_sys_capa;
	
}
#if 0
static int calculated_to_sys_capa(struct fan5405_chip *chip)
{
	static int first_time = 0;
	long long now_tm_sec = 0;
	static long long last_update_tm_sec = 0;
	
	chip->batt_volt  = fan5405_evaluate_batt_mvolt(chip);
	if (chip->batt_volt < 0) {
		pr_err("failt fo read battery voltage : %d \n", chip->batt_volt);
		return 50;
	}
	chip->vbus_present = (1 == get_yl_pm8916_vbus_status());
	now_tm_sec = get_wall_time();
	if (first_time == 0) {
		first_time = 1;
		last_update_tm_sec = now_tm_sec + EXPIRE_TIME; /*test*/
		chip->batt_crude_capa = initialized_to_sys_capa(chip);
	} else {
		chip->batt_crude_capa = fan5405_batt_volt_to_capa(chip->batt_volt);
	}

	pr_info("before to_sys_capa = %d, batt_crude_capa = %d\n", chip->to_sys_capa, chip->batt_crude_capa);
	if (now_tm_sec > last_update_tm_sec ) {
		last_update_tm_sec = now_tm_sec + EXPIRE_TIME; 

		if (chip->vbus_present == false){
		/* charger unplugged */
			pr_debug("charger unplugged \n");
			if (chip->to_sys_capa > chip->batt_crude_capa) {
				chip->to_sys_capa--;
				if ((chip->to_sys_capa - chip->batt_crude_capa) > SOC_TOO_DIFF)
					chip->to_sys_capa--;
			}
		} else {
		/* charger plugged */
			pr_debug("charger plugged \n");
			if (chip->to_sys_capa > chip->batt_crude_capa) {
				chip->to_sys_capa--;
				if ((chip->to_sys_capa - chip->batt_crude_capa) > SOC_TOO_DIFF)
					chip->to_sys_capa--;
			} else if(chip->to_sys_capa < chip->batt_crude_capa){
				chip->to_sys_capa++;
				if ((chip->batt_crude_capa - chip->to_sys_capa) > SOC_TOO_DIFF)
					chip->to_sys_capa++;
			}
		}
	}
	
	
	if (chip->last_to_sys_capa != chip->to_sys_capa) {
		yl_backup_ocv_soc(chip->batt_volt, chip->to_sys_capa);
		chip->last_to_sys_capa = chip->to_sys_capa;
		pr_err("########backup_soc chip->last_to_sys_capa = %d, chip->to_sys_capa = %d\n", chip->last_to_sys_capa, chip->to_sys_capa);
	}
	pr_debug("now_tm_sec = %lld, last_update_tm_sec = %lld\n", now_tm_sec, last_update_tm_sec);
	pr_debug("after last_to_sys_capa = %d, to_sys_capa = %d, batt_crude_capa = %d\n", chip->last_to_sys_capa,chip->to_sys_capa, chip->batt_crude_capa);
	return chip->to_sys_capa;
	
}
#endif

#define UPDATE_T32S_PERIOD_MS      10000
static void fan5405_update_t32s_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct fan5405_chip *chip = container_of(dwork,
				struct fan5405_chip, update_t32s_work);
	int rc;

	pm_stay_awake(chip->dev);

	mutex_lock(&chip->r_completed_lock);
	chip->update_t32s_waiting = true;
	if (!chip->resume_completed) {
		dev_dbg(chip->dev, "fan5405_update_t32s_work before device-resume\n");
		mutex_unlock(&chip->r_completed_lock);
		goto stop_update;
	}
	chip->update_t32s_waiting = false;
	mutex_unlock(&chip->r_completed_lock);
	
	chip->vbus_present = (1 == get_yl_pm8916_vbus_status());
	rc = fan5405_set_chg_reg(chip);
	fan5405_dump_registers(chip);
	pr_debug(" %s: charge_stat = %d, vbus_present = %d, chip->charging_disabled = %d \n", __func__, 
			chip->charge_stat, chip->vbus_present, chip->charging_disabled);
	//if ((FAN5405_STAT_DONE == chip->charge_stat) || (!chip->vbus_present) || chip->charging_disabled)
	if (!chip->vbus_present)
		goto stop_update;
	
//update_again_later:
	schedule_delayed_work(&chip->update_t32s_work,
		msecs_to_jiffies(UPDATE_T32S_PERIOD_MS));
	return;

stop_update:

	chip->vbus_present = (1 == get_yl_pm8916_vbus_status());
	if (false == chip->vbus_present){
		pr_info("vbus absent notify USB==== %s\n", __func__);
		power_supply_set_present(chip->usb_psy, chip->vbus_present);
	}	
	pr_debug("=======exit ==== %s\n", __func__);
	pm_relax(chip->dev);
}

#define BATT_CAPA_RECHG      97
static int check_fan5405_need_to_rechg(struct fan5405_chip *chip)
{

	pr_debug(" %s: charge_stat = %d, vbus_present = %d, chip->charging_disabled = %d \n", __func__, 
			chip->charge_stat, chip->vbus_present, chip->charging_disabled);
	if ((chip->vbus_present) && (BATT_CAPA_RECHG >= chip->to_sys_capa) 
		&& (FAN5405_STAT_CHARGING != chip->charge_stat)) {
		if (!chip->charging_disabled) { 	
			fan5405_force_en_charging(chip, chip->charging_disabled);
			msleep(500);
			fan5405_force_en_charging(chip, !chip->charging_disabled);
			cancel_delayed_work_sync(&chip->update_t32s_work);
			fan5405_update_t32s_work(&chip->update_t32s_work.work);
		}
	}

	return 0;
}

static int get_batt_temp_status(struct fan5405_chip *chip)
{
	enum batt_temp_status b_status;
	int batt_temp;

	batt_temp = chip->batt_temp;

	if (batt_temp < BATT_TEMP_TOO_COLD_THRESHOLD)  /* battery temp < 0 celsius*/
		b_status = BATT_TEMP_TOO_COLD;
	else if (batt_temp < BATT_TEMP_COLD_THRESHOLD)  /* 0 celsius <= battery temp < 10 celsius */
		b_status = BATT_TEMP_COLD;
	else if (batt_temp < BATT_TEMP_COOL_THRESHOLD)  /* 10 celsius <= battery temp < 20 celsius */
		b_status = BATT_TEMP_COOL;
	else if (batt_temp < BATT_TEMP_NUM_THRESHOLD)   /* 20 celsius <= battery temp <= 45 celsius */
		b_status = BATT_TEMP_NUM;
	else if (batt_temp < BATT_TEMP_HOT_THRESHOLD)    /* 45 celsius < battery temp < 60 celsius */
		b_status = BATT_TEMP_HOT;
	else 
		b_status = BATT_TEMP_TOO_HOT;     /* 60 celsius <= battery temp */
	
	pr_debug("batt_temp =%d, b_status = %d\n", batt_temp, b_status);
	return b_status;
}


#define STEP_CHG_THRESHOLD_VOLT_MV        4000
#define STEP_CHG_DELTA_VOLT_MV                  20
static int fan5405_temp_appropriate_charging(struct fan5405_chip *chip)
{
	static int last_temp_sts = BATT_TEMP_STATUS_UNKOWN;
	static bool last_volt_sts = false;
	static bool batt_volt_low = false;
	static bool batt_too_cold = false;
	static bool batt_too_hot = false;
	
	int batt_volt_mv = 0;
	/**/
	int chg_current = 0;	
	

	chip->batt_temp_status = get_batt_temp_status(chip);//set buffer
	batt_volt_mv = chip->batt_crude_volt;
	//batt_volt_mv = get_prop_battery_voltage_now(chip);


	if (batt_volt_mv < (STEP_CHG_THRESHOLD_VOLT_MV -STEP_CHG_DELTA_VOLT_MV))
		batt_volt_low = true;
	else if (batt_volt_mv > (STEP_CHG_THRESHOLD_VOLT_MV +STEP_CHG_DELTA_VOLT_MV))
		batt_volt_low = false;

	/* battery tempereture status has been changed 
	  * or battery voltage leap over 4000mv */
	if ((last_temp_sts != chip->batt_temp_status) || (last_volt_sts != batt_volt_low)) {
		last_temp_sts = chip->batt_temp_status;
		last_volt_sts = batt_volt_low;
		chip->batt_too_cold = false;
		chip->batt_too_hot = false;
		switch (last_temp_sts) {
			case BATT_TEMP_TOO_COLD:
				chip->batt_too_cold = true;
				break;
			case BATT_TEMP_COLD:
				chg_current = CHG_CURR_LIMIT_COLD;
				break;
			case BATT_TEMP_COOL:
				if (last_volt_sts)
					chg_current = CHG_CURR_LIMIT_COOL_4P0V;
				else
					chg_current = CHG_CURR_LIMIT_COOL_4P35V;
				break;
			case BATT_TEMP_NUM:
				chg_current = CHG_CURR_LIMIT_NUM;
				break;
			case BATT_TEMP_HOT:
			case BATT_TEMP_TOO_HOT:
				chip->batt_too_hot = true;
				break;
			default:
				break;
		}

		chip->chg_curr_now  = min(chg_current, 	chip->chg_curr_max);
		pr_err("setting %d mA, %d, temp = %d, batt_temp_status = %d\n",chip->chg_curr_now, 
				chg_current, chip->batt_temp, chip->batt_temp_status);
		fan5405_set_iterm_ichg(chip, chip->chg_curr_now, chip->iterm_ma);


		if ((batt_too_cold != chip->batt_too_cold) || (batt_too_hot != chip->batt_too_hot)) {
			batt_too_cold = chip->batt_too_cold;
			batt_too_hot = chip->batt_too_hot;
			if (batt_too_hot ||batt_too_cold ) {
				/* end of charging */
				chip->charging_disabled = true;
			} else {
				chip->charging_disabled = false;
			}
			
			pr_err("batt_too_cold =  %d, batt_too_hot = %d, charging_disabled = %d \n", 
				batt_too_cold, batt_too_hot, chip->charging_disabled);
			
			fan5405_force_en_charging(chip, !chip->charging_disabled);
		}
	}


	return 0;
}

#define UPDATE_HEART_PERIOD_FAST_MS      21000
#define UPDATE_HEART_PERIOD_NORMAL_MS      61000
#define BATT_CAPA_LOW_LEVEL      15
static void fan5405_update_heartbeat_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct fan5405_chip *chip = container_of(dwork,
				struct fan5405_chip, update_heartbeat_work);
	int update_period = UPDATE_HEART_PERIOD_NORMAL_MS;

	mutex_lock(&chip->r_completed_lock);
	chip->update_heartbeat_waiting = true;
	if (!chip->resume_completed) {
		dev_dbg(chip->dev, "fan5405_update_heartbeat_work before device-resume\n");
		mutex_unlock(&chip->r_completed_lock);
		return ;
	}
	chip->update_heartbeat_waiting = false;
	mutex_unlock(&chip->r_completed_lock);

	//read_yl_pm8916_batt_id();
	chip->vbus_present = (1 == get_yl_pm8916_vbus_status());
	chip->charge_stat = fan5405_get_stat(chip);
	chip->batt_temp = get_yl_pm8916_batt_temp();
	
	fan5405_temp_appropriate_charging(chip);
	calculated_to_sys_capa(chip);
	check_fan5405_need_to_rechg(chip);
	
	power_supply_changed(&chip->batt_psy);
	
	if(chip->batt_crude_capa < BATT_CAPA_LOW_LEVEL)
		update_period = UPDATE_HEART_PERIOD_FAST_MS;
	schedule_delayed_work(&chip->update_heartbeat_work,
		msecs_to_jiffies(update_period));

	pr_info(" %s: charge_stat = %d, vbus_present = %d, to_sys_capa = %d, batt_crude_volt = %d\n", __func__, 
			chip->charge_stat, chip->vbus_present, chip->to_sys_capa, chip->batt_crude_volt);
	return;
}


/* 30minute */
#define G_WAKEUP_INTERVAL        30*60

static void batt_info_alarm_set(struct fan5405_chip *chip, int seconds)
{
	ktime_t interval = ktime_set(seconds, 0);

	pr_debug("batt_info_alarm_set has been setting. \n");
	
	alarm_start_relative(&chip->report_batt_info_alarm, interval);
}

static void batt_info_alarm_work(struct work_struct *work)
{
	struct fan5405_chip *chip = container_of(work, struct fan5405_chip, batt_info_alarm_work);
	if (!this_chip) {
		pr_err("chip not yet initalized\n");
	}
	
	pr_debug("enter %s . \n", __func__);

	cancel_delayed_work_sync(&chip->update_heartbeat_work);
	fan5405_update_heartbeat_work(&chip->update_heartbeat_work.work);
	//power_supply_changed(&chip->batt_psy); 
	
	if (BATT_CAPA_LOW_LEVEL < chip->batt_crude_capa )
		batt_info_alarm_set(chip, 2*G_WAKEUP_INTERVAL);
	else
		batt_info_alarm_set(chip, G_WAKEUP_INTERVAL);
	
	wake_unlock(&chip->batt_info_alarm_wlock);

}

static enum alarmtimer_restart batt_info_alarm_callback(struct alarm *alarm, ktime_t now)
{
	struct fan5405_chip *chip = container_of(alarm, struct fan5405_chip, report_batt_info_alarm);

	pr_debug("battery alarm is coming\n");
	wake_lock(&chip->batt_info_alarm_wlock);
	schedule_work(&chip->batt_info_alarm_work);
	
	return ALARMTIMER_NORESTART;
}


static irqreturn_t fan5405_stat_irq_handler(int irq, void *_chip)
{
//	struct fan5405_chip *chip = _chip;
	
	pr_debug("fan5405 stat irq handler \n");
	

	return IRQ_HANDLED;
}

/* power supply segment */
static enum power_supply_property fan5405_battery_properties[] = {
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
};


static char *fan5405_batt_supplied_to[] = {
	"bms",
};

static int fan5405_get_prop_batt_health(struct fan5405_chip *chip)
{
	union power_supply_propval ret = {0, };

	if (chip->batt_too_hot)
		ret.intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (chip->batt_too_cold)
		ret.intval = POWER_SUPPLY_HEALTH_COLD;
	else
		ret.intval = POWER_SUPPLY_HEALTH_GOOD;

	return ret.intval;
}

static int fan5405_get_prop_batt_status(struct fan5405_chip *chip)
{
	union power_supply_propval ret = {0, };

	chip->vbus_present = (1 == get_yl_pm8916_vbus_status());
	if (true == chip->vbus_present) {
		if (chip->report_soc >= 100)
			ret.intval = POWER_SUPPLY_STATUS_FULL;
		else
			ret.intval = POWER_SUPPLY_STATUS_CHARGING;	
	} else {
		ret.intval = POWER_SUPPLY_STATUS_DISCHARGING;
	}
#if 0	
	chip->charge_stat = fan5405_get_stat(chip);

	if (FAN5405_STAT_DONE == chip->charge_stat)
		ret.intval = POWER_SUPPLY_STATUS_FULL;
	else if (FAN5405_STAT_CHARGING == chip->charge_stat)
		ret.intval = POWER_SUPPLY_STATUS_CHARGING;
	else
		ret.intval = POWER_SUPPLY_STATUS_DISCHARGING;
#endif
	return ret.intval;
}

static int fan5405_get_prop_batt_present(struct fan5405_chip *chip)
{
	chip->batt_present = is_battery_present();

//	pr_err("battery present = %d, \n", chip->batt_present);
	return chip->batt_present;
}

static int fan5405_get_prop_charging_enable(struct fan5405_chip *chip)
{
	union power_supply_propval ret = {0, };


	ret.intval = !(chip->charging_disabled);

	return ret.intval;
}

static int fan5405_get_prop_charge_type(struct fan5405_chip *chip)
{
	union power_supply_propval ret = {0, };


	ret.intval = POWER_SUPPLY_CHARGE_TYPE_FAST;

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static int fan5405_get_prop_charge_full_design(struct fan5405_chip *chip)
{
	union power_supply_propval ret = {0, };


	ret.intval = 2500;
	return ret.intval;

}

static int fan5405_get_prop_voltage_max_design(struct fan5405_chip *chip)
{
	union power_supply_propval ret = {0, };


	ret.intval = 4350000;

	return ret.intval;
}

static int fan5405_get_prop_voltage_now(struct fan5405_chip *chip)
{
	int mvol;
       int data = 0;
	int rc = 0;
	union power_supply_propval ret = {0, };
	/*external_fuel_gauge*/
	rc = lc709203_get_batt_property(POWER_SUPPLY_PROP_VOLTAGE_NOW, &data);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					data, rc);
		return rc;
	}
	mvol = data;
	ret.intval = 1000 * get_yl_pm8916_batt_mvol();
       pr_err("current votage = %d,voltage adc sample =%d\n", mvol,ret.intval);
        return mvol;
}


static int fan5405_get_prop_batt_temp(struct fan5405_chip *chip)
{
	union power_supply_propval ret = {0, };


	ret.intval = get_yl_pm8916_batt_temp();

	return ret.intval;
}


static int fan5405_get_prop_batt_capa(struct fan5405_chip *chip)
{
     #ifdef CONFIG_BATTERY_LC709203
	 int percent_soc;
        int data = 0;
	int rc = 0;
	/*external_fuel_gauge*/
	rc = lc709203_get_batt_property(POWER_SUPPLY_PROP_CAPACITY, &data);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					data, rc);
		return rc;
	}
	percent_soc = data;
        //return percent_soc;
        printk("current percent = %d,batt_crude_capa=%d\n", percent_soc,chip->batt_crude_capa);
        return chip->to_sys_capa;
	 
     #else
	int batt_capa;
	
	if (SOC_UNINITIALIZED == chip->to_sys_capa) {
		pr_err("battery capacity not yet initialized!\n");
		return 88;
	}
	batt_capa = CALIBRATE_CAPA(chip->to_sys_capa);
	if ( batt_capa > 100 )
		batt_capa = 100;
	if ( batt_capa < 0 )
		batt_capa = 0;

	chip->report_soc = batt_capa;	
	pr_debug("report_capa ret.batt_capa = %d \n", batt_capa);
	return batt_capa;
#endif
}

static int fan5405_battery_get_property(struct power_supply *psy, 
										enum power_supply_property prop,
										union power_supply_propval *val)
{
	struct fan5405_chip *chip = container_of(psy, struct fan5405_chip, batt_psy);
	
	switch (prop) {
		case POWER_SUPPLY_PROP_HEALTH:
			val->intval = fan5405_get_prop_batt_health(chip);
			break;
		case POWER_SUPPLY_PROP_STATUS:
			val->intval = fan5405_get_prop_batt_status(chip);
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = fan5405_get_prop_batt_present(chip);
			break;	
		case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = fan5405_get_prop_batt_capa(chip);
			break;
		case POWER_SUPPLY_PROP_CHARGING_ENABLED:
			val->intval = fan5405_get_prop_charging_enable(chip);
			break;
		case POWER_SUPPLY_PROP_CHARGE_TYPE:
			val->intval = fan5405_get_prop_charge_type(chip);
			break;
		case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
			val->intval = fan5405_get_prop_charge_full_design(chip);
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
			val->intval = fan5405_get_prop_voltage_max_design(chip);
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval = fan5405_get_prop_voltage_now(chip);
			break;
		case POWER_SUPPLY_PROP_TEMP:
			val->intval = fan5405_get_prop_batt_temp(chip);
			break;
		default:
			return -EINVAL;
	}
	
	return 0;
}
								
static int fan5405_battery_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

static int fan5405_battery_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	struct fan5405_chip *chip = container_of(psy,
				struct fan5405_chip, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		chip->charging_disabled = !(val->intval); 	
		fan5405_force_en_charging(chip, !chip->charging_disabled);
		dev_err(chip->dev,"chip->en_gpio value = %d  disabled = %d \n", gpio_get_value(chip->en_gpio), chip->charging_disabled);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void fan5405_external_power_changed(struct power_supply *psy)
{
	struct fan5405_chip *chip = container_of(psy,
				struct fan5405_chip, batt_psy);
	union power_supply_propval prop = {0,};
	int rc;
	
	if (!chip) {
		dev_err(chip->dev,"=======no\n");
		return;
	}

	mutex_lock(&chip->r_completed_lock);
	chip->external_change_waiting= true;
	if (!chip->resume_completed) {
		dev_dbg(chip->dev, "fan5405_external_power_changed before device-resume\n");
		mutex_unlock(&chip->r_completed_lock);
		return ;
	}
	chip->external_change_waiting= false;
	mutex_unlock(&chip->r_completed_lock);

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (rc < 0)
		dev_err(chip->dev,
			"could not read USB current_max property, rc=%d\n", rc);
	else
		chip->set_ivbus_max = prop.intval / 1000;


	rc = fan5405_set_ivbus_max(chip, chip->set_ivbus_max); //VBUS CURRENT

	power_supply_changed(&chip->batt_psy);
	pr_info("current_limit = %d\n", chip->set_ivbus_max);
	
}

static void fan5405_set_charged(struct power_supply *psy)
{
	struct fan5405_chip *chip = container_of(psy,
				struct fan5405_chip, batt_psy);

	fan5405_force_en_charging(chip, !chip->charging_disabled);
	dev_err(chip->dev,"chip->en_gpio value = %d  disabled = %d \n", gpio_get_value(chip->en_gpio), chip->charging_disabled);
	
	schedule_delayed_work(&chip->update_t32s_work, 0);
	power_supply_changed(&chip->batt_psy);
}

#if 0
/* HW initialization segment */
static int fan5405_hw_init(struct fan5405_chip *chip)
{
	int rc;

	/* 1. set charge safety register */
	if (!chip->safe_curr)
		chip->safe_curr = 1500;

	if (!chip->safe_vol)
		chip->safe_vol = 4230;
	
	rc = fan5405_set_safe_reg(chip, chip->safe_curr, chip->safe_vol);
	if (rc < 0) {
			dev_err(chip->dev,"fail to set  charge safety register, rc = %d\n", rc);

	}
 
	/* 1. set usb input current default value = 100mA */
	rc = fan5405_set_ivbus_max(chip, 100);
	if (rc < 0) {
			dev_err(chip->dev,"fail to set usb input current default, rc = %d\n", rc);
			//return rc;
	}

	/* 2. set weak battery voltage */
	if (!chip->weak_batt_vol)
		chip->weak_batt_vol = 3400;
	rc = fan5405_set_weak_batt_vol(chip, chip->weak_batt_vol);
	if (rc < 0) {
			dev_err(chip->dev,"fail to set weak battery voltage, rc = %d\n", rc);
			//return rc;
	}

	/* 3. set charge current max value */
	if (!chip->chg_curr_max)
		chip->chg_curr_max = 1000;
	rc = fan5405_set_ichg_max(chip, chip->chg_curr_max);
	if (rc < 0) {
			dev_err(chip->dev,"fail to set charge current max value, rc = %d\n", rc);
			//return rc;
	}

	/* 4. set charge voltage max value */
	if (!chip->chg_vol_max)
		chip->chg_vol_max = 4200;
	rc = fan5405_set_chg_vol_max(chip, chip->chg_vol_max);
	if (rc < 0) {
			dev_err(chip->dev,"fail to set charge current max value, rc = %d\n", rc);
			//return rc;
	}
	
	/* 5. set termination charge current */
	if (!chip->iterm_ma)
		chip->iterm_ma = 100;
	rc = fan5405_set_iterm(chip, chip->iterm_ma);
	if (rc < 0) {
			dev_err(chip->dev,"fail to set  termination charge current, rc = %d\n", rc);
			//return rc;
	}
	
	rc = fan5405_enable_te(chip, chip->enable_te);
	if (rc < 0) {
			dev_err(chip->dev,"fail to set te rc = %d\n", rc);
			//return rc;
	}

	/* 6. set special charger voltage */
	if (!chip->vsp_vol)
		chip->vsp_vol = 4500;
	rc = fan5405_set_vsp_vol(chip, chip->iterm_ma);
	if (rc < 0) {
			dev_err(chip->dev,"fail to set  vsp voltage, rc = %d\n", rc);
			//return rc;
	}

	/* 7. set charge safety current */
	if (!chip->safe_curr)
		chip->safe_curr = 1500;
	rc = fan5405_set_isafe(chip, chip->safe_curr);
	if (rc < 0) {
			dev_err(chip->dev,"fail to set  charge safety current, rc = %d\n", rc);
			//return rc;
	}

	/* 8. set charge safety voltage */
	if (!chip->safe_vol)
		chip->safe_vol = 4230;
	rc = fan5405_set_vsafe(chip, chip->safe_vol);
	if (rc < 0) {
			dev_err(chip->dev,"fail to set  charge safety voltage, rc = %d\n", rc);
			//return rc;
	}
	
	return rc;
}
#endif

static int fan5405_parse_dt(struct fan5405_chip *chip)
{
	int rc;
	struct device_node *node = chip->dev->of_node;
	
	if (!node) {
		dev_err(chip->dev, "device tree info. missing \n");
		return -EINVAL;
	}
	
	chip->en_gpio = of_get_named_gpio(node, "fan5405,en-gpio", 0);
	if (!gpio_is_valid(chip->en_gpio)) {
		dev_err(chip->dev,"chip->en_gpio = %dis invalid! \n", chip->en_gpio);
		return -EINVAL;
	}
	
	chip->irq_gpio = of_get_named_gpio(node, "fan5405,irq-gpio", 0);
	if (!gpio_is_valid(chip->irq_gpio)) {
		dev_err(chip->dev,"chip->irq_gpio = %dis invalid! \n", chip->irq_gpio);
		return -EINVAL;
	}
	
	rc = of_property_read_u32(node, "yl,max-vbus-current-mA", &chip->vbus_curr_max);
	if (rc < 0)
		return -EINVAL;

	rc = of_property_read_u32(node, "yl,weak-batt-vol-mv", &chip->weak_batt_vol);
	if (rc < 0)
		return -EINVAL;

	rc = of_property_read_u32(node, "yl,max-voltage-mv", &chip->chg_vol_max);
	if (rc < 0)
		return -EINVAL;

	rc = of_property_read_u32(node, "yl,max-charge-current-mA", &chip->chg_curr_max);
	if (rc < 0)
		return -EINVAL;
	chip->chg_curr_now = chip->chg_curr_max;
	
	rc = of_property_read_u32(node, "yl,term_current-mA", &chip->iterm_ma);
	if (rc < 0)
		return -EINVAL;

	rc = of_property_read_u32(node, "yl,vsp-mv", &chip->vsp_vol);
	if (rc < 0)
		return -EINVAL;

	rc = of_property_read_u32(node, "yl,safety-charge_current-mA", &chip->safe_curr);
	if (rc < 0)
		return -EINVAL;

	rc = of_property_read_u32(node, "yl,safety-voltage-mv", &chip->safe_vol);
	if (rc < 0)
		return -EINVAL;

	rc = of_property_read_u32(node, "yl,pon-volt-delta", &chip->pon_volt_delta);
	if (rc < 0){
		dev_err(chip->dev,"fail ===== chip->pon_volt_delta = %d \n", chip->pon_volt_delta);
		rc = 0;
		chip->pon_volt_delta = PON_VOLT_DELTA_DEFAULT;
		//return -EINVAL;
	}
	
	rc = of_property_read_u32(node, "yl,dischg-batt-mvolt-delta", &chip->dischg_batt_mvolt_delta);
	if (rc < 0){
		dev_err(chip->dev,"fail ===== chip->dischg_batt_mvolt_delta = %d \n", chip->dischg_batt_mvolt_delta);
		rc = 0;
		chip->dischg_batt_mvolt_delta = DISCHG_BATT_MVOLT_DELTA_DEFAULT;
		//return -EINVAL;
	}
	

	chip->enable_te = of_property_read_bool(node, "yl,enable-te");
	chip->charging_disabled = of_property_read_bool(node, "yl,charging-disabled");
	
	return rc;
}

static int fan5405_int_pinctrl_init(struct fan5405_chip *chip)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	chip->int_pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->int_pinctrl)) {
		dev_err(chip->dev,
			"Target does not use pinctrl\n");
		retval = PTR_ERR(chip->int_pinctrl);
		chip->int_pinctrl = NULL;
		return retval;
	}

	chip->int_state_active
		= pinctrl_lookup_state(chip->int_pinctrl, "active");
	if (IS_ERR_OR_NULL(chip->int_state_active)) {
		dev_err(chip->dev,
			"Can not get ts default pinstate\n");
		retval = PTR_ERR(chip->int_state_active);
		chip->int_pinctrl = NULL;
		return retval;
	}

	chip->int_state_suspend
		= pinctrl_lookup_state(chip->int_pinctrl, "sleep");
	if (IS_ERR_OR_NULL(chip->int_state_suspend)) {
		dev_err(chip->dev,
			"Can not get ts sleep pinstate\n");
		retval = PTR_ERR(chip->int_state_suspend);
		chip->int_pinctrl = NULL;
		return retval;
	}
	
	retval = pinctrl_select_state(chip->int_pinctrl, chip->int_state_active);
	if (retval) {
		dev_err(chip->dev,
				"can not set pins\n");
		return retval;
	}
	
	return 0;
}


static int fan5405_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc;
	struct fan5405_chip *chip;
	struct power_supply *usb_psy;
	

	chip = devm_kzalloc(&client->dev, sizeof(struct fan5405_chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}
	
	chip->client = client;
	chip->dev = &client->dev;
	
	rc = fan5405_parse_dt(chip);
	if (rc < 0) {
		dev_err(&client->dev, "Unable to parse DT nodes\n");
		return rc;
	}
	
	device_init_wakeup(chip->dev, 1);
	i2c_set_clientdata(client,chip);

	chip->shutdown_soc_invalid = false;
	rc = qpnp_pon_is_warm_reset();
	if (rc < 0) {
		pr_err("Error reading warm reset status rc=%d\n", rc);
		return rc;
	}
	chip->warm_reset = !!rc;
	
	chip->pon_volt = get_pon_batt_volt();
	pr_info("====pon_batt_volt=%d===\n", get_pon_batt_volt());
	if (BATT_CUTOFF_MVOLT > chip->pon_volt)
		chip->to_sys_capa = SOC_UNINITIALIZED;
	else
		chip->to_sys_capa = fan5405_batt_volt_to_capa(chip->pon_volt + chip->pon_volt_delta);


	chip->update_heartbeat_waiting = false;
	chip->update_t32s_waiting = false;
	chip->external_change_waiting = false;
	chip->resume_completed = true;
	mutex_init(&chip->r_completed_lock);
	
	mutex_init(&chip->reg_rw_lock);

	/* 1. set charge safety register */
	if (!chip->safe_curr)
		chip->safe_curr = 1500;

	if (!chip->safe_vol)
		chip->safe_vol = 4230;
	
	rc = fan5405_set_safe_reg(chip, chip->safe_curr, chip->safe_vol);
	if (rc < 0) {
			dev_err(chip->dev,"fail to set  charge safety register, rc = %d\n", rc);

	}
	//fan5405_hw_init(chip);

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_err(&client->dev, "USB supply not found; defer probe\n");
		return -EPROBE_DEFER;
	}
	chip->usb_psy = usb_psy;
	
	/* register battery power supply */
	chip->batt_psy.name		= "battery";
	chip->batt_psy.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.get_property	= fan5405_battery_get_property;
	chip->batt_psy.property_is_writeable = fan5405_battery_is_writeable;
	chip->batt_psy.set_property	= fan5405_battery_set_property;
	chip->batt_psy.properties	= fan5405_battery_properties;
	chip->batt_psy.num_properties  = ARRAY_SIZE(fan5405_battery_properties);
	chip->batt_psy.supplied_to = fan5405_batt_supplied_to;
	chip->batt_psy.num_supplicants =
					ARRAY_SIZE(fan5405_batt_supplied_to);
	chip->batt_psy.external_power_changed = fan5405_external_power_changed;
	chip->batt_psy.set_charged = fan5405_set_charged;
	
	rc = power_supply_register(chip->dev, &chip->batt_psy);
	if (rc < 0) {
		dev_err(&client->dev,
			"Unable to register batt_psy rc = %d\n", rc);
		goto fail_hw_init;
	}

	INIT_DELAYED_WORK(&chip->update_t32s_work, fan5405_update_t32s_work);
	INIT_DELAYED_WORK(&chip->update_heartbeat_work, fan5405_update_heartbeat_work);

	fan5405_int_pinctrl_init(chip);
	chip->irq = gpio_to_irq(chip->irq_gpio);
	rc = devm_request_irq(chip->dev, chip->irq, fan5405_stat_irq_handler,
				 IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "fan5405-stat", chip);
	if (rc < 0) {
		dev_err(chip->dev, "failed to request fan5405 irq\n");
		//return rc;
		goto unregister_batt_psy;
	}

	if (gpio_is_valid(chip->en_gpio)) {
		rc = gpio_request(chip->en_gpio, "fan_enable");
		if (rc) {		
			dev_err(chip->dev,"unable to request chip->en_gpio = %d ! \n", chip->en_gpio);
			goto unregister_batt_psy;
		}
		rc = gpio_direction_output(chip->en_gpio, 0);
		if (rc) {		
			dev_err(chip->dev,"unable to set direction for chip->en_gpio = %d ! \n", chip->en_gpio);
			goto unregister_batt_psy;
		}
	}

/* fan5405 en pin initialize */
	fan5405_force_en_charging(chip, !chip->charging_disabled);

	schedule_delayed_work(&chip->update_heartbeat_work,
		msecs_to_jiffies(UPDATE_HEART_PERIOD_FAST_MS));

	wake_lock_init(&chip->batt_info_alarm_wlock, WAKE_LOCK_SUSPEND, "batt_info_alarm");
	INIT_WORK(&chip->batt_info_alarm_work, batt_info_alarm_work);
	alarm_init(&chip->report_batt_info_alarm, ALARM_REALTIME,
			batt_info_alarm_callback);

	this_chip = chip;

	batt_info_alarm_set(chip, G_WAKEUP_INTERVAL);
	

	dev_err(&client->dev, "success ==== fan5405_probe\n");
	return 0;

unregister_batt_psy:
//	power_supply_unregister(&chip->batt_psy);
	gpio_free(chip->en_gpio);	
fail_hw_init:
		
	return rc;
}

static int fan5405_remove(struct i2c_client *client)
{
	int rc = 0;
	struct fan5405_chip *chip = i2c_get_clientdata(client);
	/* add active */
	power_supply_unregister(&chip->batt_psy);
	mutex_destroy(&chip->reg_rw_lock);
	cancel_delayed_work_sync(&chip->update_t32s_work);
	cancel_delayed_work_sync(&chip->update_heartbeat_work);
	alarm_cancel(&chip->report_batt_info_alarm);
	cancel_work_sync(&chip->batt_info_alarm_work);
	wake_lock_destroy(&chip->batt_info_alarm_wlock);
	gpio_free(chip->en_gpio);
	
	return rc;
}

static int fan5405_suspend(struct device *dev)
{
	int rc = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fan5405_chip *chip = i2c_get_clientdata(client);
	
	/* add active */
	mutex_lock(&chip->r_completed_lock);
	chip->resume_completed = false;
	mutex_unlock(&chip->r_completed_lock);
	
	return rc;
}

static int fan5405_suspend_noirq(struct device *dev)
{
//	int rc;
	struct i2c_client *client = to_i2c_client(dev);
	struct fan5405_chip *chip = i2c_get_clientdata(client);
	
	if (chip->update_heartbeat_waiting) {
		pr_err_ratelimited("Aborting suspend, an update_heartbeat_waiting while suspending\n");
		return -EBUSY;
	}

	if (chip->update_t32s_waiting) {
		pr_err_ratelimited("Aborting suspend, an update_t32s_waiting while suspending\n");
		return -EBUSY;
	}

	if (chip->external_change_waiting) {
		pr_err_ratelimited("Aborting suspend, an external_change_waiting while suspending\n");
		return -EBUSY;
	}
	
	return 0;
}

static int fan5405_resume(struct device *dev)
{
	int rc = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fan5405_chip *chip = i2c_get_clientdata(client);

	bool update_heartbeat_again = false;
	bool update_t32s_again = false;
	bool external_change_again = false;
	
	/* add active */
	int batt_mvol;
	batt_mvol = get_yl_pm8916_batt_mvol();
	pr_err("battery resume voltage : %d \n", batt_mvol);

	mutex_lock(&chip->r_completed_lock);
	chip->resume_completed = true;
	if (chip->update_heartbeat_waiting) {
		update_heartbeat_again = true;
	}

	if (chip->update_t32s_waiting) {
		update_t32s_again = true;
	}

	if (chip->external_change_waiting) {
		external_change_again = true;
	}
	mutex_unlock(&chip->r_completed_lock);
	
	if (update_heartbeat_again) {
		cancel_delayed_work_sync(&chip->update_heartbeat_work);
		fan5405_update_heartbeat_work(&chip->update_heartbeat_work.work);
	}

	if (update_t32s_again) {
		//	fan5405_set_charged(&chip->batt_psy);
		cancel_delayed_work_sync(&chip->update_t32s_work);
		fan5405_update_t32s_work(&chip->update_t32s_work.work);
	}

	if (external_change_again) {
		fan5405_external_power_changed(&chip->batt_psy);
	}
	
	return rc;
}

static const struct dev_pm_ops fan5405_pm_ops = {
	.resume = fan5405_resume,
	.suspend_noirq = fan5405_suspend_noirq,
	.suspend = fan5405_suspend,
};

static struct of_device_id fan5405_match_table[] = {
	{.compatible = "yl,fan5405_sm_charge",},
	{},
};

static const struct i2c_device_id fan5405_id[] = {
	{"fan5405_sm_charge", 0},
	{},
};

static struct i2c_driver fan5405_driver = {
	.driver = {
		.name = "fan5405_sm_charg",
		.owner = THIS_MODULE,
		.of_match_table = fan5405_match_table,
		.pm = &fan5405_pm_ops,
	},
	.probe = fan5405_probe,
	.remove = fan5405_remove,
	.id_table = fan5405_id,
};
module_i2c_driver(fan5405_driver);

MODULE_DESCRIPTION("FAN5405 charge IC driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:fan5405_sm_charge");
