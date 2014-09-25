/* Copyright (c) 2013-2014 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 *BQ24157 charge IC driver file, sm means switch mode
 *
 */

//#define POWER_TEST

#define pr_fmt(fmt) "BQ24157: %s: " fmt, __func__

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
#ifdef CONFIG_CHARGER_HAVE_LC709203_FUEL
#include "lc709203_fuelgauger_yl.h"
#endif



struct bq24157_chip {
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
	int 		chg_curr_now;
	int         iterm_ma;
	int         vsp_vol;
	int         safe_curr;
	int         safe_vol;
	
	bool      vbus_present;
	/* battery status tracking */
	bool      batt_present;
	bool      batt_too_cold;
	bool      batt_too_hot;
	int        batt_temp;
	int        batt_temp_status;
	int        batt_curr;
	int        batt_volt;
	int        batt_status;
	int        batt_crude_capa;
	int 	   pon_volt;
	int 	   pon_volt_delta;
	int 	   shutdown_ocv;
	int 	   shutdown_soc;
	int 	   warm_reset;

	int        charge_stat;
	int        to_sys_capa;

	/*  */
	int        set_ivbus_max;

	int 		last_capacity;
	
	/* add alarm for batt info report */
	struct wake_lock          batt_info_alarm_wlock;
	struct work_struct batt_info_alarm_work;
	struct alarm report_batt_info_alarm;

	struct power_supply     *usb_psy;
	struct power_supply     *vm_bms_psy;
	struct power_supply     batt_psy;
	
	//struct wake_lock          bq24157_wlock;
	//struct delayed_work     update_t32s_work;
	struct delayed_work 	bq_irq_work;
	struct delayed_work     update_heartbeat_work;

	struct mutex			reg_rw_lock;	
	struct mutex 			work_lock;
};

static struct bq24157_chip *this_chip;

#ifdef CONFIG_CHARGER_HAVE_LC709203_FUEL 
static int bq24157_get_prop_batt_capa(struct bq24157_chip *chip);
static int first_init_battery = 0;
#else
static int first_calc_capacity = 0;
static int cap_old = 0;
#endif

/* define registers macro segment */
#define BQ24157_CONTROL0        0x00
#define BQ24157_CONTROL1        0x01
#define BQ24157_OREG                0x02
#define BQ24157_IC_INFO           0x03
#define BQ24157_IBAT                 0x04
#define BQ24157_SP_CHARGER    0x05
#define BQ24157_SAFETY             0x06
#define BQ24157_MONITOR          0x10

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

#define PON_VOLT_DELTA_DEFAULT 50

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
	BATT_TEMP_TOO_COLD_THRESHOLD = -30,           /*  0 celsius*/
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


int array_soc[][2] = {  {4275, 100},
			{4260, 99}, {4245, 98}, {4232, 97}, {4220, 96},
			{4209, 95}, {4199, 94}, {4190, 93}, {4180, 92},
			{4169, 91}, {4159, 90}, {4147, 89}, {4136, 88},
			{4129, 87}, {4118, 86}, {4107, 85}, {4095, 84},
			{4084, 83}, {4073, 82}, {4062, 81}, {4050, 80},
			{4039, 79}, {4029, 78}, {4019, 77}, {4011, 76},
			{4055, 75}, {3995, 74}, {3981, 73}, {3965, 72},
			{3950, 71}, {3937, 70}, {3926, 69}, {3917, 68},
			{3909, 67}, {3902, 66}, {3896, 65}, {3888, 64},
			{3881, 63}, {3873, 62}, {3865, 61}, {3858, 60},
			{3850, 59}, {3845, 58}, {3839, 57}, {3832, 56},
			{3825, 55}, {3819, 54}, {3813, 53}, {3806, 52},
			{3800, 51}, {3795, 50}, {3789, 49}, {3786, 48},
			{3780, 47}, {3775, 46}, {3770, 45}, {3766, 44},
			{3761, 43}, {3756, 42}, {3752, 41}, {3748, 40},
			{3743, 39}, {3739, 38}, {3735, 37}, {3731, 36},
			{3727, 35}, {3724, 34}, {3721, 33}, {3718, 32},
			{3716, 31}, {3713, 30}, {3709, 29}, {3706, 28},
			{3703, 27}, {3700, 26}, {3697, 25}, {3694, 24},
			{3690, 23}, {3687, 22}, {3683, 21}, {3679, 20},
			{3675, 19}, {3671, 18}, {3666, 17}, {3661, 16},
			{3655, 15}, {3650, 14}, {3643, 13}, {3637, 12},
			{3633, 11}, {3628, 10}, {3623, 9},  {3617, 8},
			{3610, 7},  {3599, 6},	{3584, 5},  {3559, 4},
			{3517, 3},  {3465, 2},  {3402, 1},  {3350, 0} 
};


/* i2c operation API segment */
static int __bq24157_read(struct bq24157_chip *chip, int reg,
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

static int __bq24157_write(struct bq24157_chip *chip, int reg,
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

static int bq24157_read(struct bq24157_chip *chip, int reg,
				u8 *val)
{
	int rc = 0;

	mutex_lock(&chip->reg_rw_lock);
	rc = __bq24157_read(chip, reg, val);
	mutex_unlock(&chip->reg_rw_lock);

	return rc;
}

static int bq24157_write(struct bq24157_chip *chip, int reg,
						u8 val)
{
	int rc = 0;

	mutex_lock(&chip->reg_rw_lock);
	rc = __bq24157_write(chip, reg, val);
	mutex_unlock(&chip->reg_rw_lock);

	return rc;
}

#if 0
static int bq24157_read_bytes(struct bq24157_chip *chip, int reg,
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

static int bq24157_masked_write(struct bq24157_chip *chip, int reg,
						u8 mask, u8 val)
{
	s32 rc = 0;
	u8 temp = 0;

	mutex_lock(&chip->reg_rw_lock);
	rc = __bq24157_read(chip, reg, &temp);
	if (rc < 0) {
		dev_err(chip->dev, "read failed: reg=%02x, rc=%d\n", reg, rc);
		goto out;
	}
	temp &= ~mask;
	temp |= val & mask;
	pr_debug("write: reg=0x%02x, temp=0x%02x\n", reg, temp);
	rc = __bq24157_write(chip, reg, temp);
	if (rc < 0) {
		dev_err(chip->dev,
			"write failed: reg=%02x, rc=%d\n", reg, rc);
	}
out:
	mutex_unlock(&chip->reg_rw_lock);
	return rc;
}

#if 0
/* operate BQ24157 API segment */
static int bq24157_32s_tmr_rst(struct bq24157_chip *chip)
{
	return bq24157_masked_write(chip, BQ24157_CONTROL0,
				TMR_RST_MASK, TMR_RST_MASK);

}

static int bq24157_enable_stat(struct bq24157_chip *chip, bool enable)
{

	u8 temp = 0;

	if ( enable )
		temp = EN_STAT_MASK;
	
	return bq24157_masked_write(chip, BQ24157_CONTROL0,
				EN_STAT_MASK, temp);
}
#endif

static int bq24157_set_ivbus_max(struct bq24157_chip *chip, int current_ma)
{
	u8 temp = 0;
	u8 val = 0;
	
	if (current_ma <= 100)
		temp = 0x00;
	else if (current_ma <= 500)
		temp = 0x01;
	else if (current_ma <= 800)
		temp = 0x02;
	else
		temp = 0x03;

	val = (temp) << 6;
		
	pr_debug("Writing 0x%02x (7-6bit) = 0x%02x, current = %d\n", BQ24157_CONTROL1, val, current_ma);
	
	return bq24157_masked_write(chip, BQ24157_CONTROL1,
				INPUT_CURR_LIMIT_MASK, val);

}

#define WEAK_BATT_VOL_MIN      3400
#define WEAK_BATT_VOL_MAX      3700
#define WEAK_BATT_VOL_STEP       100
static int bq24157_set_weak_batt_vol(struct bq24157_chip *chip, int vol_mv)
{
	u8 temp = 0;

	if (vol_mv < WEAK_BATT_VOL_MIN ||vol_mv > WEAK_BATT_VOL_MAX) {
		dev_err(chip->dev, "set the weak battery threshold value error.\n");
		return -EINVAL;
	}
		
		
	temp = (vol_mv - WEAK_BATT_VOL_MIN) / WEAK_BATT_VOL_STEP;
	temp = (temp) << 4;
		
	pr_debug("Writing 0x%02x (5-4bit) = 0x%02x\n", BQ24157_CONTROL1, temp);
	
	return bq24157_masked_write(chip, BQ24157_CONTROL1,
				WEAK_BATT_VOL_MASK, temp);

}


static int bq24157_enable_te(struct bq24157_chip *chip, bool enable)
{
	u8 temp = 0;

	if (enable)
		temp = TERM_EN_MASK;
	
	return bq24157_masked_write(chip, BQ24157_CONTROL1,
				TERM_EN_MASK, temp);
}


static int bq24157_enable_chg(struct bq24157_chip *chip, bool enable)
{
	u8 temp = 0;

	if ( !enable )
		temp = CAHRGE_EN_MASK;
	
	return bq24157_masked_write(chip, BQ24157_CONTROL1,
				CAHRGE_EN_MASK, temp);
}

#ifdef DK
/* enable choose High impedance mode, disable choose not High impedance mode */
static int bq24157_enable_HZ_mode(struct bq24157_chip *chip, bool enable)
{
	u8 temp = 0;

	if ( enable )
		temp = HZ_MODE_MASK;
	
	return bq24157_masked_write(chip, BQ24157_CONTROL1,
				HZ_MODE_MASK, temp);
}

/* enable choose boost mode for otg function, disable choose charge mode */
static int bq24157_enable_OPA_mode(struct bq24157_chip *chip, bool enable)
{
	u8 temp = 0;

	if ( enable )
		temp = OPA_MODE_MASK;
	
	return bq24157_masked_write(chip, BQ24157_CONTROL1,
				OPA_MODE_MASK, temp);
}
#endif

#define CHG_VOL_MIN      3500
#define CHG_VOL_MAX      4440
#define CHG_VOL_STEP     20
static int bq24157_set_chg_vol_max(struct bq24157_chip *chip, int vol_mv)
{
	u8 temp = 0;
	
	if (vol_mv < CHG_VOL_MIN ||vol_mv > CHG_VOL_MAX) {
		dev_err(chip->dev, "set the weak battery threshold value error.\n");
		return -EINVAL;
	}
		
		
	temp = (vol_mv - CHG_VOL_MIN) / CHG_VOL_STEP;
	temp = (temp) << 2;
		
	pr_debug("Writing 0x%02x (7-2bit) = 0x%02x vol_mv=%d\n", BQ24157_OREG, temp, vol_mv);
	
	return bq24157_masked_write(chip, BQ24157_OREG,
				OREG_MASK, temp);
}

#ifdef DK
static int bq24157_set_otg_pl(struct bq24157_chip *chip, bool level)
{
	u8 temp = 0;

	if ( level )
		temp = OTG_PL;
	
	return bq24157_masked_write(chip, BQ24157_OREG,
				OTG_PL, temp);
}

static int bq24157_enable_otg(struct bq24157_chip *chip, bool enable)
{
	u8 temp = 0;

	if ( enable )
		temp = OTG_EN;
	
	return bq24157_masked_write(chip, BQ24157_OREG,
				OTG_EN, temp);
}
#endif

/* when Rsense = 68 */
#define CHARGE_CURR_MIN      550
#define CHARGE_CURR_MAX      1250
#define CHARGE_CURR_STEP     50
static const int charge_curr_limit[] = {550, 650, 750, 850, 950, 1050, 1150, 1250,};
#if 1
static int bq24157_set_ichg_max(struct bq24157_chip *chip, int current_ma)
{
	u8 temp = 0;
	u8 i = 0;
	for (i = ARRAY_SIZE(charge_curr_limit) - 1; i >= 0; i--) {
		if (charge_curr_limit[i] <= current_ma)
			break;
	}

	temp = i << 4;
#if 0
	if (current_ma < CHARGE_CURR_MIN ||current_ma > CHARGE_CURR_MAX) {
		dev_err(chip->dev, "set the charge current limit value error.\n");
		return -EINVAL;
	}
		
		
	temp = (current_ma - CHARGE_CURR_MIN) / CHARGE_CURR_STEP;
	temp = (temp) << 4;
#endif	
	pr_debug("Writing 0x%02x (6-4bit) = 0x%02x\n", BQ24157_IBAT, temp);
	
	return bq24157_masked_write(chip, BQ24157_IBAT,
				IOCHARGE_MASK, temp);
}
#endif
/* when Rsense = 68 */
#define I_TERM_MIN      49
#define I_TERM_MAX      388
#define I_TERM_STEP      388
static const int term_curr_limit[] = {49, 97, 146, 194, 243, 291, 340, 388,};
#if 1
static int bq24157_set_iterm(struct bq24157_chip *chip, int current_ma)
{
	u8 i = 0;

	for (i = 0; i < ARRAY_SIZE(term_curr_limit) - 1; i++) {
		if (term_curr_limit[i] >= current_ma)
			break;
	}

		
	pr_debug("Writing 0x%02x (2-0bit) = 0x%02x\n", BQ24157_IBAT, i);
	
	return bq24157_masked_write(chip, BQ24157_IBAT,
				ITEM_MASK, i);
}
#endif

#if 1
static int bq24157_set_iterm_ichg(struct bq24157_chip *chip, int current_ma, int term_ma)
{
	u8 i = 0;
	u8 temp = 0;
	u8 val = 0;

	pr_debug("current_ma = %d , term_ma = %d\n", current_ma, term_ma);
	for (i = ARRAY_SIZE(charge_curr_limit) - 1; i >= 0; i--) {
		if (charge_curr_limit[i] <= current_ma)
			break;
	}
	temp = i << 4;

	
	for (i = 0; i < ARRAY_SIZE(term_curr_limit) - 1; i++) {
		if (term_curr_limit[i] >= term_ma)
			break;
	}
	

	val = temp | i ;

	return bq24157_write(chip, BQ24157_IBAT, val);
}
#endif

#if 0
static int bq24157_reset_reg(struct bq24157_chip *chip)
{
	return bq24157_masked_write(chip, BQ24157_IBAT,
				RESET_MASK, RESET_MASK);
}
#endif

#ifdef DK
static int bq24157_set_dis_verg(struct bq24157_chip *chip, bool on)
{
	u8 temp = 0;
	if (!on) {
		temp = DIS_VREG_MASK;
	}
	
	pr_debug("Writing 0x%02x dis_verg = 0x%02x\n", BQ24157_SP_CHARGER, temp);
	return bq24157_masked_write(chip, BQ24157_SP_CHARGER,
				DIS_VREG_MASK, temp);
}
#endif

static int bq24157_set_io_level(struct bq24157_chip *chip, bool level)
{
	u8 temp = 0;
	
	if (level) {
		temp = IO_LEVEL;
	}

	pr_debug("Writing 0x%02x io = 0x%02x\n", BQ24157_SP_CHARGER, temp);
	return bq24157_masked_write(chip, BQ24157_SP_CHARGER,
				IO_LEVEL, temp);
}


#define SPECIAL_CHG_INPUT_VOL_MIN      4213
#define SPECIAL_CHG_INPUT_VOL_MAX      4773
#define SPECIAL_CHG_INPUT_VOL_STEP      80
static const int special_chg_input_vol[] = {4213, 4293, 4373, 4453, 4533, 4613, 4693, 4773,};
static int bq24157_set_vsp_vol(struct bq24157_chip *chip, int vol_mv)
{
	u8 i = 0;
	for (i = ARRAY_SIZE(special_chg_input_vol) - 1; i >= 0; i--) {
		if (special_chg_input_vol[i] <= vol_mv)
			break;
	}
	
	pr_debug("Writing 0x%02x (2-0bit) = 0x%02x\n", BQ24157_SP_CHARGER, i);
	return bq24157_masked_write(chip, BQ24157_SP_CHARGER,
				VSP_MASK, i);
}

/* when Rsense = 68 */
static const int isafe_limit[] = {550, 650, 750, 850, 950, 1050, 1150, 1250,};
#if 0
static int bq24157_set_isafe(struct bq24157_chip *chip, int current_ma)
{
	u8 temp = 0;
	u8 i = 0;
	for (i = ARRAY_SIZE(isafe_limit) - 1; i >= 0; i--) {
		if (isafe_limit[i] <= current_ma)
			break;
	}
	temp = i << 4;
	
	pr_debug("Writing 0x%02x (6-4bit) = 0x%02x\n", BQ24157_SAFETY, i);
	return bq24157_masked_write(chip, BQ24157_SAFETY,
				ISAFE_MASK, i);
}
#endif

#define VSAFE_MIN      4200
#define VSAFE_MAX      4440
#define VSAFE_step      20

#if 0 
static int bq24157_set_vsafe(struct bq24157_chip *chip, int vol_mv)
{
	u8 temp = 0;
	
	if (vol_mv < VSAFE_MIN ||vol_mv > VSAFE_MAX) {
		dev_err(chip->dev, "set the VSAFE value error.\n");
		return -EINVAL;
	}
		
		
	temp = (vol_mv - VSAFE_MIN) / VSAFE_step;
		
	pr_debug("Writing 0x%02x (3-0bit) = 0x%02x\n", BQ24157_SAFETY, temp);
	
	return bq24157_masked_write(chip, BQ24157_SAFETY,
				VSAGE_MASK, temp);
}
#endif

static int  bq24157_set_safe_reg(struct bq24157_chip *chip, int current_ma, int vol_mv)
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
		rc = bq24157_write(chip, BQ24157_SAFETY, temp);
		if (rc < 0) {
			dev_err(chip->dev,"fail to set  safety register, rc = %d\n", rc);
		}
		rc = bq24157_read(chip, BQ24157_SAFETY, &reg_val);
		if (rc < 0) {
			dev_err(chip->dev,"fail to read safety register, rc = %d\n", rc);
		}
		
		if (reg_val == temp)
			break;
		
		udelay(100);
	}
	
	pr_info("0x%02x  Writing  0x%02x :  Reading 0x%02x\n", BQ24157_SAFETY, temp, reg_val);
	
	return rc;
	
}

enum bq24157_stat {
	BQ24157_STAT_READY = 0,
	BQ24157_STAT_CHARGING,
	BQ24157_STAT_DONE,
	BQ24157_STAT_FAULT,	
};
static int bq24157_get_stat(struct bq24157_chip *chip)
{
	u8 reg_val = 0;
	u8 mask = 0;
	int rc = 0;

	mask = 0x30;
	rc = bq24157_read(chip, BQ24157_CONTROL0, &reg_val);
	if (rc < 0) {
		dev_err(chip->dev,"fail to read safety register, rc = %d\n", rc);
	}
	reg_val = reg_val & mask;
	pr_err("register 0x00 = 0x%02x \n",reg_val);
	return (reg_val >> 4);
	
}

#if 1
static void bq24157_force_en_charging(struct bq24157_chip *chip, bool enable)
{
	if (enable)
		gpio_set_value(chip->en_gpio, 0);
	else
		gpio_set_value(chip->en_gpio, 1);

	pr_err("chip->en_gpio value = %d  enable = %d \n", gpio_get_value(chip->en_gpio), enable);
	
}
#endif

static int bq24157_dump_registers(struct bq24157_chip *chip)
{
	int rc = 0;
	u8 reg_val = 0;
	u8 i = 0;

	for (i = 0; i <= 6; i++) {
		rc = bq24157_read(chip, i, &reg_val);
		if (rc < 0) {
			dev_err(chip->dev,"fail to read register : 0x%02x, rc = %d\n", i, rc);
		}
		pr_err("register 0x%02x = 0x%02x \n", i, reg_val);
	}
	
#if 0
	rc = bq24157_read(chip, BQ24157_MONITOR, &reg_val);
	if (rc < 0) {
		dev_err(chip->dev,"fail to read register : 0x%02x, rc = %d\n", BQ24157_MONITOR, rc);
	}
	pr_debug("register 0x%02x = 0x%02x \n", BQ24157_MONITOR, reg_val);
#endif
	return rc;
}

#if 1
/* HW initialization segment */
static int bq24157_hw_init(struct bq24157_chip *chip)
{
	int rc;

	/* 1. set charge safety register */
	if (!chip->safe_curr)
		chip->safe_curr = 1500;

	if (!chip->safe_vol)
		chip->safe_vol = 4230;
	
	rc = bq24157_set_safe_reg(chip, chip->safe_curr, chip->safe_vol);
	if (rc < 0) {
			dev_err(chip->dev,"fail to set  charge safety register, rc = %d\n", rc);

	}

	/* 1. set usb input current default value = 100mA */
	rc = bq24157_set_ivbus_max(chip, 100);
	if (rc < 0) {
			dev_err(chip->dev,"fail to set usb input current default, rc = %d\n", rc);
			//return rc;
	}

	/* 2. set weak battery voltage */
	if (!chip->weak_batt_vol)
		chip->weak_batt_vol = 3400;
	rc = bq24157_set_weak_batt_vol(chip, chip->weak_batt_vol);
	if (rc < 0) {
			dev_err(chip->dev,"fail to set weak battery voltage, rc = %d\n", rc);
			//return rc;
	}

	/* 3. set charge current max value */
	if (!chip->chg_curr_now)
		chip->chg_curr_now = 1000;
	rc = bq24157_set_ichg_max(chip, chip->chg_curr_now);
	if (rc < 0) {
		dev_err(chip->dev,"fail to set charge current max value, rc = %d\n", rc);
		//return rc;
	}

	rc = bq24157_enable_te(chip, true);
	rc = bq24157_enable_chg(chip, true);

#ifdef DK
	rc = bq24157_enable_HZ_mode(chip, false);
	rc = bq24157_enable_OPA_mode(chip, false);
#endif

	/* 4. set charge voltage max value */
	if (!chip->chg_vol_max)
		chip->chg_vol_max = 4200;
	rc = bq24157_set_chg_vol_max(chip, chip->chg_vol_max);
	if (rc < 0) {
			dev_err(chip->dev,"fail to set charge current max value, rc = %d\n", rc);
			//return rc;
	}

#ifdef DK
	rc = bq24157_set_otg_pl(chip, false);
	rc = bq24157_enable_otg(chip, false);
#endif
	
	/* 5. set termination charge current */
	if (!chip->iterm_ma)
		chip->iterm_ma = 100;
	rc = bq24157_set_iterm(chip, chip->iterm_ma);
	if (rc < 0) {
			dev_err(chip->dev,"fail to set  termination charge current, rc = %d\n", rc);
			//return rc;
	}

	/* 6. set special charger voltage */
	if (!chip->vsp_vol)
		chip->vsp_vol = 4500;
	rc = bq24157_set_vsp_vol(chip, chip->vsp_vol);
	if (rc < 0) {
			dev_err(chip->dev,"fail to set  vsp voltage, rc = %d\n", rc);
			//return rc;
	}

#ifdef DK
	rc = bq24157_set_dis_verg(chip, true);
#endif

	rc = bq24157_set_io_level(chip, false);
	
	return rc;
}
#endif


#if 0
/* set registers value */
static int bq24157_set_chg_reg(struct bq24157_chip *chip)
{
	int rc = 0;
#if 0
	//1.update T32 timer
	rc = bq24157_32s_tmr_rst(chip);
	if (rc < 0) {
			dev_err(chip->dev,"fail to update T32 timer, rc = %d\n", rc);
			//return rc;
	}

	//2. enable STAT pin LOW when IC is charging
	rc = bq24157_enable_stat(chip, true);
	if (rc < 0) {
			dev_err(chip->dev,"fail to enable STAT, rc = %d\n", rc);
			//return rc;
	}
#endif
	//1. 1SET  REG 0x00, update T32 timer and enable STAT pin LOW when IC is charging 
	rc = bq24157_masked_write(chip, BQ24157_CONTROL0,
				TMR_RST_MASK |EN_STAT_MASK, TMR_RST_MASK | EN_STAT_MASK);

	//1. 2 SET REG 0x01
	rc = bq24157_set_ivbus_max(chip, chip->set_ivbus_max); //VBUS CURRENT
	///rc = bq24157_set_ivbus_max(chip, 1500); //VBUS CURRENT

	
	if (!chip->weak_batt_vol)
		chip->weak_batt_vol = 3400;
	rc = bq24157_set_weak_batt_vol(chip, chip->weak_batt_vol);

	rc = bq24157_enable_te(chip, true);
	rc = bq24157_enable_chg(chip, true);
	rc = bq24157_enable_HZ_mode(chip, false);
	rc = bq24157_enable_OPA_mode(chip, false);

	//1. 3 SET REG 0x02
	if (!chip->chg_vol_max)
		chip->chg_vol_max = 4200;
	rc = bq24157_set_chg_vol_max(chip, chip->chg_vol_max);
	rc = bq24157_set_otg_pl(chip, false);
	rc = bq24157_enable_otg(chip, false);
	
	//1. 4 SET REG 0x04

	if (!chip->chg_curr_max)
		chip->chg_curr_max = 1000;
//	rc = bq24157_set_ichg_max(chip, chip->chg_curr_max);

	if (!chip->iterm_ma)
		chip->iterm_ma = 100;
//	rc = bq24157_set_iterm(chip, chip->iterm_ma);
	rc = bq24157_set_iterm_ichg(chip, chip->chg_curr_max, chip->iterm_ma);
	if (rc < 0) {
			dev_err(chip->dev,"fail to set 0x04 register value, rc = %d\n", rc);
			//return rc;
	}

	//1. 5 SET REG 0x05
	rc = bq24157_set_dis_verg(chip, true);
	rc = bq24157_set_io_level(chip, false);
	if (!chip->vsp_vol)
		chip->vsp_vol = 4500;
	rc = bq24157_set_vsp_vol(chip, chip->vsp_vol);

	return rc;
}
#endif


#if 0
#define UPDATE_T32S_PERIOD_MS      10000
static void bq24157_update_t32s_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct bq24157_chip *chip = container_of(dwork,
				struct bq24157_chip, update_t32s_work);
	int rc;
	pr_err("#########enter ==== %s\n", __func__);

	pm_stay_awake(chip->dev);

	chip->vbus_present = (1 == get_yl_pm8916_vbus_status());
	rc = bq24157_set_chg_reg(chip);
	bq24157_dump_registers(chip);
	pr_err(" %s: charge_stat = %d, vbus_present = %d, chip->charging_disabled = %d \n", __func__, chip->charge_stat, chip->vbus_present, chip->charging_disabled);
	if ((BQ24157_STAT_DONE == chip->charge_stat) || (!chip->vbus_present) || chip->charging_disabled)
		goto stop_update;
	
//update_again_later:
	schedule_delayed_work(&chip->update_t32s_work,
		msecs_to_jiffies(UPDATE_T32S_PERIOD_MS));
	return;

stop_update:

	pr_err("=======exit ==== %s\n", __func__);
	pm_relax(chip->dev);
}
#endif

#define BATT_CAPA_RECHG      98
static int bq24157_need_to_rechg(struct bq24157_chip *chip)
{

	chip->vbus_present = (1 == get_yl_pm8916_vbus_status());
	chip->charge_stat = bq24157_get_stat(chip);

	pr_err("charge_stat = %d, vbus_present = %d, chip->charging_disabled = %d \n", chip->charge_stat, chip->vbus_present, chip->charging_disabled);
	if ((chip->vbus_present) && (BATT_CAPA_RECHG >= chip->batt_crude_capa) 
		&& (BQ24157_STAT_CHARGING != chip->charge_stat)) {
		if (!chip->charging_disabled) { 	
			bq24157_force_en_charging(chip, chip->charging_disabled);
			msleep(500);
			bq24157_force_en_charging(chip, !chip->charging_disabled);
		}
	}

	return 0;
}

#ifndef CONFIG_CHARGER_HAVE_LC709203_FUEL
static int bq24157_calc_vbat(struct bq24157_chip *chip, int *ocv)
{
	int i, j, vbatt_adc;
	int vbatt_avg, vbatt_sum, vbatt_temp;
	int vbatt_buf[10];

	if(!ocv)
		return -EINVAL;
	
	for (i = 0, vbatt_sum = 0; i < 10; i++) {
		vbatt_adc = get_yl_pm8916_batt_mvol();
		vbatt_buf[i] = vbatt_adc;
		//pr_err("vbatt_buf[%d]=%d\n", i, vbatt_buf[i]);
		msleep(100);
	}
	
	for(j=0; j<9; j++){
		for(i=0; i<9-j; i++){
			if(vbatt_buf[i] > vbatt_buf[i+1]){
				vbatt_temp = vbatt_buf[i];
				vbatt_buf[i] = vbatt_buf[i+1];
				vbatt_buf[i+1] = vbatt_temp;
			}
		}
	} 
	
	for(i=1; i<9; i++){
		//pr_err("vbatt_buf[%d]=%d\n", i, vbatt_buf[i]);
		vbatt_sum += vbatt_buf[i];
	}
	
	vbatt_avg = vbatt_sum / 8;

	if(chip->vbus_present == 1)
		vbatt_avg = vbatt_avg - 50;
	else
		vbatt_avg = vbatt_avg + 50;

	pr_err("vbat_read = %d\n", vbatt_avg);

	*ocv = vbatt_avg;

	return 0;
}

#if 0
static int bq24157_get_bms_soc(struct bq24157_chip *chip)
{
	union power_supply_propval ret = {0, };
	int soc = 0;

	if (!chip->vm_bms_psy) {
		chip->vm_bms_psy = power_supply_get_by_name("bms");
		pr_err("not register bms power supply\n");
	}	
	if (chip->vm_bms_psy) {

		chip->vm_bms_psy->get_property(chip->vm_bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);

		soc = ret.intval;
	}
	
	pr_err("soc = %d\n", soc);

	return soc;
}
#endif

static int bq24157_calc_pon_soc(struct bq24157_chip *chip)
{
	int vbat = 0;
	int soc = 0;
	int count = 0;
	int i = 0;

	vbat = chip->pon_volt + chip->pon_volt_delta;
	chip->batt_volt = chip->pon_volt + chip->pon_volt_delta;
	
	count = ARRAY_SIZE(array_soc);
		if(vbat < array_soc[count-1][0]){
			soc = 0;
			pr_err("voltage outside the battery curve, chip->batt_volt = %d\n", chip->batt_volt);
			return 0;
		}
	
	for(i = 0; i < count; i++){
		if(vbat >= array_soc[i][0]){
			soc = array_soc[i][1];
			break;
		}
	}

	pr_err("pon soc = %d\n", soc);
		
	return soc;
}


static int bq24157_calc_soc(struct bq24157_chip *chip, int *soc)
{
	int ret = -EINVAL;
	int vbat = 0;
	int count = 0;
	int i = 0;
	

	ret = bq24157_calc_vbat(chip, &vbat);
	if(ret)
		return ret;

	chip->batt_volt = vbat;

	count = ARRAY_SIZE(array_soc);
	if(vbat < array_soc[count-1][0]){
		*soc = 0;
		pr_err("voltage outside the battery curve, chip->batt_volt = %d\n", chip->batt_volt);
		return 0;
	}

	for(i = 0; i < count; i++){
		if(vbat >= array_soc[i][0]){
			*soc = array_soc[i][1];
			break;
		}
	}

	pr_err("the cap is %d, vbat is %d\n", *soc, vbat);

	return 0;
}

static int bq24157_initial_soc(struct bq24157_chip *chip)
{
	int read_ocv;
	int read_soc;
	int calculate_soc;
	int shutdown_soc_invalid = 0;
	int rc;

	rc = yl_read_shutdown_ocv_soc(&read_ocv, &read_soc);
	if(rc < 0)
		shutdown_soc_invalid = 1;

#if 1
	rc = bq24157_calc_soc(chip, &calculate_soc);
	if(rc){
		pr_err("initial calculate soc failed\n");
	}
#else
	//calculate_soc = bq24157_calc_pon_soc(chip);
	calculate_soc = chip->last_capacity;
#endif

	if(chip->warm_reset){
		if(shutdown_soc_invalid){
			//rc = bq24157_calc_soc(chip, &calculate_soc);
			chip->last_capacity = calculate_soc;
			pr_err("warm_reset: using PON SOC\n");
		}
		else {
			chip->last_capacity = read_soc;
			chip->batt_volt = read_ocv;
			pr_err("warm_reset: using shutdown SOC\n");
		}
	} else {
	if(!shutdown_soc_invalid &&
		(abs(read_soc - calculate_soc) < 10)){
		chip->last_capacity = read_soc;
			chip->batt_volt = read_ocv;
			pr_err("using shutdown SOC\n");
		} else {
			pr_err("using PON SOC\n");
		}
	}

	pr_err("shutdown_soc_invalid = %d read_ocv = %d read_soc = %d calculate_soc = %d\n", shutdown_soc_invalid, read_ocv, read_soc, calculate_soc);
	
	return 0;
}


#define CAPACITY_BASE   60
static int bq24157_calc_capacity(struct bq24157_chip *chip)
{
	struct timeval time_current;
	static int time_old, time_now, cap_time;
	int ret = 0;
	int cap = 0;

	do_gettimeofday(&time_current);
	time_now = time_current.tv_sec;
	
	if(first_calc_capacity == 0) {
		bq24157_initial_soc(chip);
		cap_old = chip->last_capacity * CAPACITY_BASE;
		time_old = time_now;
		first_calc_capacity = 1;
		pr_err("the first calc_capacity\n");
	}

	ret = bq24157_calc_soc(chip, &cap);
	if(ret)
		return ret;

	chip->batt_crude_capa = cap;

	if(cap < 0)
		cap = 0;
	else if (cap > 100)
		cap = 100;

	cap *= CAPACITY_BASE;

	cap_time = time_now-time_old;
	if(cap_time < 0)
		cap_time = 30;

	pr_err("[before] the cap_time is %d\n", cap_time);

	if(cap_time >= 3600)
		cap_time = 120;
	else if(cap_time >= 60)
		cap_time = 60;
	else if(cap_time >= 30)
	{
		if((chip->last_capacity <= 15) && (chip->last_capacity > 5))
			cap_time = 60;
		else if(chip->last_capacity <= 5)
			cap_time = 150;
	}

	pr_err("[after] the cap_time is %d\n", cap_time);
	pr_err("the cap_old is %d\n", cap_old);
	
	if(chip->batt_status == POWER_SUPPLY_STATUS_CHARGING || chip->batt_status == POWER_SUPPLY_STATUS_FULL)
	{
		if(cap > cap_old + cap_time)
			cap = cap_old + cap_time;
		else if(cap <= cap_old)
			cap = cap_old;			
	}
	else
	{
			if(cap < cap_old - cap_time)
				cap = cap_old - cap_time;
			else if(cap >= cap_old)
				cap = cap_old;
	}

	time_old = time_now;
	cap_old = cap;
	chip->last_capacity = cap/CAPACITY_BASE;

	pr_err("chip->batt_crude_capa = %d chip->last_capacity = %d\n", chip->batt_crude_capa, chip->last_capacity);

	return ret;
}
#endif

//DK test, 4 degree margin.
static int get_batt_temp_status(struct bq24157_chip *chip)
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

        pr_err("batt_temp =%d, b_status = %d\n", batt_temp, b_status);
        return b_status;
}

#define STEP_CHG_THRESHOLD_VOLT_MV        4000
#define STEP_CHG_DELTA_VOLT_MV                  20
static int bq24157_temp_appropriate_charging(struct bq24157_chip *chip)
{
        static int last_temp_sts = BATT_TEMP_STATUS_UNKOWN;
        static bool last_volt_sts = false;
        static bool batt_volt_low = false;
        static bool batt_too_cold = false;
        static bool batt_too_hot = false;

        int batt_volt_mv = 0;
        /**/
        int chg_current = 0;

		chip->batt_temp = get_yl_pm8916_batt_temp();
        chip->batt_temp_status = get_batt_temp_status(chip);//set buffer
        batt_volt_mv = chip->batt_volt;
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

                chip->chg_curr_now  = min(chg_current,  chip->chg_curr_max);
                pr_err("setting %d mA, %d, temp = %d, batt_temp_status = %d\n",chip->chg_curr_now, chg_current, chip->batt_temp, chip->batt_temp_status);
                bq24157_set_iterm_ichg(chip, chip->chg_curr_now, chip->iterm_ma);


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

                        bq24157_force_en_charging(chip, !chip->charging_disabled);
                }
        }


        return 0;
}


#define BQ24157_SHIFT_FAULT			0
#define BQ24157_MASK_FAULT			(0x7 << BQ24157_SHIFT_FAULT)

enum bq24157_fault_t {
	BQ24157_FAULT_NORMAL = 0,
	BQ24157_FAULT_VBUS_OVP,
	BQ24157_FAULT_SLEEP_MODE,
	BQ24157_FAULT_POOR_INPUT,
	BQ24157_FAULT_BATT_OVP,
	BQ24157_FAULT_THERMAL,
	BQ24157_FAULT_TIMER,
	BQ24157_FAULT_NO_BATT,
};
static int bq24157_get_charging_fault(struct bq24157_chip *chip)
{
	u8 value = 0;

	bq24157_read(chip, BQ24157_CONTROL0, &value);
	
	switch(value & BQ24157_MASK_FAULT) {		
		case BQ24157_FAULT_NORMAL:
			pr_info("none\n");
			break;
		case BQ24157_FAULT_VBUS_OVP:
			pr_info("vbus ovp\n");
			break;
		case BQ24157_FAULT_SLEEP_MODE:
			pr_info("sleep mode\n");
			break;
		case BQ24157_FAULT_POOR_INPUT:
			pr_info("poor input source\n");
			break;
		case BQ24157_FAULT_BATT_OVP:
			pr_info("battery ovp\n");
			break;
		case BQ24157_FAULT_THERMAL:
			pr_info("thermal fault\n");
			break;
		case BQ24157_FAULT_TIMER:
			pr_info("timer fault\n");
			break;
		case BQ24157_FAULT_NO_BATT:
			pr_info("no battery\n");
			break;
		}
		
		return (value & BQ24157_MASK_FAULT);
}

static void bq24157_update_battery_status(struct bq24157_chip *chip)
{
	int chg_stat;
	
	pr_err("enter\n");
	
	mutex_lock(&chip->work_lock);

	chg_stat = bq24157_get_stat(chip);
	
	switch (chg_stat) {
		case BQ24157_STAT_READY:
			pr_info("ready\n");
			chip->batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case BQ24157_STAT_CHARGING:
			pr_info("charging\n");
			chip->batt_status = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case BQ24157_STAT_DONE:
			pr_info("chg done\n");
			chip->batt_status = POWER_SUPPLY_STATUS_FULL;
			break;
		case BQ24157_STAT_FAULT:
			pr_info("chg fault\n");
			bq24157_get_charging_fault(chip);
			bq24157_hw_init(chip);   //why fault??
			bq24157_force_en_charging(chip, !chip->charging_disabled);
			break;
		default:
			break;
	}



#ifdef CONFIG_CHARGER_HAVE_LC709203_FUEL
	chip->last_capacity = bq24157_get_prop_batt_capa(chip);
	chip->batt_crude_capa = chip->last_capacity;
	chip->batt_temp = get_yl_pm8916_batt_temp();
	chip->batt_volt = get_yl_pm8916_batt_mvol();
	if(first_init_battery == 0){
		lc709203_hw_init_by_battery();
		first_init_battery = 1;
	}
	//pr_err("report soc = %d temp = %d batt_volt = %d\n",chip->last_capacity,chip->batt_temp,chip->batt_volt);
#else
	bq24157_calc_capacity(chip);
#endif	
	bq24157_temp_appropriate_charging(chip);

#ifndef CONFIG_CHARGER_HAVE_LC709203_FUEL
	if(chip->batt_volt <= 3350){
		pr_err("low voltage, report capacity 0 to shutdown\n");
		chip->last_capacity = 0;
	}
#endif

	bq24157_need_to_rechg(chip);

	power_supply_changed(&chip->batt_psy);
	
	mutex_unlock(&chip->work_lock);
}
 
#define UPDATE_HEART_PERIOD_FAST_MS      20000
#define UPDATE_HEART_PERIOD_NORMAL_MS    60000  //120000
#define BATT_CAPA_LOW_LEVEL      15
static void bq24157_update_heartbeat_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct bq24157_chip *chip = container_of(dwork,
				struct bq24157_chip, update_heartbeat_work);
	int update_period = UPDATE_HEART_PERIOD_NORMAL_MS;

	pr_err(" enter\n");
	
	bq24157_dump_registers(chip);
	
	//get_yl_pm8916_batt_mvol();
	//get_yl_pm8916_batt_temp();
	//ad_yl_pm8916_batt_id();
	//power_supply_changed(&chip->batt_psy);
	//chip->vbus_present = 1;
	//chip->charge_stat = bq24157_get_stat(chip);

	
	chip->vbus_present = (1 == get_yl_pm8916_vbus_status());
	chip->batt_present = (1 == is_battery_present());
	pr_err(" charge_stat = %d, vbus_present = %d batt_present = %d\n", chip->charge_stat, chip->vbus_present, chip->batt_present);

	//battery not present, stop update status.
	bq24157_update_battery_status(chip);

	yl_backup_ocv_soc(chip->batt_volt, chip->last_capacity);
	
	if(chip->batt_crude_capa< BATT_CAPA_LOW_LEVEL)
		update_period = UPDATE_HEART_PERIOD_FAST_MS;
	
	schedule_delayed_work(&chip->update_heartbeat_work,
		msecs_to_jiffies(update_period));
	
	return;
}


/* 30minute */
#define G_WAKEUP_INTERVAL		         30*60  
#define G_CHARGE_WAKEUP_INTERVAL		  1*60

static void batt_info_alarm_set(struct bq24157_chip *chip, int seconds)
{
	ktime_t interval = ktime_set(seconds, 0);

	pr_err("batt_info_alarm_set has been setting. \n");
	
	alarm_start_relative(&chip->report_batt_info_alarm, interval);
}

#if 0
static void batt_info_alarm_work(struct work_struct *work)
{
	struct bq24157_chip *chip = container_of(work, struct bq24157_chip, batt_info_alarm_work);
	if (!this_chip) {
		pr_err("chip not yet initalized\n");
	}
	
	pr_err("enter %s . \n", __func__);

	power_supply_changed(&chip->batt_psy); 
	
	if (BATT_CAPA_LOW_LEVEL < chip->batt_crude_capa )
		batt_info_alarm_set(chip, 2*G_WAKEUP_INTERVAL);
	else
		batt_info_alarm_set(chip, G_WAKEUP_INTERVAL);
	
	wake_unlock(&chip->batt_info_alarm_wlock);

}
#endif

static enum alarmtimer_restart batt_info_alarm_callback(struct alarm *alarm, ktime_t now)
{
	//struct bq24157_chip *chip = container_of(alarm, struct bq24157_chip, report_batt_info_alarm);

	pr_err("battery alarm is coming\n");
	//wake_lock(&chip->batt_info_alarm_wlock);
	//schedule_work(&chip->batt_info_alarm_work);
	
	return ALARMTIMER_NORESTART;
}

#if 0
static void bq24157_irq_schedule_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct bq24157_chip *chip = container_of(dwork,struct bq24157_chip, bq_irq_work);
	int stat;
	
	pr_err(" enter\n");

	stat = bq24157_get_stat(chip);

	switch (stat) {
	case BQ24157_STAT_READY:
		pr_info("%s: ready\n", __func__);
		break;
	case BQ24157_STAT_CHARGING:
		pr_info("%s: charging\n", __func__);
		break;
	case BQ24157_STAT_DONE:
		pr_info("%s: chg done\n", __func__);
		break;
	case BQ24157_STAT_FAULT:
		pr_info("%s: chg fault\n", __func__);
		bq24157_get_charging_fault(chip);

		break;
	}

	return;
}

static irqreturn_t bq24157_stat_irq_handler(int irq, void *_chip)
{
	struct bq24157_chip *chip = _chip;
	
	pr_info("bq24157 stat irq handler \n");
	pr_info("chip->irq_gpio = %d\n", gpio_get_value(chip->irq_gpio));
	
	schedule_delayed_work(&chip->bq_irq_work, 0);
	
	return IRQ_HANDLED;
}
#endif

/* power supply segment */
static enum power_supply_property bq24157_battery_properties[] = {
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
};


static char *bq24157_batt_supplied_to[] = {
	"bms",
};

static int bq24157_get_prop_batt_health(struct bq24157_chip *chip)
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

static int bq24157_get_prop_batt_status(struct bq24157_chip *chip)
{
	union power_supply_propval ret = {0, };
	
	chip->charge_stat = bq24157_get_stat(chip);

    if ((BQ24157_STAT_DONE == chip->charge_stat) && (chip->last_capacity == 100))
    	ret.intval = POWER_SUPPLY_STATUS_FULL;
    else if (BQ24157_STAT_CHARGING == chip->charge_stat)
        ret.intval = POWER_SUPPLY_STATUS_CHARGING;
    else
        ret.intval = POWER_SUPPLY_STATUS_DISCHARGING;
	
	return ret.intval;
}

static int bq24157_get_prop_batt_present(struct bq24157_chip *chip)
{
	union power_supply_propval ret = {0, };

	chip->batt_present = (1 == is_battery_present());

	ret.intval = chip->batt_present;

#ifdef POWER_TEST
	return 1;
#else
	return ret.intval;
#endif
}

static int bq24157_get_prop_charging_enable(struct bq24157_chip *chip)
{
	union power_supply_propval ret = {0, };


	ret.intval = !(chip->charging_disabled);

	return ret.intval;
}

static int bq24157_get_prop_charge_type(struct bq24157_chip *chip)
{
	union power_supply_propval ret = {0, };


	ret.intval = POWER_SUPPLY_CHARGE_TYPE_FAST;

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static int bq24157_get_prop_charge_full_design(struct bq24157_chip *chip)
{
	union power_supply_propval ret = {0, };


	ret.intval = 3000;
	return ret.intval;

}

static int bq24157_get_prop_voltage_max_design(struct bq24157_chip *chip)
{
	union power_supply_propval ret = {0, };


	ret.intval = chip->chg_vol_max * 1000;

	return ret.intval;
}

static int bq24157_get_prop_voltage_now(struct bq24157_chip *chip)
{
	union power_supply_propval ret = {0, };


	ret.intval = chip->batt_volt * 1000;

	return ret.intval;
}

static int bq24157_get_prop_batt_temp(struct bq24157_chip *chip)
{
	union power_supply_propval ret = {0, };


	ret.intval = chip->batt_temp;

	return ret.intval;
}

static int bq24157_get_prop_batt_capa(struct bq24157_chip *chip)
{
#ifdef CONFIG_CHARGER_HAVE_LC709203_FUEL
	int percent_soc = 0;
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
	pr_err("current percent = %d data:%d\n", percent_soc,data);
	return percent_soc;
#else
#ifdef POWER_TEST
	return 88;
#else
	//return chip->last_capacity;
	return chip->batt_crude_capa;
#endif
#endif


}

#if 0
#define LOWER      (0)
#define HIGER      (2)
#define CALIBRATE_CAPA(x)      ((100+HIGER)*((x)-LOWER)/(100-LOWER))
static long long get_wall_time(void)
{
	struct timespec     wall_time;
	getnstimeofday(&wall_time);
	return wall_time.tv_sec;
}
#define SOC_TOO_DIFF        20
#define EXPIRE_TIME           35 /* 15 secs */
static int bq24157_calculated_to_sys_batt_capa(struct bq24157_chip *chip)
{
	int reported_cap =0; /* YULONG added for fixed battery capacity more than 100 */
	static int first_time = 0;
	long long now_tm_sec = 0;
	static long long last_update_tm_sec = 0;
	chip->vbus_present = (1 == get_yl_pm8916_vbus_status());
	chip->batt_crude_capa = bq24157_get_prop_batt_capa(chip);
	now_tm_sec = get_wall_time();
	pr_err("now_tm_sec = %lld, last_update_tm_sec = %lld\n", now_tm_sec, last_update_tm_sec);
	pr_err("before to_sys_capa = %d, batt_crude_capa = %d\n", chip->to_sys_capa, chip->batt_crude_capa);
	if (first_time == 0) {
		first_time = 1;
		last_update_tm_sec = now_tm_sec + EXPIRE_TIME; /*test*/
		chip->to_sys_capa = chip->batt_crude_capa;
	}
	
	if (now_tm_sec > last_update_tm_sec ) {
		last_update_tm_sec = now_tm_sec + EXPIRE_TIME; 

		if (chip->vbus_present == false){
		/* charger unplugged */
			pr_err("charger unplugged \n");
			if (chip->to_sys_capa > chip->batt_crude_capa) {
				chip->to_sys_capa--;
				if ((chip->to_sys_capa - chip->batt_crude_capa) > SOC_TOO_DIFF)
					chip->to_sys_capa--;
			}
		} else {
		/* charger plugged */
			pr_err("charger plugged \n");
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

	 /* YULONG added for fixed battery capacity more than 100 */
	reported_cap = CALIBRATE_CAPA(chip->to_sys_capa);
	if ( reported_cap > 100 )
		reported_cap = 100;
	if ( reported_cap < 0 )
		reported_cap = 0; 
	pr_err("now_tm_sec = %lld, last_update_tm_sec = %lld\n", now_tm_sec, last_update_tm_sec);
	pr_err("after to_sys_capa = %d, batt_crude_capa = %d, reported_cap = %d \n", chip->to_sys_capa, chip->batt_crude_capa, reported_cap);
	return reported_cap;
}
#endif

static int bq24157_battery_get_property(struct power_supply *psy, 
										enum power_supply_property prop,
										union power_supply_propval *val)
{
	struct bq24157_chip *chip = container_of(psy, struct bq24157_chip, batt_psy);
	
	switch (prop) {
		case POWER_SUPPLY_PROP_HEALTH:
			val->intval = bq24157_get_prop_batt_health(chip);
			break;
		case POWER_SUPPLY_PROP_STATUS:
			val->intval = bq24157_get_prop_batt_status(chip);
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = bq24157_get_prop_batt_present(chip);
			break;	
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = bq24157_get_prop_batt_capa(chip);
			break;
		case POWER_SUPPLY_PROP_CHARGING_ENABLED:
			val->intval = bq24157_get_prop_charging_enable(chip);
			break;
		case POWER_SUPPLY_PROP_CHARGE_TYPE:
			val->intval = bq24157_get_prop_charge_type(chip);
			break;
		case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
			val->intval = bq24157_get_prop_charge_full_design(chip);
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
			val->intval = bq24157_get_prop_voltage_max_design(chip);
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval = bq24157_get_prop_voltage_now(chip);
			break;
		case POWER_SUPPLY_PROP_TEMP:
			val->intval = bq24157_get_prop_batt_temp(chip);
			break;
		default:
			return -EINVAL;
	}
	
	return 0;
}
								
static int bq24157_battery_is_writeable(struct power_supply *psy,
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

static int bq24157_battery_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	struct bq24157_chip *chip = container_of(psy,
				struct bq24157_chip, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		chip->charging_disabled = !(val->intval); 	
		bq24157_force_en_charging(chip, !chip->charging_disabled);
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

static void bq24157_external_power_changed(struct power_supply *psy)
{
	struct bq24157_chip *chip = container_of(psy,
				struct bq24157_chip, batt_psy);
	union power_supply_propval prop = {0,};
	int rc;
	
	if (!chip) {
		dev_err(chip->dev,"=======no\n");
		return;
	}

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (rc < 0)
		dev_err(chip->dev,
			"could not read USB current_max property, rc=%d\n", rc);
	else
		chip->set_ivbus_max = prop.intval / 1000;

	//rc = set_yl_pm8916_chg_iusbmax(chip->set_ivbus_max);

	rc = bq24157_set_ivbus_max(chip, chip->set_ivbus_max); //VBUS CURRENT

	pr_info("vbus_present = %d current_limit = %d\n", chip->vbus_present, chip->set_ivbus_max);
	
}


static void bq24157_set_charged(struct power_supply *psy)
{
	struct bq24157_chip *chip = container_of(psy,
				struct bq24157_chip, batt_psy);
	
	bq24157_force_en_charging(chip, !chip->charging_disabled);
	pr_err("chip->en_gpio value = %d  disabled = %d \n", gpio_get_value(chip->en_gpio), chip->charging_disabled);
	//schedule_delayed_work(&chip->update_t32s_work, 0);
}

static int bq24157_parse_dt(struct bq24157_chip *chip)
{
	int rc;
	struct device_node *node = chip->dev->of_node;
	
	if (!node) {
		dev_err(chip->dev, "device tree info. missing \n");
		return -EINVAL;
	}
	
	chip->en_gpio = of_get_named_gpio(node, "bq24157,en-gpio", 0);
	if (!gpio_is_valid(chip->en_gpio)) {
		dev_err(chip->dev,"chip->en_gpio = %dis invalid! \n", chip->en_gpio);
		return -EINVAL;
	}
	dev_err(chip->dev,"chip->en_gpio = %d =========== \n", chip->en_gpio);
	//chip->irq_gpio = of_get_named_gpio_flags(node, "bq24157,irq-gpio", 0, &chip->irq_flag);
	chip->irq_gpio = of_get_named_gpio(node, "bq24157,irq-gpio", 0);
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

	chip->enable_te = of_property_read_bool(node, "yl,enable-te");
	chip->charging_disabled = of_property_read_bool(node, "yl,charging-disabled");
	
	dev_err(chip->dev,"charging_disabled = %d \n", chip->charging_disabled);
	return rc;
}

static int bq24157_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc;
	struct bq24157_chip *chip;
	struct power_supply *usb_psy;
#if 0
	struct power_supply *bms_psy;
#endif
	
	dev_err(&client->dev, "bq24157_probe\n");

	chip = devm_kzalloc(&client->dev, sizeof(struct bq24157_chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}
	
	chip->client = client;
	chip->dev = &client->dev;
	
	rc = bq24157_parse_dt(chip);
	if (rc < 0) {
		dev_err(&client->dev, "Unable to parse DT nodes\n");
		return rc;
	}
	
	device_init_wakeup(chip->dev, 1);
	i2c_set_clientdata(client,chip);
	
	mutex_init(&chip->reg_rw_lock);
	mutex_init(&chip->work_lock);
	
	bq24157_hw_init(chip);

#ifdef CONFIG_CHARGER_HAVE_LC709203_FUEL
	rc = qpnp_pon_is_warm_reset();
	if (rc < 0) {
		pr_err("Error reading warm reset status rc=%d\n", rc);
		//return rc;
	}
	chip->warm_reset = !!rc;

	chip->last_capacity = bq24157_get_prop_batt_capa(chip);
	chip->pon_volt = get_pon_batt_volt();
	pr_err("chip->last_capacity = %d chip->pon_volt = %d chip->warm_reset =%d\n",chip->last_capacity,chip->pon_volt,chip->warm_reset);
#else
	rc = qpnp_pon_is_warm_reset();
	if (rc < 0) {
		pr_err("Error reading warm reset status rc=%d\n", rc);
		return rc;
	}
	chip->warm_reset = !!rc;

	chip->pon_volt = get_pon_batt_volt();

	chip->last_capacity = bq24157_calc_pon_soc(chip);
	pr_err("pon_batt_volt = %d chip->last_capacity = %d\n", chip->pon_volt, chip->last_capacity);
#endif

#if 0   //bms not used	
	bms_psy = power_supply_get_by_name("bms");
	if (!bms_psy) {
		dev_err(&client->dev, "bms supply not found; defer probe\n");
		return -EPROBE_DEFER;
	}
	chip->vm_bms_psy = bms_psy;
#endif

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_err(&client->dev, "USB supply not found; defer probe\n");
		return -EPROBE_DEFER;
	}
	chip->usb_psy = usb_psy;
	
	/* register battery power supply */
	chip->batt_psy.name		= "battery";
	chip->batt_psy.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.get_property	= bq24157_battery_get_property;
	chip->batt_psy.property_is_writeable = bq24157_battery_is_writeable;
	chip->batt_psy.set_property	= bq24157_battery_set_property;
	chip->batt_psy.properties	= bq24157_battery_properties;
	chip->batt_psy.num_properties  = ARRAY_SIZE(bq24157_battery_properties);
	chip->batt_psy.supplied_to = bq24157_batt_supplied_to;
	chip->batt_psy.num_supplicants =
					ARRAY_SIZE(bq24157_batt_supplied_to);
	chip->batt_psy.external_power_changed = bq24157_external_power_changed;
	chip->batt_psy.set_charged = bq24157_set_charged;
	
	rc = power_supply_register(chip->dev, &chip->batt_psy);
	if (rc < 0) {
		dev_err(&client->dev,
			"Unable to register batt_psy rc = %d\n", rc);
		goto fail_hw_init;
	}

	//INIT_DELAYED_WORK(&chip->update_t32s_work, bq24157_update_t32s_work);
	INIT_DELAYED_WORK(&chip->update_heartbeat_work, bq24157_update_heartbeat_work);
	//INIT_DELAYED_WORK(&chip->bq_irq_work, bq24157_irq_schedule_work);
	
#if 0
	chip->irq = gpio_to_irq(chip->irq_gpio);
	dev_err(chip->dev, "irq = %d, irq_gpio = %d\n", chip->irq, chip->irq_gpio);
	rc = devm_request_irq(chip->dev, chip->irq, bq24157_stat_irq_handler,
				 IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "bq24157-stat", chip);
	if (rc < 0) {
		dev_err(chip->dev, "failed to request bq24157 irq\n");
		//return rc;
		goto unregister_batt_psy;
	}
#endif

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

	bq24157_force_en_charging(chip, !chip->charging_disabled);
	pr_err("chip->en_gpio value = %d  disabled = %d \n", gpio_get_value(chip->en_gpio), chip->charging_disabled);
	
	schedule_delayed_work(&chip->update_heartbeat_work,
		msecs_to_jiffies(3000));

	wake_lock_init(&chip->batt_info_alarm_wlock, WAKE_LOCK_SUSPEND, "batt_info_alarm");
	alarm_init(&chip->report_batt_info_alarm, ALARM_BOOTTIME,
			batt_info_alarm_callback);

	this_chip = chip;

	//batt_info_alarm_set(chip, G_WAKEUP_INTERVAL);
	

	dev_err(&client->dev, "bq24157_probe end\n");
	return 0;

unregister_batt_psy:
//	power_supply_unregister(&chip->batt_psy);
	gpio_free(chip->en_gpio);	
fail_hw_init:
		
	return rc;
}

static int bq24157_remove(struct i2c_client *client)
{
	int rc = 0;
	struct bq24157_chip *chip = i2c_get_clientdata(client);
	/* add active */
	power_supply_unregister(&chip->batt_psy);
	mutex_destroy(&chip->reg_rw_lock);
	//cancel_delayed_work_sync(&chip->update_t32s_work);
	cancel_delayed_work_sync(&chip->update_heartbeat_work);
	alarm_cancel(&chip->report_batt_info_alarm);
	//cancel_work_sync(&chip->batt_info_alarm_work);
	wake_lock_destroy(&chip->batt_info_alarm_wlock);
	gpio_free(chip->en_gpio);
	
	return rc;
}

static int bq24157_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq24157_chip *chip = i2c_get_clientdata(client);

	pr_err("enter\n");
	
	cancel_delayed_work_sync(&chip->update_heartbeat_work);

	if(get_yl_pm8916_vbus_status()){
		batt_info_alarm_set(chip, G_CHARGE_WAKEUP_INTERVAL);
	} 
	else {
		if (BATT_CAPA_LOW_LEVEL < chip->batt_crude_capa )
			batt_info_alarm_set(chip, 2*G_WAKEUP_INTERVAL);
		else
			batt_info_alarm_set(chip, G_WAKEUP_INTERVAL);
	}

	//set irq wake here if registered
	
	return 0;
}

static int bq24157_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq24157_chip *chip = i2c_get_clientdata(client);

	pr_err("enter\n");

	wake_lock_timeout(&chip->batt_info_alarm_wlock, 5 * HZ);
	schedule_delayed_work(&chip->update_heartbeat_work, 2 * HZ);
	
	return 0;
}

static const struct dev_pm_ops bq24157_pm_ops = {
	.resume = bq24157_resume,
	.suspend = bq24157_suspend,
};

static struct of_device_id bq24157_match_table[] = {
	{.compatible = "yl,bq24157_charge",},
	{},
};

static const struct i2c_device_id bq24157_id[] = {
	{"bq24157_charge", 0},
	{},
};

static struct i2c_driver bq24157_driver = {
	.driver = {
		.name = "bq24157_charge",
		.owner = THIS_MODULE,
		.of_match_table = bq24157_match_table,
		.pm = &bq24157_pm_ops,
	},
	.probe = bq24157_probe,
	.remove = bq24157_remove,
	.id_table = bq24157_id,
};
module_i2c_driver(bq24157_driver);

MODULE_DESCRIPTION("BQ24157 charge IC driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:bq24157_charge");
