/*
 * Device driver for monitoring ambient light intensity in (lux)
 * proximity detection (prox), and Gesture functionality within the
 * AMS-TAOS TMG3992/3.
 *
 * Copyright (c) 2013, AMS-TAOS USA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/sensors/tmg399x.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

static u8 const tmg399x_ids[] = {
	0x9C,
	0x9E,
	0x9F,
};

static char const *tmg399x_names[] = {
	"tmg399x",
	"tmg399x",
	"tmg399x",
};

static u8 const restorable_regs[] = {
	TMG399X_ALS_TIME,
	TMG399X_WAIT_TIME,
	TMG399X_PERSISTENCE,
	TMG399X_CONFIG_1,
	TMG399X_PRX_PULSE,
	TMG399X_GAIN,
	TMG399X_CONFIG_2,
	TMG399X_PRX_OFFSET_NE,
	TMG399X_PRX_OFFSET_SW,
	TMG399X_CONFIG_3,
};

static u8 const prox_gains[] = {
	1,
	2,
	4,
	8
};

static u8 const als_gains[] = {
	1,
	4,
	16,
	64
};

static u8 const prox_pplens[] = {
	4,
	8,
	16,
	32
};

static u8 const led_drives[] = {
	100,
	50,
	25,
	12
};

static u16 const led_boosts[] = {
	100,
	150,
	200,
	300
};

static struct lux_segment segment_default[] = {
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
};

static struct tmg399x_parameters param_default = {
	.als_time = 0xEE, /* 50ms */
	.als_gain = AGAIN_16,
	.als_deltaP = 10,
	.wait_time = 0xFF, /* 2.78ms */
	.prox_th_min = 50,
	.prox_th_max = 80,
	.persist = PRX_PERSIST(1) | ALS_PERSIST(2),
	.als_prox_cfg1 = 0x60,
	.prox_pulse = PPLEN_4US | PRX_PULSE_CNT(8),
	.prox_gain = PGAIN_1,
	.ldrive = PDRIVE_100MA,
	.als_prox_cfg2 = LEDBOOST_300 | 0x01,
	.prox_offset_ne = 0,
	.prox_offset_sw = 0,
	.als_prox_cfg3 = 0,

	.ges_entry_th = 0,
	.ges_exit_th = 0,
	.ges_cfg1 = FIFOTH_1 | GEXPERS_2,
	.ges_cfg2 = GGAIN_4 | GLDRIVE_100 | GWTIME_6,
	.ges_offset_n = 0,
	.ges_offset_s = 0,
	.ges_pulse = GPLEN_16US | GES_PULSE_CNT(2),
	.ges_offset_w = 0,
	.ges_offset_e = 0,
	.ges_dimension = GBOTH_PAIR,
};

static const struct lux_segment tmg399x_segment[] = {
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
};

struct tmg399x_i2c_platform_data tmg399x_data = {
	.platform_power = NULL,
	.platform_init = NULL,
	.platform_teardown = NULL,
	.prox_name = "tmg399x_proximity",
	.als_name = "tmg399x_als",
	.parameters = {
		.als_time = 0xEE, /* 50ms */
		.als_gain = AGAIN_16,
		.als_deltaP = 10,
		.wait_time = 0xFF, /* 2.78ms */
		.prox_th_min = 50,
		.prox_th_max = 80,
		.persist = PRX_PERSIST(1) | ALS_PERSIST(2),
		.als_prox_cfg1 = 0x60,
		.prox_pulse = PPLEN_4US | PRX_PULSE_CNT(8),
		.prox_gain = PGAIN_1,
		.ldrive = PDRIVE_100MA,
		.als_prox_cfg2 = LEDBOOST_300 | 0x01,
		.prox_offset_ne = 0,
		.prox_offset_sw = 0,
		.als_prox_cfg3 = 0,
		
		.ges_entry_th = 0,
		.ges_exit_th = 0,
		.ges_cfg1 = FIFOTH_1 | GEXPERS_2,
		.ges_cfg2 = GGAIN_4 | GLDRIVE_100 | GWTIME_6,
		.ges_offset_n = 0,
		.ges_offset_s = 0,
		.ges_pulse = GPLEN_16US | GES_PULSE_CNT(16),
		.ges_offset_w = 0,
		.ges_offset_e = 0,
		.ges_dimension = GBOTH_PAIR,
	},
	.als_can_wake = false,
	.proximity_can_wake = true,
	.segment = (struct lux_segment *) tmg399x_segment,
	.segment_num = ARRAY_SIZE(tmg399x_segment),

};

static int tmg399x_i2c_read(struct tmg399x_chip *chip, u8 reg, u8 *val)
{
	int ret;

	s32 read;
	struct i2c_client *client = chip->client;

	reg += I2C_ADDR_OFFSET;
	ret = i2c_smbus_write_byte(client, reg);
	if (ret < 0) {
		mdelay(3);
		ret = i2c_smbus_write_byte(client, reg);
		if (ret < 0) {
			dev_err(&client->dev, "%s: failed 2x to write register %x\n",
				__func__, reg);
		return ret;
		}
	}

	read = i2c_smbus_read_byte(client);
	if (read < 0) {
		mdelay(3);
		read = i2c_smbus_read_byte(client);
		if (read < 0) {
			dev_err(&client->dev, "%s: failed read from register %x\n",
				__func__, reg);
		}
		return ret;
	}

	*val = (u8)read;
	return 0;
}

static int tmg399x_i2c_write(struct tmg399x_chip *chip, u8 reg, u8 val)
{
	int ret;
	struct i2c_client *client = chip->client;

	reg += I2C_ADDR_OFFSET;
	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		mdelay(3);
		ret = i2c_smbus_write_byte_data(client, reg, val);
		if (ret < 0) {
			dev_err(&client->dev, "%s: failed to write register %x err= %d\n",
				__func__, reg, ret);
		}
	}
	return ret;
}

#if 0
static int tmg399x_i2c_modify(struct tmg399x_chip *chip, u8 reg, u8 mask, u8 val)
{
	int ret;
	u8 temp;

	ret = tmg399x_i2c_read(chip, reg, &temp);
	temp &= ~mask;
	temp |= val;
	ret |= tmg399x_i2c_write(chip, reg, temp);

	return ret;
}
#endif

static int tmg399x_i2c_reg_blk_write(struct tmg399x_chip *chip,
		u8 reg, u8 *val, int size)
{
	s32 ret;
	struct i2c_client *client = chip->client;

	reg += I2C_ADDR_OFFSET;
	ret =  i2c_smbus_write_i2c_block_data(client,
			reg, size, val);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed 2X at address %x (%d bytes)\n",
				__func__, reg, size);
	}

	return ret;
}

static int tmg399x_i2c_ram_blk_read(struct tmg399x_chip *chip,
		u8 reg, u8 *val, int size)
{
	s32 ret;
	struct i2c_client *client = chip->client;

	reg += I2C_ADDR_OFFSET;
	ret =  i2c_smbus_read_i2c_block_data(client,
			reg, size, val);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed 2X at address %x (%d bytes)\n",
				__func__, reg, size);
	}

	return ret;
}

#if 0
static int tmg399x_i2c_ram_blk_write(struct tmg399x_chip *chip,
		u8 reg, u8 *val, int size)
{
	s32 ret;
	struct i2c_client *client = chip->client;
	ret =  i2c_smbus_write_i2c_block_data(client,
			reg, size, val);
	if (ret < 0)
		dev_err(&client->dev, "%s: failed at address %x (%d bytes)\n",
				__func__, reg, size);
	return ret;
}
#endif

static int tmg399x_flush_regs(struct tmg399x_chip *chip)
{
	unsigned i;
	int ret;
	u8 reg;

	dev_info(&chip->client->dev, "%s\n", __func__);

	for (i = 0; i < ARRAY_SIZE(restorable_regs); i++) {
		reg = restorable_regs[i];
		ret = tmg399x_i2c_write(chip, reg, chip->shadow[reg]);
		if (ret < 0) {
			dev_err(&chip->client->dev, "%s: err on reg 0x%02x\n",
					__func__, reg);
			break;
		}
	}

	return ret;
}

static int tmg399x_irq_clr(struct tmg399x_chip *chip, u8 int2clr)
{
	int ret, ret2;

	ret = i2c_smbus_write_byte(chip->client, int2clr);
	if (ret < 0) {
		mdelay(3);
		ret2 = i2c_smbus_write_byte(chip->client, int2clr);
		if (ret2 < 0) {
			dev_err(&chip->client->dev, "%s: failed 2x, int to clr %02x\n",
					__func__, int2clr);
		}
		return ret2;
	}

	return ret;
}

static int tmg399x_update_enable_reg(struct tmg399x_chip *chip)
{
	int ret;
	
	ret = tmg399x_i2c_write(chip, TMG399X_CONTROL,
			chip->shadow[TMG399X_CONTROL]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_CFG_4,
			chip->shadow[TMG399X_GES_CFG_4]);
	
	return ret;
}

static int tmg399x_set_als_gain(struct tmg399x_chip *chip, int gain)
{
	int ret;
	u8 ctrl_reg  = chip->shadow[TMG399X_GAIN] & ~TMG399X_ALS_GAIN_MASK;

	switch (gain) {
	case 1:
		ctrl_reg |= AGAIN_1;
		break;
	case 4:
		ctrl_reg |= AGAIN_4;
		break;
	case 16:
		ctrl_reg |= AGAIN_16;
		break;
	case 64:
		ctrl_reg |= AGAIN_64;
		break;
	default:
		break;
	}

	ret = tmg399x_i2c_write(chip, TMG399X_GAIN, ctrl_reg);
	if (!ret) {
		chip->shadow[TMG399X_GAIN] = ctrl_reg;
		chip->params.als_gain = ctrl_reg & TMG399X_ALS_GAIN_MASK;
	}
	return ret;
}

static void tmg399x_calc_cpl(struct tmg399x_chip *chip)
{
	u32 cpl;
	u32 sat;
	u8 atime = chip->shadow[TMG399X_ALS_TIME];

	cpl = 256 - chip->shadow[TMG399X_ALS_TIME];
	cpl *= TMG399X_ATIME_PER_100;
	cpl /= 100;
	cpl *= als_gains[chip->params.als_gain];

	sat = min_t(u32, MAX_ALS_VALUE, (u32)(256 - atime) << 10);
	sat = sat * 8 / 10;
	chip->als_inf.cpl = cpl;
	chip->als_inf.saturation = sat;
}

static int tmg399x_get_lux(struct tmg399x_chip *chip)
{
	u32 rp1, gp1, bp1, cp1;
	u32 lux = 0;
	u32 cct;
	u32 sat;
	u32 sf;

	/* use time in ms get scaling factor */
	tmg399x_calc_cpl(chip);
	sat = chip->als_inf.saturation;

	if (!chip->als_gain_auto) {
		if (chip->als_inf.clear_raw <= MIN_ALS_VALUE) {
			dev_info(&chip->client->dev,
				"%s: darkness\n", __func__);
			lux = 0;
			goto exit;
		} else if (chip->als_inf.clear_raw >= sat) {
			dev_info(&chip->client->dev,
				"%s: saturation, keep lux & cct\n", __func__);
			lux = chip->als_inf.lux;
			goto exit;
		}
	} else {
		u8 gain = als_gains[chip->params.als_gain];
		int ret = -EIO;

		if (gain == 16 && chip->als_inf.clear_raw >= sat) {
			ret = tmg399x_set_als_gain(chip, 1);
		} else if (gain == 16 &&
			chip->als_inf.clear_raw < GAIN_SWITCH_LEVEL) {
			ret = tmg399x_set_als_gain(chip, 64);
		} else if ((gain == 64 &&
			chip->als_inf.clear_raw >= (sat - GAIN_SWITCH_LEVEL)) ||
			(gain == 1 && chip->als_inf.clear_raw > GAIN_SWITCH_LEVEL)) {
			ret = tmg399x_set_als_gain(chip, 16);
		}
		if (!ret) {
			dev_info(&chip->client->dev, "%s: gain adjusted, skip\n",
					__func__);
			tmg399x_calc_cpl(chip);
			ret = -EAGAIN;
			lux = chip->als_inf.lux;
			goto exit;
		}

		if (chip->als_inf.clear_raw <= MIN_ALS_VALUE) {
			dev_info(&chip->client->dev,
					"%s: darkness\n", __func__);
			lux = 0;
			goto exit;
		} else if (chip->als_inf.clear_raw >= sat) {
			dev_info(&chip->client->dev, "%s: saturation, keep lux\n",
					__func__);
			lux = chip->als_inf.lux;
			goto exit;
		}
	}

	/* remove ir from counts*/
	rp1 = chip->als_inf.red_raw - chip->als_inf.ir;
	gp1 = chip->als_inf.green_raw - chip->als_inf.ir;
	bp1 = chip->als_inf.blue_raw - chip->als_inf.ir;
	cp1 = chip->als_inf.clear_raw - chip->als_inf.ir;

	if (!chip->als_inf.cpl) {
		dev_info(&chip->client->dev, "%s: zero cpl. Setting to 1\n",
				__func__);
		chip->als_inf.cpl = 1;
	}

	if (chip->als_inf.red_raw > chip->als_inf.ir)
		lux += chip->segment[chip->device_index].r_coef * rp1;
	else
		dev_err(&chip->client->dev, "%s: lux rp1 = %d\n",
			__func__,
			(chip->segment[chip->device_index].r_coef * rp1));

	if (chip->als_inf.green_raw > chip->als_inf.ir)
		lux += chip->segment[chip->device_index].g_coef * gp1;
	else
		dev_err(&chip->client->dev, "%s: lux gp1 = %d\n",
			__func__,
			(chip->segment[chip->device_index].g_coef * rp1));

	if (chip->als_inf.blue_raw > chip->als_inf.ir)
		lux -= chip->segment[chip->device_index].b_coef * bp1;
	else
		dev_err(&chip->client->dev, "%s: lux bp1 = %d\n",
			__func__,
			(chip->segment[chip->device_index].b_coef * rp1));

	sf = chip->als_inf.cpl;

	if (sf > 131072)
		goto error;

	lux /= sf;
	lux *= chip->segment[chip->device_index].d_factor;
	lux += 512;
	lux >>= 10;
	chip->als_inf.lux = (u16) lux;

	cct = ((chip->segment[chip->device_index].ct_coef * bp1) / rp1) +
		chip->segment[chip->device_index].ct_offset;

	chip->als_inf.cct = (u16) cct;

exit:
	return 0;

error:
	dev_err(&chip->client->dev, "ERROR Scale factor = %d", sf);

	return 1;
}

static void tmg399x_get_ges(struct tmg399x_chip *chip)
{
	int ret;
	u8 gstatus;
	u8 numofdset;
	u8 buffer[128];

	ret = tmg399x_i2c_read(chip, TMG399X_GES_STAT, &gstatus);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s: failed to read gesture status\n",
			__func__);
		return;
	}
	if ((gstatus & TMG399X_GES_ST_MASK) == TMG399X_GES_VALID) {
		while(1) {
			ret = tmg399x_i2c_read(chip, TMG399X_GES_FLVL, &numofdset);
			if (ret < 0) {
				dev_err(&chip->client->dev,
						"%s: failed to read gesture fifo level\n",
						__func__);
				return;
			}

			if (numofdset == 0) {
				//printk (KERN_INFO "fifo empty!\n");
				break;
			}
			ret = tmg399x_i2c_ram_blk_read(chip, TMG399X_GES_NFIFO,
				buffer, numofdset*4);
			if (ret >= 0) {
				chip->ges_raw_data.north = buffer[(numofdset - 1) * 4];
				chip->ges_raw_data.south = buffer[(numofdset - 1) * 4 + 1];
				chip->ges_raw_data.west = buffer[(numofdset - 1) * 4 + 2];
				chip->ges_raw_data.east = buffer[(numofdset - 1) * 4 + 3];
				printk("north:%d south:%d west:%d east:%d\n", 
						chip->ges_raw_data.north, chip->ges_raw_data.south, chip->ges_raw_data.west, 
						chip->ges_raw_data.east);
			}
		}
	} else {
		printk(KERN_INFO "overflow %d\n", gstatus);
	}
}

static void tmg399x_get_prox(struct tmg399x_chip *chip)
{
	int ret;
	
	ret = tmg399x_i2c_read(chip, TMG399X_PRX_CHAN,
			&chip->shadow[TMG399X_PRX_CHAN]);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s: failed to read proximity raw data\n",
			__func__);
		return;
	} else
		chip->prx_inf.raw = chip->shadow[TMG399X_PRX_CHAN];

	if (chip->prx_inf.detected == false) {
		if (chip->prx_inf.raw >= chip->shadow[TMG399X_PRX_MAXTHRESHHI]) {
			chip->prx_inf.detected = true;
			chip->shadow[TMG399X_PRX_MINTHRESHLO] =
					chip->pdata->parameters.prox_th_min;
			chip->shadow[TMG399X_PRX_MAXTHRESHHI] = 0xff;
			ret = tmg399x_i2c_write(chip, TMG399X_PRX_MINTHRESHLO,
					chip->shadow[TMG399X_PRX_MINTHRESHLO]);
			ret |= tmg399x_i2c_write(chip, TMG399X_PRX_MAXTHRESHHI,
					chip->shadow[TMG399X_PRX_MAXTHRESHHI]);
			if (ret < 0)
				dev_err(&chip->client->dev,
					"%s: failed to write proximity threshold\n",
					__func__);
		}
	} else {
		if (chip->prx_inf.raw <= chip->shadow[TMG399X_PRX_MINTHRESHLO]) {
			chip->prx_inf.detected = false;
			chip->shadow[TMG399X_PRX_MINTHRESHLO] = 0x00;
			chip->shadow[TMG399X_PRX_MAXTHRESHHI] =
					chip->pdata->parameters.prox_th_max;
			ret = tmg399x_i2c_write(chip, TMG399X_PRX_MINTHRESHLO,
					chip->shadow[TMG399X_PRX_MINTHRESHLO]);
			ret |= tmg399x_i2c_write(chip, TMG399X_PRX_MAXTHRESHHI,
					chip->shadow[TMG399X_PRX_MAXTHRESHHI]);
			if (ret < 0)
				dev_err(&chip->client->dev,
					"%s: failed to write proximity threshold\n",
					__func__);
		}
	}

}

static int tmg399x_read_rgb(struct tmg399x_chip *chip)
{
	int ret;

	ret = tmg399x_i2c_read(chip, TMG399X_CLR_CHANLO,
			&chip->shadow[TMG399X_CLR_CHANLO]);
	ret |= tmg399x_i2c_read(chip, TMG399X_CLR_CHANHI,
			&chip->shadow[TMG399X_CLR_CHANHI]);

	ret |= tmg399x_i2c_read(chip, TMG399X_RED_CHANLO,
			&chip->shadow[TMG399X_RED_CHANLO]);
	ret |= tmg399x_i2c_read(chip, TMG399X_RED_CHANHI,
			&chip->shadow[TMG399X_RED_CHANHI]);

	ret |= tmg399x_i2c_read(chip, TMG399X_GRN_CHANLO,
			&chip->shadow[TMG399X_GRN_CHANLO]);
	ret |= tmg399x_i2c_read(chip, TMG399X_GRN_CHANHI,
			&chip->shadow[TMG399X_GRN_CHANHI]);

	ret |= tmg399x_i2c_read(chip, TMG399X_BLU_CHANLO,
			&chip->shadow[TMG399X_BLU_CHANLO]);
	ret |= tmg399x_i2c_read(chip, TMG399X_BLU_CHANHI,
			&chip->shadow[TMG399X_BLU_CHANHI]);

	return (ret < 0) ? ret : 0;
}

static void tmg399x_get_als(struct tmg399x_chip *chip)
{
	int ret;

	ret = tmg399x_read_rgb(chip);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s: failed to read rgb raw data\n",
			__func__);
		return;
	} else {
		u8 *buf = &chip->shadow[TMG399X_CLR_CHANLO];
		/* extract raw channel data */
		chip->als_inf.clear_raw = le16_to_cpup((const __le16 *)&buf[0]);
		chip->als_inf.red_raw = le16_to_cpup((const __le16 *)&buf[2]);
		chip->als_inf.green_raw = le16_to_cpup((const __le16 *)&buf[4]);
		chip->als_inf.blue_raw = le16_to_cpup((const __le16 *)&buf[6]);
		chip->als_inf.ir =
			(chip->als_inf.red_raw + chip->als_inf.green_raw +
			chip->als_inf.blue_raw - chip->als_inf.clear_raw + 1) >> 1;
		if (chip->als_inf.ir < 0)
			chip->als_inf.ir = 0;
	}
}

static int tmg399x_update_als_thres(struct tmg399x_chip *chip, bool on_enable)
{
	s32 ret;
	u8 *buf = &chip->shadow[TMG399X_ALS_MINTHRESHLO];
	u16 deltaP = chip->params.als_deltaP;
	u16 from, to, cur;
	u16 saturation = chip->als_inf.saturation;

	cur = chip->als_inf.clear_raw;

	if (on_enable) {
		/* move deltaP far away from current position to force an irq */
		from = to = cur > saturation / 2 ? 0 : saturation;
	} else {
		deltaP = cur * deltaP / 100;
		if (!deltaP)
			deltaP = 1;

		if (cur > deltaP)
			from = cur - deltaP;
		else
			from = 0;

		if (cur < (saturation - deltaP))
			to = cur + deltaP;
		else
			to = saturation;

	}

	*buf++ = from & 0xff;
	*buf++ = from >> 8;
	*buf++ = to & 0xff;
	*buf++ = to >> 8;
	ret = tmg399x_i2c_reg_blk_write(chip, TMG399X_ALS_MINTHRESHLO,
			&chip->shadow[TMG399X_ALS_MINTHRESHLO],
			TMG399X_ALS_MAXTHRESHHI - TMG399X_ALS_MINTHRESHLO + 1);

	return (ret < 0) ? ret : 0;
}

static int tmg399x_ges_init(struct tmg399x_chip *chip, int on)
{
	int ret;
	
	if (on) {
		if (chip->pdata) {
			chip->params.ges_entry_th = chip->pdata->parameters.ges_entry_th;
			chip->params.ges_exit_th = chip->pdata->parameters.ges_exit_th;
			chip->params.ges_cfg1 = chip->pdata->parameters.ges_cfg1;
			chip->params.ges_cfg2 = chip->pdata->parameters.ges_cfg2;
			chip->params.ges_offset_n = chip->pdata->parameters.ges_offset_n;
			chip->params.ges_offset_s = chip->pdata->parameters.ges_offset_s;
			chip->params.ges_pulse = chip->pdata->parameters.ges_pulse;
			chip->params.ges_offset_w = chip->pdata->parameters.ges_offset_w;
			chip->params.ges_offset_e = chip->pdata->parameters.ges_offset_e;
			chip->params.ges_dimension = chip->pdata->parameters.ges_dimension;
		} else {
			chip->params.ges_entry_th = param_default.ges_entry_th;
			chip->params.ges_exit_th = param_default.ges_exit_th;
			chip->params.ges_cfg1 = param_default.ges_cfg1;
			chip->params.ges_cfg2 = param_default.ges_cfg2;
			chip->params.ges_offset_n = param_default.ges_offset_n;
			chip->params.ges_offset_s = param_default.ges_offset_s;
			chip->params.ges_pulse = param_default.ges_pulse;
			chip->params.ges_offset_w = param_default.ges_offset_w;
			chip->params.ges_offset_e = param_default.ges_offset_e;
			chip->params.ges_dimension = param_default.ges_dimension;
		}
	} else {
		chip->params.ges_entry_th = 0;
		chip->params.ges_exit_th = 0;
		chip->params.ges_cfg1 = 0;
		chip->params.ges_cfg2 = 0;
		chip->params.ges_offset_n = 0;
		chip->params.ges_offset_s = 0;
		chip->params.ges_pulse = 0;
		chip->params.ges_offset_w = 0;
		chip->params.ges_offset_e = 0;
		chip->params.ges_dimension = 0;	
	}

	/* Initial gesture registers */
	chip->shadow[TMG399X_GES_ENTH]  = chip->params.ges_entry_th;
	chip->shadow[TMG399X_GES_EXTH]  = chip->params.ges_exit_th;
	chip->shadow[TMG399X_GES_CFG_1] = chip->params.ges_cfg1;
	chip->shadow[TMG399X_GES_CFG_2] = chip->params.ges_cfg2;
	chip->shadow[TMG399X_GES_OFFSET_N] = chip->params.ges_offset_n;
	chip->shadow[TMG399X_GES_OFFSET_S] = chip->params.ges_offset_s;
	chip->shadow[TMG399X_GES_PULSE] = chip->params.ges_pulse;
	chip->shadow[TMG399X_GES_OFFSET_W] = chip->params.ges_offset_w;
	chip->shadow[TMG399X_GES_OFFSET_E] = chip->params.ges_offset_e;
	chip->shadow[TMG399X_GES_CFG_3] = chip->params.ges_dimension;
	
	ret = tmg399x_i2c_write(chip, TMG399X_GES_ENTH,
			chip->shadow[TMG399X_GES_ENTH]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_EXTH,
			chip->shadow[TMG399X_GES_EXTH]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_CFG_1,
			chip->shadow[TMG399X_GES_CFG_1]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_CFG_2,
			chip->shadow[TMG399X_GES_CFG_2]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_N,
			chip->shadow[TMG399X_GES_OFFSET_N]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_S,
			chip->shadow[TMG399X_GES_OFFSET_S]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_PULSE,
			chip->shadow[TMG399X_GES_PULSE]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_W,
			chip->shadow[TMG399X_GES_OFFSET_W]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_E,
			chip->shadow[TMG399X_GES_OFFSET_E]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_CFG_3,
			chip->shadow[TMG399X_GES_CFG_3]);

	return ret;
}

static int tmg399x_ges_enable(struct tmg399x_chip *chip, int on)
{
	int ret = 0;

	dev_info(&chip->client->dev, "%s: on = %d\n", __func__, on);
	if (on) {
		//ret = tmg399x_i2c_modify(chip, TMG399X_GES_CFG_4,
		//		TMG399X_GES_INT_CLR, TMG399X_GES_INT_CLR);
		/* initialize */		
		ret |= tmg399x_ges_init(chip, 1);		
		if (ret < 0)
			return ret;

		chip->shadow[TMG399X_CONTROL] |= (TMG399X_EN_PWR_ON |
			TMG399X_EN_PRX | TMG399X_EN_GES);
		chip->shadow[TMG399X_GES_CFG_4] |= (TMG399X_GES_EN_IRQ |
			TMG399X_GES_MODE);
		ret |= tmg399x_update_enable_reg(chip);
		
		mdelay(3);
	} else {
		chip->shadow[TMG399X_CONTROL] &= ~TMG399X_EN_GES;
		chip->shadow[TMG399X_GES_CFG_4] &= ~ (TMG399X_GES_EN_IRQ |
			TMG399X_GES_MODE);
		if (!chip->prx_enabled)
			chip->shadow[TMG399X_CONTROL] &= ~TMG399X_EN_PRX;
		if (!chip->prx_enabled && !chip->als_enabled)
			chip->shadow[TMG399X_CONTROL] &= ~TMG399X_EN_PWR_ON;
		ret = tmg399x_update_enable_reg(chip);

		/* deinitialize */		
		ret |= tmg399x_ges_init(chip, 0);
		//ret |= tmg399x_i2c_modify(chip, TMG399X_GES_CFG_4,
		//		TMG399X_GES_INT_CLR, TMG399X_GES_INT_CLR);
		if (ret < 0)
			return ret;
	}
	if (!ret)
		chip->ges_enabled = on;
	return ret;
}

static int tmg399x_prox_enable(struct tmg399x_chip *chip, int on)
{
	int ret;

	dev_info(&chip->client->dev, "%s: on = %d\n", __func__, on);
	if (on) {
		tmg399x_irq_clr(chip, TMG399X_CMD_PROX_INT_CLR);
		chip->shadow[TMG399X_CONTROL] |= (TMG399X_EN_PWR_ON |
			TMG399X_EN_PRX | TMG399X_EN_PRX_IRQ);
		ret = tmg399x_update_enable_reg(chip);
		if (ret < 0)
			return ret;
		mdelay(3);
	} else {
		chip->shadow[TMG399X_CONTROL] &= ~TMG399X_EN_PRX_IRQ;
		if (!chip->ges_enabled)
			chip->shadow[TMG399X_CONTROL] &= ~TMG399X_EN_PRX;
		if (!chip->ges_enabled && !chip->als_enabled)
			chip->shadow[TMG399X_CONTROL] &= ~TMG399X_EN_PWR_ON;
		ret = tmg399x_update_enable_reg(chip);
		if (ret < 0)
			return ret;
		tmg399x_irq_clr(chip, TMG399X_CMD_PROX_INT_CLR);
	}
	if (!ret)
		chip->prx_enabled = on;
	return ret;
}

static int tmg399x_als_enable(struct tmg399x_chip *chip, int on)
{
	int ret;

	dev_info(&chip->client->dev, "%s: on = %d\n", __func__, on);
	if (on) {
		tmg399x_irq_clr(chip, TMG399X_CMD_ALS_INT_CLR);
		tmg399x_update_als_thres(chip, 1);
		chip->shadow[TMG399X_CONTROL] |=
				(TMG399X_EN_PWR_ON | TMG399X_EN_ALS |
				TMG399X_EN_ALS_IRQ);

		ret = tmg399x_update_enable_reg(chip);
		if (ret < 0)
			return ret;
		mdelay(3);
	} else {
		chip->shadow[TMG399X_CONTROL] &=
			~(TMG399X_EN_ALS | TMG399X_EN_ALS_IRQ);

		if (!chip->ges_enabled && !chip->prx_enabled)
			chip->shadow[TMG399X_CONTROL] &= ~TMG399X_EN_PWR_ON;
		ret = tmg399x_update_enable_reg(chip);
		if (ret < 0)
			return ret;
		tmg399x_irq_clr(chip, TMG399X_CMD_ALS_INT_CLR);
	}
	if (!ret)
		chip->als_enabled = on;

	return ret;
}

static int tmg399x_wait_enable(struct tmg399x_chip *chip, int on)
{
	int ret;

	dev_info(&chip->client->dev, "%s: on = %d\n", __func__, on);
	if (on) {
		chip->shadow[TMG399X_CONTROL] |= TMG399X_EN_WAIT;

		ret = tmg399x_update_enable_reg(chip);
		if (ret < 0)
			return ret;
		mdelay(3);
	} else {
		chip->shadow[TMG399X_CONTROL] &= ~TMG399X_EN_WAIT;

		ret = tmg399x_update_enable_reg(chip);
		if (ret < 0)
			return ret;
	}
	if (!ret)
	        chip->wait_enabled = on;

	return ret;
}

static ssize_t tmg399x_ges_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->ges_enabled);
}

static ssize_t tmg399x_ges_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	mutex_lock(&chip->lock);
	if (strtobool(buf, &value)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (value)
		tmg399x_ges_enable(chip, 1);
	else
		tmg399x_ges_enable(chip, 0);

	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_prox_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_enabled);
}

static ssize_t tmg399x_prox_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	mutex_lock(&chip->lock);
	if (strtobool(buf, &value)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (value)
		tmg399x_prox_enable(chip, 1);
	else
		tmg399x_prox_enable(chip, 0);

	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_als_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_enabled);
}

static ssize_t tmg399x_als_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	mutex_lock(&chip->lock);
	if (strtobool(buf, &value)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (value)
		tmg399x_als_enable(chip, 1);
	else
		tmg399x_als_enable(chip, 0);

	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_wait_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->wait_enabled);
}

static ssize_t tmg399x_wait_enable_store(struct device *dev,
                                struct device_attribute *attr,
                                const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	mutex_lock(&chip->lock);
	if (strtobool(buf, &value)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (value)
		tmg399x_wait_enable(chip, 1);
	else
		tmg399x_wait_enable(chip, 0);

		mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_als_itime_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int t;
	t = 256 - chip->params.als_time;
	t *= TMG399X_ATIME_PER_100;
	t /= 100;
	return snprintf(buf, PAGE_SIZE, "%d (in ms)\n", t);
}

static ssize_t tmg399x_als_itime_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long itime;
	int ret;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &itime);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	if (itime > 712 || itime < 3) {
		dev_err(&chip->client->dev,
			"als integration time range [3,712]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	itime *= 100;
	itime /= TMG399X_ATIME_PER_100;
	itime = (256 - itime);
	chip->shadow[TMG399X_ALS_TIME] = (u8)itime;
	chip->params.als_time = (u8)itime;
	tmg399x_i2c_write(chip, TMG399X_ALS_TIME,
		chip->shadow[TMG399X_ALS_TIME]);
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_wait_time_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int t;
	t = 256 - chip->params.wait_time;
	t *= TMG399X_ATIME_PER_100;
	t /= 100;
	if (chip->params.als_prox_cfg1 & WLONG)
		t *= 12;
	return snprintf(buf, PAGE_SIZE, "%d (in ms)\n", t);
}

static ssize_t tmg399x_wait_time_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long time;
	int ret;
	u8 cfg1;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &time);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	if (time > 8540 || time < 3) {
		dev_err(&chip->client->dev,
			"wait time range [3,8540]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	
	cfg1 = chip->shadow[TMG399X_CONFIG_1] & ~0x02;
	if (time > 712) {
		cfg1 |= WLONG;
		time /= 12;
	}

	time *= 100;
	time /= TMG399X_ATIME_PER_100;
	time = (256 - time);
	chip->shadow[TMG399X_WAIT_TIME] = (u8)time;
	chip->params.wait_time = (u8)time;
	chip->shadow[TMG399X_CONFIG_1] = cfg1;
	chip->params.als_prox_cfg1 = cfg1;
	tmg399x_i2c_write(chip, TMG399X_WAIT_TIME,
		chip->shadow[TMG399X_WAIT_TIME]);
	tmg399x_i2c_write(chip, TMG399X_CONFIG_1,
		chip->shadow[TMG399X_CONFIG_1]);
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_prox_persist_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			(chip->params.persist & 0xF0) >> 4);
}

static ssize_t tmg399x_prox_persist_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long persist;
	int ret;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &persist);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	
	if (persist > 15) {
		dev_err(&chip->client->dev,
			"prox persistence range [0,15]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	
	chip->shadow[TMG399X_PERSISTENCE] &= 0x0F;
	chip->shadow[TMG399X_PERSISTENCE] |= (((u8)persist << 4) & 0xF0);
	chip->params.persist = chip->shadow[TMG399X_PERSISTENCE];
	tmg399x_i2c_write(chip, TMG399X_PERSISTENCE,
		chip->shadow[TMG399X_PERSISTENCE]);
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_als_persist_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			(chip->params.persist & 0x0F));
}

static ssize_t tmg399x_als_persist_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long persist;
	int ret;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &persist);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (persist > 15) {
		dev_err(&chip->client->dev,
			"als persistence range [0,15]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	chip->shadow[TMG399X_PERSISTENCE] &= 0xF0;
	chip->shadow[TMG399X_PERSISTENCE] |= ((u8)persist & 0x0F);
	chip->params.persist = chip->shadow[TMG399X_PERSISTENCE];
	tmg399x_i2c_write(chip, TMG399X_PERSISTENCE,
		chip->shadow[TMG399X_PERSISTENCE]);
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_prox_pulse_len_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int i = (chip->params.prox_pulse & 0xC0) >> 6;
	return snprintf(buf, PAGE_SIZE, "%duS\n", prox_pplens[i]);
}

static ssize_t tmg399x_prox_pulse_len_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long length;
	int ret;
	u8 ppulse;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &length);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (length != 4 && length != 8 &&
		length != 16 &&	length != 32) {
		dev_err(&chip->client->dev, 
			"pulse length set: {4, 8, 16, 32}\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	
	ppulse = chip->shadow[TMG399X_PRX_PULSE] & 0x3F;
	switch (length){
	case 4:
		ppulse |= PPLEN_4US;
		break;
	case 8:
		ppulse |= PPLEN_8US;
		break;
	case 16:
		ppulse |= PPLEN_16US;
		break;
	case 32:
		ppulse |= PPLEN_32US;
		break;
	default:
		break;
	}
	chip->shadow[TMG399X_PRX_PULSE] = ppulse;
	chip->params.prox_pulse = ppulse;
	tmg399x_i2c_write(chip, TMG399X_PRX_PULSE,
		chip->shadow[TMG399X_PRX_PULSE]);
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_prox_pulse_cnt_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			(chip->params.prox_pulse & 0x3F) + 1);
}

static ssize_t tmg399x_prox_pulse_cnt_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long count;
	int ret;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &count);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (count > 32 || count == 0) {
		dev_err(&chip->client->dev,
			"prox pulse count range [1,32]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	count -= 1;

	chip->shadow[TMG399X_PRX_PULSE] &= 0xC0;
	chip->shadow[TMG399X_PRX_PULSE] |= ((u8)count & 0x3F);
	chip->params.prox_pulse = chip->shadow[TMG399X_PRX_PULSE];
	tmg399x_i2c_write(chip, TMG399X_PRX_PULSE,
		chip->shadow[TMG399X_PRX_PULSE]);
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_prox_gain_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int i = chip->params.prox_gain >> 2;
	return snprintf(buf, PAGE_SIZE, "%d\n", prox_gains[i]);
}

static ssize_t tmg399x_prox_gain_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long gain;
	int ret;
	u8 ctrl_reg, cfg1_reg;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &gain);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (gain != 1 && gain != 2 && gain != 4 && gain != 8) {
		dev_err(&chip->client->dev,
			"prox gain set: {1, 2, 4, 8}\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	ctrl_reg = chip->shadow[TMG399X_GAIN] & ~TMG399X_PRX_GAIN_MASK;
	cfg1_reg = chip->shadow[TMG399X_CONFIG_1] & ~0x80;
	switch (gain){
	case 1:
		ctrl_reg |= PGAIN_1;
		break;
	case 2:
		ctrl_reg |= PGAIN_2;
		break;
	case 4:
		ctrl_reg |= PGAIN_4;
		break;
	case 8:
		ctrl_reg |= PGAIN_8;
		break;
	default:
		break;
	}

	ret = tmg399x_i2c_write(chip, TMG399X_GAIN, ctrl_reg);
	ret |= tmg399x_i2c_write(chip, TMG399X_CONFIG_1, cfg1_reg);
	if (!ret) {
		chip->shadow[TMG399X_GAIN] = ctrl_reg;
		chip->params.prox_gain = ctrl_reg & TMG399X_PRX_GAIN_MASK;
		chip->shadow[TMG399X_CONFIG_1] = cfg1_reg;
		chip->params.als_prox_cfg1 = cfg1_reg;
	}
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_led_drive_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%dmA\n",
			led_drives[chip->params.ldrive]);
}

static ssize_t tmg399x_led_drive_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long ldrive;
	int ret;
	u8 ctrl_reg;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &ldrive);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (ldrive != 100 && ldrive != 50 &&
		ldrive != 25 && ldrive != 12) {
		dev_err(&chip->client->dev,
			"led drive set: {100, 50, 25, 12}\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	ctrl_reg = chip->shadow[TMG399X_GAIN] & ~TMG399X_LDRIVE_MASK;
	switch (ldrive){
	case 100:
		ctrl_reg |= PDRIVE_100MA;
		chip->params.ldrive = 0;
		break;
	case 50:
		ctrl_reg |= PDRIVE_50MA;
		chip->params.ldrive = 1;
		break;
	case 25:
		ctrl_reg |= PDRIVE_25MA;
		chip->params.ldrive = 2;
		break;
	case 12:
		ctrl_reg |= PDRIVE_12MA;
		chip->params.ldrive = 3;
		break;
	default:
		break;
	}
	chip->shadow[TMG399X_GAIN] = ctrl_reg;
	tmg399x_i2c_write(chip, TMG399X_GAIN, chip->shadow[TMG399X_GAIN]);
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_als_gain_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d (%s)\n",
			als_gains[chip->params.als_gain],
			chip->als_gain_auto ? "auto" : "manual");
}

static ssize_t tmg399x_als_gain_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long gain;
	int i = 0;
	int ret;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &gain);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
		
	if (gain != 0 && gain != 1 && gain != 4 &&
		gain != 16 && gain != 64) {
		dev_err(&chip->client->dev,
			"als gain set: {0(auto), 1, 4, 16, 64}\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	while (i < sizeof(als_gains)) {
		if (gain == als_gains[i])
			break;
		i++;
	}

	if (gain) {
		chip->als_gain_auto = false;
		ret = tmg399x_set_als_gain(chip, als_gains[i]);
		if (!ret)
			tmg399x_calc_cpl(chip);
	} else {
		chip->als_gain_auto = true;
	}
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_led_boost_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int i = (chip->params.als_prox_cfg2 & 0x30) >> 4;
	return snprintf(buf, PAGE_SIZE, "%d percents\n", led_boosts[i]);
}

static ssize_t tmg399x_led_boost_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long lboost;
	int ret;
	u8 cfg2;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &lboost);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (lboost != 100 && lboost != 150 &&
		lboost != 200 && lboost != 300) {
		dev_err(&chip->client->dev,
			"led boost set: {100, 150, 200, 300}\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	cfg2 = chip->shadow[TMG399X_CONFIG_2] & ~0x30;
	switch (lboost){
	case 100:
		cfg2 |= LEDBOOST_100;
		break;
	case 150:
		cfg2 |= LEDBOOST_150;
		break;
	case 200:
		cfg2 |= LEDBOOST_200;
		break;
	case 300:
		cfg2 |= LEDBOOST_300;
		break;
	default:
		break;
	}
	chip->shadow[TMG399X_CONFIG_2] = cfg2;
	chip->params.als_prox_cfg2 = cfg2;
	tmg399x_i2c_write(chip, TMG399X_CONFIG_2,
		chip->shadow[TMG399X_CONFIG_2]);
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_sat_irq_en_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			(chip->params.als_prox_cfg2 & 0x80) >> 7);
}

static ssize_t tmg399x_sat_irq_en_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool psien;

	mutex_lock(&chip->lock);
	if (strtobool(buf, &psien)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	chip->shadow[TMG399X_CONFIG_2] &= 0x7F;
	if (psien)
		chip->shadow[TMG399X_CONFIG_2] |= PSIEN;
	chip->params.als_prox_cfg2 = chip->shadow[TMG399X_CONFIG_2];
	tmg399x_i2c_write(chip, TMG399X_CONFIG_2,
		chip->shadow[TMG399X_CONFIG_2]);
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_prox_offset_ne_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			chip->params.prox_offset_ne);
}

static ssize_t tmg399x_prox_offset_ne_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	long offset_ne;
	int ret;
	u8 offset = 0;

	mutex_lock(&chip->lock);
	ret = kstrtol(buf, 10, &offset_ne);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (offset_ne > 127 || offset_ne < -127) {
		dev_err(&chip->client->dev, "prox offset range [-127, 127]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	if (offset_ne < 0)
		offset = 128 - offset_ne;
	else
		offset = offset_ne;

	ret = tmg399x_i2c_write(chip, TMG399X_PRX_OFFSET_NE, offset);
	if (!ret) {
		chip->params.prox_offset_ne = (s8)offset_ne;
		chip->shadow[TMG399X_PRX_OFFSET_NE] = offset;
	}
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_prox_offset_sw_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			chip->params.prox_offset_sw);
}

static ssize_t tmg399x_prox_offset_sw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	long offset_sw;
	int ret;
	u8 offset = 0;

	mutex_lock(&chip->lock);
	ret = kstrtol(buf, 10, &offset_sw);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (offset_sw > 127 || offset_sw < -127) {
		dev_err(&chip->client->dev, "prox offset range [-127, 127]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	if (offset_sw < 0)
		offset = 128 - offset_sw;
	else
		offset = offset_sw;

	ret = tmg399x_i2c_write(chip, TMG399X_PRX_OFFSET_SW, offset);
	if (!ret) {
		chip->params.prox_offset_sw = (s8)offset_sw;
		chip->shadow[TMG399X_PRX_OFFSET_SW] = offset;
	}
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_prox_mask_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%.2x\n",
			chip->params.als_prox_cfg3 & 0x0F);
}

static ssize_t tmg399x_prox_mask_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long prx_mask;
	int ret;

	mutex_lock(&chip->lock);
	ret = kstrtol(buf, 10, &prx_mask);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (prx_mask > 15) {
		dev_err(&chip->client->dev, "prox mask range [0, 15]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	if ((prx_mask >> 3) ^ ((prx_mask >> 2) & 0x01) ||
		((prx_mask >> 1) & 0x01) ^ (prx_mask & 0x01))
		prx_mask |= PCMP;

	chip->shadow[TMG399X_CONFIG_3] &= 0xD0;
	chip->shadow[TMG399X_CONFIG_3] |= (u8)prx_mask;
	chip->params.als_prox_cfg3 = chip->shadow[TMG399X_CONFIG_3];
	tmg399x_i2c_write(chip, TMG399X_CONFIG_3,
		chip->shadow[TMG399X_CONFIG_3]);
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_device_prx_raw(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	mutex_lock(&chip->lock);
	tmg399x_get_prox(chip);
	mutex_unlock(&chip->lock);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.raw);
}

static ssize_t tmg399x_device_prx_detected(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	mutex_lock(&chip->lock);
	tmg399x_get_prox(chip);
	mutex_unlock(&chip->lock);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.detected);
}

static ssize_t tmg399x_get_ges_data(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	struct tmg399x_ges_raw_data data;
	mutex_lock(&chip->lock);
	data.north = chip->ges_raw_data.north;
	data.south= chip->ges_raw_data.south;
	data.west= chip->ges_raw_data.west;
	data.east= chip->ges_raw_data.east;
	mutex_unlock(&chip->lock);
	memcpy(buf, &data, sizeof(struct tmg399x_ges_raw_data));
	return sizeof(struct tmg399x_ges_raw_data);
}

static ssize_t tmg399x_ges_offset_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	buf[0] = chip->params.ges_offset_n;
	buf[1] = chip->params.ges_offset_s;
	buf[2] = chip->params.ges_offset_w;
	buf[3] = chip->params.ges_offset_e;
	buf[4] = '\0';
	return 5;
}
static ssize_t tmg399x_ges_offset_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long ges_offset;
	u8 channel;
	u8 offset;
	s8 value;
	int ret;

	mutex_lock(&chip->lock);
	ret = kstrtol(buf, 10, &ges_offset);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (ges_offset & 0x80)
		value = -(ges_offset & 0x7F);
	else
		value = ges_offset & 0x7F;

	switch (ges_offset & 0xF00) {
	case 0x100:
		channel = TMG399X_GES_OFFSET_N;
		chip->params.ges_offset_n = value;
		break;
	case 0x200:
		channel = TMG399X_GES_OFFSET_S;
		chip->params.ges_offset_s = value;
		break;
	case 0x400:
		channel = TMG399X_GES_OFFSET_W;
		chip->params.ges_offset_w = value;
		break;
	case 0x800:
		channel = TMG399X_GES_OFFSET_E;
		chip->params.ges_offset_e = value;
		break;
	default:
		dev_err(&chip->client->dev, "wrong gesture offset channel\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	offset = ges_offset & 0xFF;
	ret = tmg399x_i2c_write(chip, channel, offset);
	if (!ret) {
		chip->shadow[channel] = offset;
	}
	mutex_unlock(&chip->lock);
	return size;
}


static ssize_t tmg399x_lux_table_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	struct lux_segment *s = chip->segment;
	int i, k;

	for (i = k = 0; i < chip->segment_num; i++)
		k += snprintf(buf + k, PAGE_SIZE - k,
				"%d:%d,%d,%d,%d,%d,%d\n", i,
				s[i].d_factor,
				s[i].r_coef,
				s[i].g_coef,
				s[i].b_coef,
				s[i].ct_coef,
				s[i].ct_offset
				);
	return k;
}

static ssize_t tmg399x_lux_table_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int i;
	u32 d_factor, r_coef, g_coef, b_coef, ct_coef, ct_offset;

	mutex_lock(&chip->lock);
	if (7 != sscanf(buf, "%10d:%10d,%10d,%10d,%10d,%10d,%10d",
		&i, &d_factor, &r_coef, &g_coef, &b_coef, &ct_coef, &ct_offset)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	if (i >= chip->segment_num) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	chip->segment[i].d_factor = d_factor;
	chip->segment[i].r_coef = r_coef;
	chip->segment[i].g_coef = g_coef;
	chip->segment[i].b_coef = b_coef;
	chip->segment[i].ct_coef = ct_coef;
	chip->segment[i].ct_offset = ct_offset;
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_als_deltaP_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE,
			"%d (in %%)\n", chip->params.als_deltaP);
}

static ssize_t tmg399x_als_deltaP_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long deltaP;
	int ret;
	struct tmg399x_chip *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &deltaP);
	if (ret || deltaP > 100) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	chip->params.als_deltaP = deltaP;
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_auto_gain_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%s\n",
				chip->als_gain_auto ? "auto" : "manual");
}

static ssize_t tmg399x_auto_gain_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	mutex_lock(&chip->lock);
	if (strtobool(buf, &value)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (value)
		chip->als_gain_auto = true;
	else
		chip->als_gain_auto = false;

	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_device_als_lux(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	mutex_lock(&chip->lock);
	tmg399x_get_als(chip);
	tmg399x_get_lux(chip);
	mutex_unlock(&chip->lock);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.lux);
}

static ssize_t tmg399x_als_red_show(struct device *dev,
	struct device_attribute *attr, char *buf)
		{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.red_raw);
}

static ssize_t tmg399x_als_green_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.green_raw);
}

static ssize_t tmg399x_als_blue_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.blue_raw);
}

static ssize_t tmg399x_als_clear_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.clear_raw);
}

static ssize_t tmg399x_als_cct_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	mutex_lock(&chip->lock);
	tmg399x_get_als(chip);
	tmg399x_get_lux(chip);
	mutex_unlock(&chip->lock);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.cct);
}

static struct device_attribute prox_attrs[] = {
	__ATTR(prx_power_state, 0666, tmg399x_prox_enable_show,
			tmg399x_prox_enable_store),
	__ATTR(ges_power_state, 0666, tmg399x_ges_enable_show,
			tmg399x_ges_enable_store),
	__ATTR(prx_persist, 0666, tmg399x_prox_persist_show,
			tmg399x_prox_persist_store),
	__ATTR(prx_pulse_length, 0666, tmg399x_prox_pulse_len_show,
			tmg399x_prox_pulse_len_store),
	__ATTR(prx_pulse_count, 0666, tmg399x_prox_pulse_cnt_show,
			tmg399x_prox_pulse_cnt_store),
	__ATTR(prx_gain, 0666, tmg399x_prox_gain_show,
			tmg399x_prox_gain_store),
	__ATTR(led_drive, 0666, tmg399x_led_drive_show,
			tmg399x_led_drive_store),
	__ATTR(led_boost, 0666, tmg399x_led_boost_show,
			tmg399x_led_boost_store),
	__ATTR(prx_sat_irq_en, 0666, tmg399x_sat_irq_en_show,
			tmg399x_sat_irq_en_store),
	__ATTR(prx_offset_ne, 0666, tmg399x_prox_offset_ne_show,
			tmg399x_prox_offset_ne_store),
	__ATTR(prx_offset_sw, 0666, tmg399x_prox_offset_sw_show,
			tmg399x_prox_offset_sw_store),
	__ATTR(prx_mask, 0666, tmg399x_prox_mask_show,
			tmg399x_prox_mask_store),
	__ATTR(prx_raw, 0666, tmg399x_device_prx_raw, NULL),
	__ATTR(prx_detect, 0666, tmg399x_device_prx_detected, NULL),
	__ATTR(ges_raw_data, 0666, tmg399x_get_ges_data, NULL),
	__ATTR(ges_offset, 0666, tmg399x_ges_offset_show,
			tmg399x_ges_offset_store),
};

static struct device_attribute als_attrs[] = {
	__ATTR(als_power_state, 0666, tmg399x_als_enable_show,
			tmg399x_als_enable_store),
	__ATTR(wait_time_en, 0666, tmg399x_wait_enable_show,
			tmg399x_wait_enable_store),
	__ATTR(als_Itime, 0666, tmg399x_als_itime_show,
		tmg399x_als_itime_store),
	__ATTR(wait_time, 0666, tmg399x_wait_time_show,
		tmg399x_wait_time_store),
	__ATTR(als_persist, 0666, tmg399x_als_persist_show,
			tmg399x_als_persist_store),
	__ATTR(als_gain, 0666, tmg399x_als_gain_show,
			tmg399x_als_gain_store),
	__ATTR(lux_table, 0666, tmg399x_lux_table_show,
			tmg399x_lux_table_store),
	__ATTR(als_thresh_deltaP, 0666, tmg399x_als_deltaP_show,
			tmg399x_als_deltaP_store),
	__ATTR(als_auto_gain, 0666, tmg399x_auto_gain_enable_show,
			tmg399x_auto_gain_enable_store),
	__ATTR(als_lux, 0666, tmg399x_device_als_lux, NULL),
	__ATTR(als_red, 0666, tmg399x_als_red_show, NULL),
	__ATTR(als_green, 0666, tmg399x_als_green_show, NULL),
	__ATTR(als_blue, 0666, tmg399x_als_blue_show, NULL),
	__ATTR(als_clear, 0666, tmg399x_als_clear_show, NULL),
	__ATTR(als_cct, 0666, tmg399x_als_cct_show, NULL),
};

static void tmg399x_report_prox(struct tmg399x_chip *chip)
{
	if (chip->p_idev) {
		input_report_abs(chip->p_idev, ABS_DISTANCE,
				chip->prx_inf.detected ? 0 : 1);
		input_sync(chip->p_idev);
	}
}

static void tmg399x_report_als(struct tmg399x_chip *chip)
{
	if (chip->a_idev) {
		int ret = tmg399x_get_lux(chip);
		if (!ret) {
			int lux = chip->als_inf.lux;
			input_report_abs(chip->a_idev, ABS_MISC, lux);
			input_sync(chip->a_idev);
			tmg399x_update_als_thres(chip, 0);
		} else {
			tmg399x_update_als_thres(chip, 1);
		}
	}
}

static int tmg399x_check_and_report(struct tmg399x_chip *chip)
{
	u8 status;
	int ret;

	mutex_lock(&chip->lock);
	ret = tmg399x_i2c_read(chip, TMG399X_STATUS,
				&chip->shadow[TMG399X_STATUS]);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s: failed to read tmg399x status\n",
			__func__);
		goto exit_clr;
	}

	status = chip->shadow[TMG399X_STATUS];

	if ((status & (TMG399X_ST_GES_IRQ)) == (TMG399X_ST_GES_IRQ)) {
		//printk(KERN_INFO "Gesture interrupt generated\n");
		tmg399x_get_ges(chip);
		//ret |= tmg399x_i2c_modify(chip, TMG399X_GES_CFG_4,
		//		TMG399X_GES_INT_CLR, TMG399X_GES_INT_CLR);
	}

	if ((status & (TMG399X_ST_PRX_IRQ)) == (TMG399X_ST_PRX_IRQ)) {
		printk(KERN_INFO "Proximity interrupt generated\n");
		tmg399x_get_prox(chip);
		tmg399x_report_prox(chip);
		tmg399x_irq_clr(chip, TMG399X_CMD_PROX_INT_CLR);
	}

	if ((status & (TMG399X_ST_ALS_VALID | TMG399X_ST_ALS_IRQ)) ==
			(TMG399X_ST_ALS_VALID | TMG399X_ST_ALS_IRQ)) {
		printk(KERN_INFO "ALS interrupt generated\n");
		tmg399x_get_als(chip);
		tmg399x_report_als(chip);
		tmg399x_irq_clr(chip, TMG399X_CMD_ALS_INT_CLR);
	}

exit_clr:
	tmg399x_irq_clr(chip, TMG399X_CMD_NON_GES_INT_CLR);
	mutex_unlock(&chip->lock);

	return ret;
}

static void tmg399x_irq_work(struct work_struct *work)
{
	struct tmg399x_chip *chip =
		container_of(work, struct tmg399x_chip, irq_work);
	tmg399x_check_and_report(chip);
	enable_irq(chip->client->irq);
};

static irqreturn_t tmg399x_irq(int irq, void *handle)
{
	struct tmg399x_chip *chip = handle;
	struct device *dev = &chip->client->dev;

	disable_irq_nosync(chip->client->irq);

	if (chip->in_suspend) {
		dev_info(dev, "%s: in suspend\n", __func__);
		chip->irq_pending = 1;
		goto bypass;
	}
	schedule_work(&chip->irq_work);
bypass:
	return IRQ_HANDLED;
}

static int tmg399x_set_segment_table(struct tmg399x_chip *chip,
		struct lux_segment *segment, int seg_num)
{
	int i;
	struct device *dev = &chip->client->dev;

	chip->seg_num_max = chip->pdata->segment_num ?
			chip->pdata->segment_num : ARRAY_SIZE(segment_default);

	if (!chip->segment) {
		dev_info(dev, "%s: allocating segment table\n", __func__);
		chip->segment = kzalloc(sizeof(*chip->segment) *
				chip->seg_num_max, GFP_KERNEL);
		if (!chip->segment) {
			dev_err(dev, "%s: no memory!\n", __func__);
			return -ENOMEM;
		}
	}
	if (seg_num > chip->seg_num_max) {
		dev_warn(dev, "%s: %d segment requested, %d applied\n",
				__func__, seg_num, chip->seg_num_max);
		chip->segment_num = chip->seg_num_max;
	} else {
		chip->segment_num = seg_num;
	}
	memcpy(chip->segment, segment,
			chip->segment_num * sizeof(*chip->segment));
	dev_info(dev, "%s: %d segment requested, %d applied\n", __func__,
			seg_num, chip->seg_num_max);
	for (i = 0; i < chip->segment_num; i++)
		dev_info(dev,
		"seg %d: d_factor %d, r_coef %d, g_coef %d, b_coef %d, ct_coef %d ct_offset %d\n",
		i, chip->segment[i].d_factor, chip->segment[i].r_coef,
		chip->segment[i].g_coef, chip->segment[i].b_coef,
		chip->segment[i].ct_coef, chip->segment[i].ct_offset);
	return 0;
}

static void tmg399x_set_defaults(struct tmg399x_chip *chip)
{
	struct device *dev = &chip->client->dev;

	if (chip->pdata) {
		dev_info(dev, "%s: Loading pltform data\n", __func__);
		chip->params.als_time = chip->pdata->parameters.als_time;
		chip->params.als_gain = chip->pdata->parameters.als_gain;
		chip->params.als_deltaP = chip->pdata->parameters.als_deltaP;
		chip->params.wait_time = chip->pdata->parameters.wait_time;
		chip->params.prox_th_min = chip->pdata->parameters.prox_th_min;
		chip->params.prox_th_max = chip->pdata->parameters.prox_th_max;
		chip->params.persist = chip->pdata->parameters.persist;
		chip->params.als_prox_cfg1 = chip->pdata->parameters.als_prox_cfg1;
		chip->params.prox_pulse = chip->pdata->parameters.prox_pulse;
		chip->params.prox_gain = chip->pdata->parameters.prox_gain;
		chip->params.ldrive = chip->pdata->parameters.ldrive;
		chip->params.als_prox_cfg2 = chip->pdata->parameters.als_prox_cfg2;
		chip->params.prox_offset_ne = chip->pdata->parameters.prox_offset_ne;
		chip->params.prox_offset_sw = chip->pdata->parameters.prox_offset_sw;
		chip->params.als_prox_cfg3 = chip->pdata->parameters.als_prox_cfg3;
	} else {
		dev_info(dev, "%s: use defaults\n", __func__);
		chip->params.als_time = param_default.als_time;
		chip->params.als_gain = param_default.als_gain;
		chip->params.als_deltaP = param_default.als_deltaP;
		chip->params.wait_time = param_default.wait_time;
		chip->params.prox_th_min = param_default.prox_th_min;
		chip->params.prox_th_max = param_default.prox_th_max;
		chip->params.persist = param_default.persist;
		chip->params.als_prox_cfg1 = param_default.als_prox_cfg1;
		chip->params.prox_pulse = param_default.prox_pulse;
		chip->params.prox_gain = param_default.prox_gain;
		chip->params.ldrive = param_default.ldrive;
		chip->params.als_prox_cfg2 = param_default.als_prox_cfg2;
		chip->params.prox_offset_ne = param_default.prox_offset_ne;
		chip->params.prox_offset_sw = param_default.prox_offset_sw;
		chip->params.als_prox_cfg3 = param_default.als_prox_cfg3;
	}

	chip->als_gain_auto = false;

	/* Initial proximity threshold */
	chip->shadow[TMG399X_PRX_MINTHRESHLO] = 0;//chip->params.prox_th_min;
	chip->shadow[TMG399X_PRX_MAXTHRESHHI] = chip->params.prox_th_max;
	tmg399x_i2c_write(chip, TMG399X_PRX_MINTHRESHLO,
			chip->shadow[TMG399X_PRX_MINTHRESHLO]);
	tmg399x_i2c_write(chip, TMG399X_PRX_MAXTHRESHHI,
			chip->shadow[TMG399X_PRX_MAXTHRESHHI]);
			
	chip->shadow[TMG399X_ALS_TIME]      = chip->params.als_time;
	chip->shadow[TMG399X_WAIT_TIME]     = chip->params.wait_time;
	chip->shadow[TMG399X_PERSISTENCE]   = chip->params.persist;
	chip->shadow[TMG399X_CONFIG_1]      = chip->params.als_prox_cfg1;	
	chip->shadow[TMG399X_PRX_PULSE]     = chip->params.prox_pulse;
	chip->shadow[TMG399X_GAIN]          = chip->params.als_gain |
				chip->params.prox_gain | chip->params.ldrive;
	chip->shadow[TMG399X_CONFIG_2]      = chip->params.als_prox_cfg2;
	chip->shadow[TMG399X_PRX_OFFSET_NE] = chip->params.prox_offset_ne;
	chip->shadow[TMG399X_PRX_OFFSET_SW] = chip->params.prox_offset_sw;
	chip->shadow[TMG399X_CONFIG_3]      = chip->params.als_prox_cfg3;
}

static int tmg399x_get_id(struct tmg399x_chip *chip, u8 *id, u8 *rev)
{
	int ret;
	ret = tmg399x_i2c_read(chip, TMG399X_REVID, rev);
	ret |= tmg399x_i2c_read(chip, TMG399X_CHIPID, id);
	return ret;
}

static int tmg399x_pltf_power_on(struct tmg399x_chip *chip)
{
	int ret = 0;
	if (chip->pdata->platform_power) {
		ret = chip->pdata->platform_power(&chip->client->dev,
			POWER_ON);
		mdelay(10);
	}
	chip->unpowered = ret != 0;
	return ret;
}

static int tmg399x_pltf_power_off(struct tmg399x_chip *chip)
{
	int ret = 0;
	if (chip->pdata->platform_power) {
		ret = chip->pdata->platform_power(&chip->client->dev,
			POWER_OFF);
		chip->unpowered = ret == 0;
	} else {
		chip->unpowered = false;
	}
	return ret;
}

static int tmg399x_power_on(struct tmg399x_chip *chip)
{
	int ret;
	ret = tmg399x_pltf_power_on(chip);
	if (ret)
		return ret;
	dev_info(&chip->client->dev, "%s: chip was off, restoring regs\n",
			__func__);
	return tmg399x_flush_regs(chip);
}

static int tmg399x_prox_idev_open(struct input_dev *idev)
{
	struct tmg399x_chip *chip = dev_get_drvdata(&idev->dev);
	int ret;
	bool als = chip->a_idev && chip->a_idev->users;

	dev_info(&idev->dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	if (chip->unpowered) {
		ret = tmg399x_power_on(chip);
		if (ret)
			goto chip_on_err;
	}
	ret = tmg399x_prox_enable(chip, 1);
	if (ret && !als)
		tmg399x_pltf_power_off(chip);
chip_on_err:
	mutex_unlock(&chip->lock);
	return ret;
}

static void tmg399x_prox_idev_close(struct input_dev *idev)
{
	struct tmg399x_chip *chip = dev_get_drvdata(&idev->dev);

	dev_info(&idev->dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	tmg399x_prox_enable(chip, 0);
	if (!chip->a_idev || !chip->a_idev->users)
		tmg399x_pltf_power_off(chip);
	mutex_unlock(&chip->lock);
}

static int tmg399x_als_idev_open(struct input_dev *idev)
{
	struct tmg399x_chip *chip = dev_get_drvdata(&idev->dev);
	int ret;
	bool prox = chip->p_idev && chip->p_idev->users;

	dev_info(&idev->dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	if (chip->unpowered) {
		ret = tmg399x_power_on(chip);
		if (ret)
			goto chip_on_err;
	}
	ret = tmg399x_als_enable(chip, 1);
	if (ret && !prox)
		tmg399x_pltf_power_off(chip);
chip_on_err:
	mutex_unlock(&chip->lock);
	return ret;
}

static void tmg399x_als_idev_close(struct input_dev *idev)
{
	struct tmg399x_chip *chip = dev_get_drvdata(&idev->dev);
	dev_info(&idev->dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	tmg399x_als_enable(chip, 0);
	if (!chip->p_idev || !chip->p_idev->users)
		tmg399x_pltf_power_off(chip);
	mutex_unlock(&chip->lock);
}

static int tmg399x_add_sysfs_interfaces(struct device *dev,
	struct device_attribute *a, int size)
{
	int i;
	for (i = 0; i < size; i++)
		if (device_create_file(dev, a + i))
			goto undo;
	return 0;
undo:
	for (; i >= 0 ; i--)
		device_remove_file(dev, a + i);
	dev_err(dev, "%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}

static void tmg399x_remove_sysfs_interfaces(struct device *dev,
	struct device_attribute *a, int size)
{
	int i;
	for (i = 0; i < size; i++)
		device_remove_file(dev, a + i);
}

static int __devinit tmg399x_probe(struct i2c_client *client,
	const struct i2c_device_id *idp)
{
	int i, ret;
	u8 id, rev;
	struct device *dev = &client->dev;
	static struct tmg399x_chip *chip;
	struct tmg399x_i2c_platform_data *pdata = &tmg399x_data; //dev->platform_data;

	bool powered = 0;

	dev_info(dev, "%s: client->irq = %d\n", __func__, client->irq);
	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "%s: i2c smbus byte data unsupported\n", __func__);
		ret = -EOPNOTSUPP;
		goto init_failed;
	}
	if (!pdata) {
		dev_err(dev, "%s: platform data required\n", __func__);
		ret = -EINVAL;
		goto init_failed;
	}
	if (!(pdata->prox_name || pdata->als_name) || client->irq < 0) {
		dev_err(dev, "%s: no reason to run.\n", __func__);
		ret = -EINVAL;
		goto init_failed;
	}
	if (pdata->platform_init) {
		ret = pdata->platform_init();
		if (ret)
			goto init_failed;
	}
	if (pdata->platform_power) {
		ret = pdata->platform_power(dev, POWER_ON);
		if (ret) {
			dev_err(dev, "%s: pltf power on failed\n", __func__);
			goto pon_failed;
		}
		powered = true;
		mdelay(10);
	}
	chip = kzalloc(sizeof(struct tmg399x_chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto malloc_failed;
	}
	chip->client = client;
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);

	chip->seg_num_max = chip->pdata->segment_num ?
			chip->pdata->segment_num : ARRAY_SIZE(segment_default);
	if (chip->pdata->segment)
		ret = tmg399x_set_segment_table(chip, chip->pdata->segment,
			chip->pdata->segment_num);
	else
		ret =  tmg399x_set_segment_table(chip, segment_default,
			ARRAY_SIZE(segment_default));
	if (ret)
		goto set_segment_failed;

	ret = tmg399x_get_id(chip, &id, &rev);
	if (ret < 0)
		dev_err(&chip->client->dev,
			"%s: failed to get tmg399x id\n",
			__func__);

	dev_info(dev, "%s: device id:%02x device rev:%02x\n", __func__,
				id, rev);

	for (i = 0; i < ARRAY_SIZE(tmg399x_ids); i++) {
		if (id == tmg399x_ids[i])
			break;
	}
	if (i < ARRAY_SIZE(tmg399x_names)) {
		dev_info(dev, "%s: '%s rev. %d' detected\n", __func__,
			tmg399x_names[i], rev);
		chip->device_index = i;
	} else {
		dev_err(dev, "%s: not supported chip id\n", __func__);
		ret = -EOPNOTSUPP;
		goto id_failed;
	}

	mutex_init(&chip->lock);
	tmg399x_set_defaults(chip);
	ret = tmg399x_flush_regs(chip);
	if (ret)
		goto flush_regs_failed;
	if (pdata->platform_power) {
		pdata->platform_power(dev, POWER_OFF);
		powered = false;
		chip->unpowered = true;
	}

	if (!pdata->prox_name)
		goto bypass_prox_idev;
	chip->p_idev = input_allocate_device();
	if (!chip->p_idev) {
		dev_err(dev, "%s: no memory for input_dev '%s'\n",
				__func__, pdata->prox_name);
		ret = -ENODEV;
		goto input_p_alloc_failed;
	}
	chip->p_idev->name = pdata->prox_name;
	chip->p_idev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, chip->p_idev->evbit);
	set_bit(ABS_DISTANCE, chip->p_idev->absbit);
	input_set_abs_params(chip->p_idev, ABS_DISTANCE, 0, 1, 0, 0);
	chip->p_idev->open = tmg399x_prox_idev_open;
	chip->p_idev->close = tmg399x_prox_idev_close;
	dev_set_drvdata(&chip->p_idev->dev, chip);
	ret = input_register_device(chip->p_idev);
	if (ret) {
		input_free_device(chip->p_idev);
		dev_err(dev, "%s: cant register input '%s'\n",
				__func__, pdata->prox_name);
		goto input_p_register_failed;
	}
	ret = tmg399x_add_sysfs_interfaces(&chip->p_idev->dev,
			prox_attrs, ARRAY_SIZE(prox_attrs));
	if (ret)
		goto input_p_sysfs_failed;
bypass_prox_idev:
	if (!pdata->als_name)
		goto bypass_als_idev;
	chip->a_idev = input_allocate_device();
	if (!chip->a_idev) {
		dev_err(dev, "%s: no memory for input_dev '%s'\n",
				__func__, pdata->als_name);
		ret = -ENODEV;
		goto input_a_alloc_failed;
	}
	chip->a_idev->name = pdata->als_name;
	chip->a_idev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, chip->a_idev->evbit);
	set_bit(ABS_MISC, chip->a_idev->absbit);
	input_set_abs_params(chip->a_idev, ABS_MISC, 0, 65535, 0, 0);
	chip->a_idev->open = tmg399x_als_idev_open;
	chip->a_idev->close = tmg399x_als_idev_close;
	dev_set_drvdata(&chip->a_idev->dev, chip);
	ret = input_register_device(chip->a_idev);
	if (ret) {
		input_free_device(chip->a_idev);
		dev_err(dev, "%s: cant register input '%s'\n",
				__func__, pdata->prox_name);
		goto input_a_register_failed;
	}
	ret = tmg399x_add_sysfs_interfaces(&chip->a_idev->dev,
			als_attrs, ARRAY_SIZE(als_attrs));
	if (ret)
		goto input_a_sysfs_failed;

	gpio_request(74, "tmg-irq");
	gpio_direction_input(74);

bypass_als_idev:
	INIT_WORK(&chip->irq_work, tmg399x_irq_work);
	ret = request_threaded_irq(client->irq, NULL, &tmg399x_irq,
		      IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		      dev_name(dev), chip);
	if (ret) {
		dev_info(dev, "Failed to request irq %d\n", client->irq);
		goto irq_register_fail;
	}

	dev_info(dev, "Probe ok.\n");
	return 0;

irq_register_fail:
	if (chip->a_idev) {
		tmg399x_remove_sysfs_interfaces(&chip->a_idev->dev,
			als_attrs, ARRAY_SIZE(als_attrs));
input_a_sysfs_failed:
		input_unregister_device(chip->a_idev);
input_a_register_failed:
		input_free_device(chip->a_idev);
	}
input_a_alloc_failed:
	if (chip->p_idev) {
		tmg399x_remove_sysfs_interfaces(&chip->p_idev->dev,
			prox_attrs, ARRAY_SIZE(prox_attrs));
input_p_sysfs_failed:
		input_unregister_device(chip->p_idev);
input_p_register_failed:
		input_free_device(chip->p_idev);
	}	
input_p_alloc_failed:
flush_regs_failed:
id_failed:
	kfree(chip->segment);
set_segment_failed:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
malloc_failed:
	if (powered && pdata->platform_power)
		pdata->platform_power(dev, POWER_OFF);
pon_failed:
	if (pdata->platform_teardown)
		pdata->platform_teardown(dev);
init_failed:
	dev_err(dev, "Probe failed.\n");
	return ret;
}

static int tmg399x_suspend(struct device *dev)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	struct tmg399x_i2c_platform_data *pdata = dev->platform_data;

	dev_info(dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	chip->in_suspend = 1;

	if (chip->p_idev && chip->p_idev->users) {
		if (pdata->proximity_can_wake) {
			dev_info(dev, "set wake on proximity\n");
			chip->wake_irq = 1;
		} else {
			dev_info(dev, "proximity off\n");
			tmg399x_prox_enable(chip, 0);
		}
	}
	if (chip->a_idev && chip->a_idev->users) {
		if (pdata->als_can_wake) {
			dev_info(dev, "set wake on als\n");
			chip->wake_irq = 1;
		} else {
			dev_info(dev, "als off\n");
			tmg399x_als_enable(chip, 0);
		}
	}
	if (chip->wake_irq) {
		irq_set_irq_wake(chip->client->irq, 1);
	} else if (!chip->unpowered) {
		dev_info(dev, "powering off\n");
		tmg399x_pltf_power_off(chip);
	}
	mutex_unlock(&chip->lock);

	return 0;
}

static int tmg399x_resume(struct device *dev)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool als_on, prx_on;
	int ret = 0;
	mutex_lock(&chip->lock);
	prx_on = chip->p_idev && chip->p_idev->users;
	als_on = chip->a_idev && chip->a_idev->users;
	chip->in_suspend = 0;

	dev_info(dev, "%s: powerd %d, als: needed %d  enabled %d",
			__func__, !chip->unpowered, als_on,
			chip->als_enabled);
	dev_info(dev, " %s: prox: needed %d  enabled %d\n",
			__func__, prx_on, chip->prx_enabled);

	if (chip->wake_irq) {
		irq_set_irq_wake(chip->client->irq, 0);
		chip->wake_irq = 0;
	}
	if (chip->unpowered && (prx_on || als_on)) {
		dev_info(dev, "powering on\n");
		ret = tmg399x_power_on(chip);
		if (ret)
			goto err_power;
	}
	if (prx_on && !chip->prx_enabled)
		(void)tmg399x_prox_enable(chip, 1);
	if (als_on && !chip->als_enabled)
		(void)tmg399x_als_enable(chip, 1);
	if (chip->irq_pending) {
		dev_info(dev, "%s: pending interrupt\n", __func__);
		chip->irq_pending = 0;
		(void)tmg399x_check_and_report(chip);
		enable_irq(chip->client->irq);
	}
err_power:
	mutex_unlock(&chip->lock);

	return 0;
}

static int __devexit tmg399x_remove(struct i2c_client *client)
{
	struct tmg399x_chip *chip = i2c_get_clientdata(client);
	mutex_lock(&chip->lock);
	free_irq(client->irq, chip);
	if (chip->a_idev) {
		tmg399x_remove_sysfs_interfaces(&chip->a_idev->dev,
			als_attrs, ARRAY_SIZE(als_attrs));
		input_unregister_device(chip->a_idev);
	}
	if (chip->p_idev) {
		tmg399x_remove_sysfs_interfaces(&chip->p_idev->dev,
			prox_attrs, ARRAY_SIZE(prox_attrs));
		input_unregister_device(chip->p_idev);
	}
	if (chip->pdata->platform_teardown)
		chip->pdata->platform_teardown(&client->dev);
	i2c_set_clientdata(client, NULL);
	kfree(chip->segment);
	kfree(chip);
	mutex_unlock(&chip->lock);
	return 0;
}

static struct i2c_device_id tmg399x_idtable[] = {
	{ "tmg399x", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, tmg399x_idtable);

#ifdef CONFIG_OF
static struct of_device_id tmg399x_match_table[] = {
	{ .compatible = "tmg399x",},
	{ },
};
#endif

static const struct dev_pm_ops tmg399x_pm_ops = {
	.suspend = tmg399x_suspend,
	.resume  = tmg399x_resume,
};

static struct i2c_driver tmg399x_driver = {
	.driver = {
		.name = "tmg399x",
		.pm = &tmg399x_pm_ops,
	#ifdef CONFIG_OF
		.of_match_table = tmg399x_match_table,
    #endif
	},
	.id_table = tmg399x_idtable,
	.probe = tmg399x_probe,
	.remove = __devexit_p(tmg399x_remove),
};

static int __init tmg399x_init(void)
{
	return i2c_add_driver(&tmg399x_driver);
}

static void __exit tmg399x_exit(void)
{
	i2c_del_driver(&tmg399x_driver);
}

module_init(tmg399x_init);
module_exit(tmg399x_exit);

MODULE_AUTHOR("J. August Brenner<jon.brenner@ams.com>");
MODULE_AUTHOR("Byron Shi<byron.shi@ams.com>");
MODULE_DESCRIPTION("AMS-TAOS tmg3992/3 Ambient, Proximity, Gesture sensor driver");
MODULE_LICENSE("GPL");
