/*
* drivers/misc/lps25.c
*
* STMicroelectronics LPS25H Pressure / Temperature Sensor module driver
*
* Copyright (C) 2013 STMicroelectronics- AMS/High-End Sensors and Analog Division
* Environmental Sensor Application Development Team
* Lorenzo Sarchi (lorenzo.sarchi@st.com)
* Version: 1.0.1, Date: 2013/Oct/18 
*
* Based on the original work from Matteo Dameno and Carmine Iascone
* (upated to revision 1.0.0 2012/Nov/21):
*
* Please refer to Raffaele Di Vaio (raffaele.di-vaio@st.com) as contact and update point
*
*
** Output data from the device are available from the assigned
* /dev/input/eventX device;
*
* LPS25 can be controlled by sysfs interface looking inside:
* /sys/bus/i2c/devices/<busnum>-<devaddr>/
*
* LPS25 make available two i2C addresses selectable from platform_data
* by the LPS25H_PRS_I2C_SAD_H or LPS25H_PRS_I2C_SAD_L.
*
* Read pressures and temperatures output can be converted in units of
* measurement by dividing them respectively for SENSITIVITY_P and SENSITIVITY_T.
* Temperature values must then be added by the constant float TEMPERATURE_OFFSET
* expressed as Celsius degrees.
*
* Obtained values are then expessed as
* mbar (=0.1 kPa) and Celsius degrees.
*
* To use autozero feature you can write 0 zero or 1 to its corresponding sysfs
* file. This lets you to write current temperature and pressure into reference
* registers or to reset them.
*
* To use fifo, you can write 0 or 1 to its corresponding sysfs file.
* To set fifo mode, you can write proper values in fifo_mode sysfs file:
* fifo: 1; stream: 2; mean: 6; stream2fifo: 3; bypass2stream: 4; bypass2fifo: 7
* Same for setting nbr of samples in corresponding sysfs_file num_samples_fifo.
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
* 02110-1301 USA
*
*/
/******************************************************************************
 Revision 0.0.1 2013/Oct/14: 1st draft
******************************************************************************/

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
//#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>

//#include <linux/input/lps25.h>
#include <linux/sensors/lps25h.h>
#include <linux/of_gpio.h>
#include <linux/of.h>

//#define	DEBUG	0
//#define	DEBUG	1


#define	PR_ABS_MAX	8388607		/* 24 bit 2'compl */
#define	PR_ABS_MIN	-8388608

#ifdef SHRT_MAX
#define	TEMP_MAX	SHRT_MAX
#define TEMP_MIN	SHRT_MIN
#else
#define	TEMP_MAX	SHORT_MAX
#define TEMP_MIN	SHORT_MIN
#endif

#define	WHOAMI_LPS25_PRS	0xBD	/*	Expctd content for WAI	*/

/*	CONTROL REGISTERS	*/
#define	REF_P_XL	0x08		/*	pressure reference	*/
#define	REF_P_L		0x09		/*	pressure reference	*/
#define	REF_P_H		0x0A		/*	pressure reference	*/
#define	REF_T_L		0x0B		/*	temperature reference	*/
#define	REF_T_H		0x0C		/*	temperature reference	*/


#define	WHO_AM_I	0x0F		/*	WhoAmI register		*/

#define	RESOL_CONF	0x10		/*	Pres Temp resolution set*/
#define	TP_RESOL	0x10		/*	Pres Temp resolution set*/

#define	DGAIN_L		0x18		/*	Dig Gain (3 regs)	*/

#define	CTRL_REG1	0x20		/*	power / ODR control reg	*/
#define	CTRL_REG2	0x21		/*	boot reg		*/
#define	CTRL_REG3	0x22		/*	interrupt control reg	*/
#define	CTRL_REG4	0x23		/*	interrupt control reg	*/
#define	INT_CFG_REG	0x24		/*	interrupt config reg	*/
#define	INT_SRC_REG	0x25		/*	interrupt source reg	*/

#define	STATUS_REG	0X27		/*	status reg		*/

#define	PRESS_OUT_XL	0x28		/*	press output (3 regs)	*/
#define	TEMP_OUT_L	0x2B		/*	temper output (2 regs)	*/

#define	FIFO_CTRL	0x2E
#define	FIFO_STATUS	0x2F

#define	THS_P_L		0x30		/*	pressure threshold	*/
#define	THS_P_H		0x31		/*	pressure threshold	*/


#define	RPDS_TRM_L	0x39		/*	NEW	*/
#define	RPDS_TRM_H	0x3A		/*	NEW	*/


/*	REGISTERS ALIASES	*/
#define	P_REF_INDATA_REG	REF_P_XL
#define	T_REF_INDATA_REG	REF_T_L
#define	P_THS_INDATA_REG	THS_P_L
#define	P_OUTDATA_REG		PRESS_OUT_XL
#define	T_OUTDATA_REG		TEMP_OUT_L
#define	OUTDATA_REG		PRESS_OUT_XL

/* */
#define	ENABLE_MASK		0x80	/*  ctrl_reg1 */
#define	ODR_MASK		0x70	/*  ctrl_reg1 */
#define	DIFF_MASK		0x08	/*  ctrl_reg1 */
#define	BDU_MASK		0x04	/*  ctrl_reg1 */
#define	RESET_AZ		0x02	/*  ctrl_reg1 */

#define	AUTOZ_MASK		0x02	/*  ctrl_reg2 */
#define	AUTOZ_ON		0x02	/* Enab AutoZero Function */
#define	AUTOZ_OFF		0x00	/* Disab Difference Function */
/* Pressure Sensor Operating Mode */
#define	AUTOZ_ENABLE	1
#define	AUTOZ_DISABLE	0


#define	LPS25_PRS_DELTA_EN_MASK	0x02	/*  ctrl_reg1 */
#define	LPS25_PRS_AUTOZ_MASK		0x02	/*  ctrl_reg2 */

#define	LPS25_PRS_RESET_AZ_MASK		0x02	/*  ctrl_reg1 */


#define	PM_NORMAL		0x80	/* Power Normal Mode*/
#define	PM_OFF		0x00	/* Power Down */

#define	LPS25_PRS_DIFF_ON		0x08	/* En Difference circuitry */
#define	LPS25_PRS_DIFF_OFF		0x00	/* Dis Difference circuitry */

#define	LPS25_PRS_AUTOZ_ON		0x02	/* En AutoZero Function */
#define	LPS25_PRS_AUTOZ_OFF		0x00	/* Dis Difference Function */

#define	LPS25_PRS_BDU_ON		0x04	/* En BDU Block Data Upd */

#define	RES_AVGTEMP_064		0X0C
#define	RES_AVGTEMP_032		0X08

#define	RES_AVGPRES_512		0X03
#define	RES_AVGPRES_128		0X02

#define	RES_MAX	(RES_AVGTEMP_064 | RES_AVGPRES_512)	/* Max Resol. */


#define	LPS25_PRS_DELTA_EN_ON	0x02	/* En Delta Press registers */

#define	FUZZ			0
#define	FLAT			0

#define FIFO_MODE_MASK		0xE0	// 3 MSB FIFO_CTRL: Mode
#define FIFO_SAMPLE_MASK	0x1F	// 5 LSB FIFO_CTRL: Num Samples

#define	I2C_AUTO_INCREMENT	0x80

#define I2C_RETRY_DELAY		5
#define I2C_RETRIES		5

/* RESUME STATE INDICES */
#define	RES_REF_P_XL		0
#define	RES_REF_P_L		1
#define	RES_REF_P_H		2
#define	RES_REFT_L		3
#define	RES_REFT_H		4
//#define	LPS25_RES_TP_RESOL		5
#define	RES_RESOL_CONF			5
#define	RES_CTRL_REG1		6
#define	RES_CTRL_REG2		7
#define	RES_CTRL_REG3		8
#define	RES_CTRL_REG4		9
#define	RES_INT_CFG_REG		10
#define	RES_FIFO_CTRL		11
#define	RES_THS_P_L		12
#define	RES_THS_P_H		13
#define	RES_RPSD_TRIM_L		14
#define	RES_RPSD_TRIM_H		15

#define	RESUME_ENTRIES		16


/* end RESUME STATE INDICES */

/* Pressure Sensor Operating Mode */
#define	LPS25_PRS_DIFF_ENABLE	1
#define LPS25_PRS_DIFF_DISABLE	0
#define	LPS25_PRS_AUTOZ_ENABLE	1
#define	LPS25_PRS_AUTOZ_DISABLE	0

#define	BDU_ON			0x04	/* En BDU Block Data Upd */

/* Barometer and Termometer output data rate ODR */
#define	ODR_ONESH	0x00	/* one shot both		*/
#define	ODR_1_1		0x10	/*  1  Hz baro,  1  Hz term ODR	*/
#define	ODR_7_7		0x20	/*  7  Hz baro,  7  Hz term ODR	*/
#define	ODR_12_12	0x30	/* 12.5Hz baro, 12.5Hz term ODR	*/
#define	ODR_25_25	0x40	/* 25  Hz baro, 25  Hz term ODR	*/

#define LPS25_DEV_NAME_PRS 		"pressure"


static const struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lps25h_prs_odr_table[] = {
	{40,	ODR_25_25 },
	{80,	ODR_12_12 },
	{143,	ODR_7_7 },
	{1000,	ODR_1_1 },
};

struct lps25_prs_data {
	struct i2c_client *client;
	struct lps25_prs_platform_data *pdata;

	struct mutex lock;
	struct delayed_work input_work;

	struct input_dev *input_dev_pres;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;


	atomic_t enabled;
	int on_before_suspend;

	u8 resume_state[RESUME_ENTRIES];

#ifdef DEBUG
	u8 reg_addr;
#endif

	u32 TSL, TSH; 		// temperature points 1 - 2 - 3
	u32 TCV1, TCV2, TCV3;
	u32 TCS1, TCS2, TCS3;
	u32 digGain;
	s8 deltaT1, deltaT2, deltaT3;
	u8 testVer;

        int use_smbus;	
};

struct outputdata {
	s32 press;
	s16 temperature;
};

static const struct lps25_prs_platform_data default_lps25_pdata = {
        .poll_interval = 1000,
        .min_interval = LPS25_PRS_MIN_POLL_PERIOD_MS,
};


static int lps25_prs_i2c_read(struct lps25_prs_data *prs, u8 *buf, int len)
{
        int ret;
	int tries = 0;
        u8 reg = buf[0];
        u8 cmd = reg;	

#ifdef DEBUG
        unsigned int ii;
#endif
 
 
        if (len > 1)
                cmd = (I2C_AUTO_INCREMENT | reg);
        if (prs->use_smbus) {
                if (len == 1) {
                        ret = i2c_smbus_read_byte_data(prs->client, cmd);
                        buf[0] = ret & 0xff;
#ifdef DEBUG
                        dev_warn(&prs->client->dev,
                                "i2c_smbus_read_byte_data: ret=0x%02x, len:%d ,"
                                "command=0x%02x, buf[0]=0x%02x\n",
                                ret, len, cmd , buf[0]);
#endif
                } else if (len > 1) {
                        ret = i2c_smbus_read_i2c_block_data(prs->client,
                                                                cmd, len, buf);
#ifdef DEBUG
                        dev_warn(&prs->client->dev,
                                "i2c_smbus_read_i2c_block_data: ret:%d len:%d, "
                                "command=0x%02x, ",
                                ret, len, cmd);
                        for (ii = 0; ii < len; ii++)
                                printk(KERN_DEBUG "buf[%d]=0x%02x,",
                                                                ii, buf[ii]);
 
                        printk("\n");
#endif
                } else
                        ret = -1;
 
                if (ret < 0) {
                        dev_err(&prs->client->dev,
                                "read transfer error: len:%d, command=0x%02x\n",
                                len, cmd);
                        return 0;
                }
                return len;
        }

	do {
		ret = i2c_master_send(prs->client, &cmd, sizeof(cmd));	
	        if (ret != sizeof(cmd))		
			msleep_interruptible (I2C_RETRY_DELAY);
	}	while ( (ret != sizeof(cmd)) && (++tries < I2C_RETRIES));
        if (ret != sizeof(cmd))
                return ret;
        return i2c_master_recv(prs->client, (char *)buf, len);	
}


static int lps25_prs_i2c_write(struct lps25_prs_data *prs,
				   u8 *buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
		 .addr = prs->client->addr,
		 .flags = prs->client->flags & I2C_M_TEN,
		 .len = len + 1,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(prs->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible (I2C_RETRY_DELAY);
	}	while ( (err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&prs->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}
	return 0;
}


static int lps25_prs_register_write(struct lps25_prs_data *prs, u8 *buf,
		u8 reg_address, u8 new_value)
{
	int err;

	/* Sets configuration register at reg_address
	 *  NOTE: this is a straight overwrite  */
	buf[0] = reg_address;
	buf[1] = new_value;
	err = lps25_prs_i2c_write(prs, buf, 1);
	if (err < 0)
		return err;
	return err;
}

static int lps25_prs_register_read(struct lps25_prs_data *prs, u8 *buf,
		u8 reg_address)
{
	int err;
	buf[0] = (reg_address);
	err = lps25_prs_i2c_read(prs, buf, 1);

	return err;
}

static int lps25_prs_register_update(struct lps25_prs_data *prs, u8 *buf,
		u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err;
	u8 init_val;
	u8 updated_val;
	err = lps25_prs_register_read(prs, buf, reg_address);
	if (!(err < 0)) {
		init_val = buf[0];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		err = lps25_prs_register_write(prs, buf, reg_address,
				updated_val);
	}
	return err;
}

static int lps25_prs_hw_init(struct lps25_prs_data *prs)
{
	int err;
	u8 buf[6];

        pr_info("%s: hw init start\n", LPS25_PRS_DEV_NAME);
	dev_dbg(&prs->client->dev,"%s: hw init start\n", LPS25_PRS_DEV_NAME);

	buf[0] = WHO_AM_I;
	err = lps25_prs_i2c_read(prs, buf, 1);
	if (err < 0)
		goto error_firstread;
	else
		prs->hw_working = 1;
	if (buf[0] != WHOAMI_LPS25_PRS) {
		err = -1; /* TODO:choose the right coded error */
		goto error_unknown_device;
	}


	buf[0] = (I2C_AUTO_INCREMENT | P_REF_INDATA_REG);
	buf[1] = prs->resume_state[RES_REF_P_XL];
	buf[2] = prs->resume_state[RES_REF_P_L];
	buf[3] = prs->resume_state[RES_REF_P_H];

	err = lps25_prs_i2c_write(prs, buf, 3);
	if (err < 0)
		goto err_resume_state;

	buf[0] = RESOL_CONF;
	buf[1] = prs->resume_state[RES_RESOL_CONF];
	err = lps25_prs_i2c_write(prs, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (I2C_AUTO_INCREMENT | P_THS_INDATA_REG);
	buf[1] = prs->resume_state[RES_THS_P_L];
	buf[2] = prs->resume_state[RES_THS_P_H];
	err = lps25_prs_i2c_write(prs, buf, 2);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (I2C_AUTO_INCREMENT | CTRL_REG2);
	buf[1] = prs->resume_state[RES_CTRL_REG2];
	buf[2] = prs->resume_state[RES_CTRL_REG3];
	buf[3] = prs->resume_state[RES_CTRL_REG4];
	err = lps25_prs_i2c_write(prs, buf, 3);
	if (err < 0)
		goto err_resume_state;

	buf[0] = INT_CFG_REG;
	buf[1] = prs->resume_state[RES_INT_CFG_REG];
	err = lps25_prs_i2c_write(prs, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = CTRL_REG1;
	buf[1] = prs->resume_state[RES_CTRL_REG1];
	err = lps25_prs_i2c_write(prs, buf, 1);
	if (err < 0)
		goto err_resume_state;

	prs->hw_initialized = 1;
	dev_dbg(&prs->client->dev, "%s: hw init done\n", LPS25_PRS_DEV_NAME);
	return 0;



        pr_info("%s: hw init done\n", LPS25_PRS_DEV_NAME);
	dev_dbg(&prs->client->dev, "%s: hw init done\n", LPS25_PRS_DEV_NAME);
	return 0;

error_firstread:
	prs->hw_working = 0;
	dev_warn(&prs->client->dev, "Error reading WHO_AM_I: is device "
		"available/working?\n");
	goto err_resume_state;
error_unknown_device:
	dev_err(&prs->client->dev,
		"device unknown. Expected: 0x%02x,"
		" Replies: 0x%02x\n", WHOAMI_LPS25_PRS, buf[0]);
err_resume_state:
	prs->hw_initialized = 0;
	dev_err(&prs->client->dev, "hw init error 0x%02x,0x%02x: %d\n", buf[0],
			buf[1], err);
	return err;
}

static void lps25_prs_device_power_off(struct lps25_prs_data *prs)
{
	int err;
	u8 buf[2] = { CTRL_REG1, PM_OFF };

	err = lps25_prs_i2c_write(prs, buf, 1);
	if (err < 0)
		dev_err(&prs->client->dev, "soft power off failed: %d\n", err);

	if (prs->pdata->power_off)
		prs->pdata->power_off();
	prs->hw_initialized = 0;
}

static int lps25_prs_device_power_on(struct lps25_prs_data *prs)
{
	int err = -1;

	if (prs->pdata->power_on) {
		err = prs->pdata->power_on();
		if (err < 0) {
			dev_err(&prs->client->dev,
					"power_on failed: %d\n", err);
			return err;
		}
	}

	if (!prs->hw_initialized) {
		err = lps25_prs_hw_init(prs);
		if (prs->hw_working == 1 && err < 0) {
			lps25_prs_device_power_off(prs);
			return err;
		}
	}

	return 0;
}


int lps25_prs_update_odr(struct lps25_prs_data *prs, int poll_period_ms)
{
	int err = -1;
	int i;

	u8 buf[2];
	u8 init_val, updated_val;
	u8 curr_val, new_val;
	u8 mask = ODR_MASK;
	u8 resol = RES_MAX;

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (longest period) backward (shortest
	 * period), to support the poll_interval requested by the system.
	 * It must be the longest period shorter then the set poll period.*/
	for (i = ARRAY_SIZE(lps25h_prs_odr_table) - 1; i >= 0; i--) {
		if ((lps25h_prs_odr_table[i].cutoff_ms <= poll_period_ms)
								|| (i == 0))
			break;
	}

	new_val = lps25h_prs_odr_table[i].mask;

	/* before to change the ODR it is mandatory to power down
	the device */

	buf[0] = CTRL_REG1;
	err = lps25_prs_i2c_read(prs, buf, 1);
	if (err < 0)
		goto error;
	/* work on all but ENABLE bits */
	/* power down */
	init_val = buf[0];
	prs->resume_state[RES_CTRL_REG1] = init_val ;

	curr_val = ((ENABLE_MASK & PM_OFF)
			| ((~ENABLE_MASK) & init_val));
	buf[0] = CTRL_REG1;
	buf[1] = curr_val;
	err = lps25_prs_i2c_write(prs, buf, 1);
	if (err < 0)
		goto error;

	/* set new ODR*/
	buf[0] = CTRL_REG1;
	updated_val = ((mask & new_val) | ((~mask) & curr_val));

	buf[0] = CTRL_REG1;
	buf[1] = updated_val;
	err = lps25_prs_i2c_write(prs, buf, 1);
	if (err < 0)
		goto error;

	/* power up */
	curr_val = ((ENABLE_MASK &
					PM_NORMAL)
		| ((~ENABLE_MASK) & updated_val));
	buf[0] = CTRL_REG1;
	buf[1] = curr_val;
	err = lps25_prs_i2c_write(prs, buf, 1);
	if (err < 0)
		goto error;

	buf[0] = RESOL_CONF;
	buf[1] = resol;
	err = lps25_prs_i2c_write(prs, buf, 1);
	if (err < 0)
		goto error;

	prs->resume_state[RES_CTRL_REG1] = curr_val;
	prs->resume_state[RES_RESOL_CONF] = resol;

	return err;

error:
	dev_err(&prs->client->dev, "update odr failed 0x%02x,0x%02x: %d\n",
			buf[0], buf[1], err);

	return err;
}

static int lps25_prs_set_press_reference(struct lps25_prs_data *prs,
				s32 new_reference)
{
	int err;

	u8 bit_valuesXL,bit_valuesL, bit_valuesH;
	u8 buf[4];

	bit_valuesXL = (u8) (new_reference & 0x0000FF);
	bit_valuesL = (u8)((new_reference & 0x00FF00) >> 8);
	bit_valuesH = (u8)((new_reference & 0xFF0000) >> 16);


	buf[0] = (I2C_AUTO_INCREMENT | P_REF_INDATA_REG);
	buf[1] = bit_valuesXL;
	buf[2] = bit_valuesL;
	buf[3] = bit_valuesH;

	err = lps25_prs_i2c_write(prs,buf,3);
	if (err < 0)
		return err;

	prs->resume_state[RES_REF_P_XL] = bit_valuesXL;
	prs->resume_state[RES_REF_P_L] = bit_valuesL;
	prs->resume_state[RES_REF_P_H] = bit_valuesH;

	return err;
}

static int lps25_prs_get_press_reference(struct lps25_prs_data *prs,
		s32 *buf32)
{
	int err;

	u8 bit_valuesXL, bit_valuesL, bit_valuesH;
	u8 buf[3];
	u16 temp = 0;

	buf[0] =  (I2C_AUTO_INCREMENT | P_REF_INDATA_REG);
	err = lps25_prs_i2c_read(prs, buf, 3);
	if (err < 0)
		return err;
	bit_valuesXL = buf[0];
	bit_valuesL = buf[1];
	bit_valuesH = buf[2];


	temp = (( bit_valuesH ) << 8 ) | ( bit_valuesL ) ;
	*buf32 = (s32)((((s16) temp) << 8) | ( bit_valuesXL ));
#ifdef DEBUG
	dev_dbg(&prs->client->dev,"%s val: %+d", LPS25_PRS_DEV_NAME, *buf32 );
#endif
	return err;
}

static int lps25_prs_set_temperature_reference(struct lps25_prs_data *prs,
				s16 new_reference)
{
	int err;
	u8 bit_valuesL, bit_valuesH;
	u8 buf[3];

	bit_valuesL = (u8) ( new_reference & 0x00FF );
	bit_valuesH = (u8)(( new_reference & 0xFF00 ) >> 8);

	buf[0] = (I2C_AUTO_INCREMENT | T_REF_INDATA_REG);
	buf[1] = bit_valuesL;
	buf[2] = bit_valuesH;
	err = lps25_prs_i2c_write(prs,buf,2);

	if (err < 0)
		return err;

	prs->resume_state[RES_REFT_L] = bit_valuesL;
	prs->resume_state[RES_REFT_H] = bit_valuesH;
	return err;
}

static int lps25_prs_get_temperature_reference(struct lps25_prs_data *prs,
		s16 *buf16)
{
	int err;

	u8 bit_valuesL, bit_valuesH;
	u8 buf[2] = {0};
	u16 temp = 0;


	buf[0] =  (I2C_AUTO_INCREMENT | T_REF_INDATA_REG);
	err = lps25_prs_i2c_read(prs, buf, 2);
	if (err < 0)
		return err;

	bit_valuesL = buf[0];
	bit_valuesH = buf[1];

	temp = ( ( (u16) bit_valuesH  ) << 8 );
	*buf16 = (s16)( temp | ((u16) bit_valuesL ));

	return err;
}


static int lps25_prs_get_presstemp_data(struct lps25_prs_data *prs,
						struct outputdata *out)
{
	int err;
	/* Data bytes from hardware	PRESS_OUT_XL,PRESS_OUT_L,PRESS_OUT_H, */
	/*				TEMP_OUT_L, TEMP_OUT_H */

	u8 prs_data[5];

	s32 pressure;
	s16 temperature;

	int regToRead = 5;

	prs_data[0] = (I2C_AUTO_INCREMENT | OUTDATA_REG);
	err = lps25_prs_i2c_read(prs, prs_data, regToRead);
	if (err < 0)
		return err;

#ifdef DEBUG
	pr_info("temp out tH = 0x%02x, tL = 0x%02x,"
			"press_out: pH = 0x%02x, pL = 0x%02x, pXL= 0x%02x\n",
					prs_data[4],
					prs_data[3],
					prs_data[2],
					prs_data[1],
					prs_data[0]);
#endif

	pressure = (s32)((((s8) prs_data[2]) << 16) |
				(prs_data[1] <<  8) |
						( prs_data[0]));
	temperature = (s16) ((((s8) prs_data[4]) << 8) | (prs_data[3]));

#ifdef DEBUG
	pr_info("\n Pressure: %d ", (int32_t)pressure);
#endif

	out->press = pressure;

	out->temperature = temperature;

	return err;
}



static void lps25_prs_report_values(struct lps25_prs_data *prs,
					struct outputdata *out)
{

	input_report_abs(prs->input_dev_pres, LPS25_ABS_PR, out->press);

	input_report_abs(prs->input_dev_pres, LPS25_ABS_TEMP, out->temperature);
	input_sync(prs->input_dev_pres);
}


static int lps25_prs_enable(struct lps25_prs_data *prs)
{
	int err;

	if (!atomic_cmpxchg(&prs->enabled, 0, 1)) {
		err = lps25_prs_device_power_on(prs);
		if (err < 0) {
			atomic_set(&prs->enabled, 0);
			return err;
		}
		schedule_delayed_work(&prs->input_work,
			msecs_to_jiffies(prs->pdata->poll_interval));
	}

	return 0;
}

static int lps25_prs_disable(struct lps25_prs_data *prs)
{
	if (atomic_cmpxchg(&prs->enabled, 1, 0)) {
		cancel_delayed_work_sync(&prs->input_work);
		lps25_prs_device_power_off(prs);
	}

	return 0;
}


static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct lps25_prs_data *prs = dev_get_drvdata(dev);
	mutex_lock(&prs->lock);
	val = prs->pdata->poll_interval;
	mutex_unlock(&prs->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct lps25_prs_data *prs = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	interval_ms = max((unsigned int)interval_ms,prs->pdata->min_interval);
	mutex_lock(&prs->lock);
	prs->pdata->poll_interval = interval_ms;
	lps25_prs_update_odr(prs, interval_ms);
	mutex_unlock(&prs->lock);
	return size;
}


static ssize_t attr_get_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lps25_prs_data *prs = dev_get_drvdata(dev);
	int val = atomic_read(&prs->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lps25_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;

#ifdef DEBUG
	pr_info("\n%s Value= \"%s\" \n", LPS25_PRS_DEV_NAME, buf);
#endif

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

#ifdef DEBUG
	pr_info("\n%s Valid val: %lu ", LPS25_PRS_DEV_NAME, val);
#endif

	if (val)
		lps25_prs_enable(prs);
	else
		lps25_prs_disable(prs);

	return size;
}

static ssize_t attr_get_press_ref(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int err;
	struct lps25_prs_data *prs = dev_get_drvdata(dev);
	s32 val = 0;

	mutex_lock(&prs->lock);
	err = lps25_prs_get_press_reference(prs, &val);
	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_press_ref(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	int err = -1;
	struct lps25_prs_data *prs = dev_get_drvdata(dev);
	long val = 0;

	if (strict_strtol(buf, 10, &val))
		return -EINVAL;

	if (val < PR_ABS_MIN || val > PR_ABS_MAX)
		return -EINVAL;


	mutex_lock(&prs->lock);
	err = lps25_prs_set_press_reference(prs, val);
	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;
	return size;
}

static ssize_t attr_get_temperature_ref(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int err;
	struct lps25_prs_data *prs = dev_get_drvdata(dev);
	s16 val = 0;

	mutex_lock(&prs->lock);
	err = lps25_prs_get_temperature_reference(prs, &val);
	mutex_unlock(&prs->lock);
	if (err < 0 )
		return err;

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_temperature_ref(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int err = -1;
	struct lps25_prs_data *prs = dev_get_drvdata(dev);
	long val=0;

	if (strict_strtol(buf, 10, &val))
		return -EINVAL;


	if ( val< TEMP_MIN || val> TEMP_MAX)
		return -EINVAL;


	mutex_lock(&prs->lock);
	err = lps25_prs_set_temperature_reference(prs, val);
	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;
	return size;
}


static int lps25h_prs_autozero_manage(struct lps25_prs_data *prs,
								u8 control)
{
	int err;
	u8 buf[6];
	u8 const mask = AUTOZ_MASK;
	u8 bit_values = AUTOZ_OFF;
	u8 init_val;

	if (control >= AUTOZ_ENABLE) {
		bit_values = AUTOZ_ON;



		buf[0] = CTRL_REG2;
		err = lps25_prs_i2c_read(prs, buf, 1);
		if (err < 0)
			return err;

		init_val = buf[0];
		prs->resume_state[RES_CTRL_REG2] = init_val;

		err = lps25_prs_register_update(prs, buf, CTRL_REG2,
					mask, bit_values);
		if (err < 0)
			return err;
	} else {

		buf[0] = (I2C_AUTO_INCREMENT | P_REF_INDATA_REG);
		buf[1] = 0;
		buf[2] = 0;
		buf[3] = 0;
		buf[4] = 0;
		buf[5] = 0;
		err = lps25_prs_i2c_write(prs, buf, 3);
		if (err < 0)
			return err;
		prs->resume_state[RES_REF_P_XL] = 0;
		prs->resume_state[RES_REF_P_L] = 0;
		prs->resume_state[RES_REF_P_H] = 0;
	}
	return 0;
}

static ssize_t attr_set_autozero(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	int err;
	struct lps25_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&prs->lock);
	err = lps25h_prs_autozero_manage(prs, (u8) val);
	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;
	return size;
}


static ssize_t attr_get_compensation_param(struct device *dev,
				struct device_attribute *attr, char *buf)
{

	struct lps25_prs_data *prs = dev_get_drvdata(dev);
	return sprintf(buf,
			"%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
			prs->TSL,
			prs->TSH,
			prs->TCV1,
			prs->TCV2,
			prs->TCV3,
			prs->TCS1,
			prs->TCS2,
			prs->TCS3,
			prs->digGain,
			prs->deltaT1,
			prs->deltaT2,
			prs->deltaT3,
			prs->testVer
	);
}

static ssize_t attr_set_fifo(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int err= -1;
        struct lps25_prs_data *prs = dev_get_drvdata(dev);
        unsigned long val;
	u8 x[2];

#ifdef DEBUG
	pr_info("\n%s Value= \"%s\" \n", LPS25_PRS_DEV_NAME, buf);
#endif
        if (strict_strtoul(buf, 10, &val))
                return -EINVAL;
#ifdef DEBUG
	pr_info("\n%s Valid val to put in reg2: %lu ", LPS25_PRS_DEV_NAME, val);
#endif 

        if (val) 
		x[1] = 0x40;
        else x[1] = 0x00;
	x[0] = CTRL_REG2;

	mutex_lock(&prs->lock);

	err = lps25_prs_i2c_write(prs, x, 1);
	if (err < 0)
		return err;

	msleep(100);
	x[0] = CTRL_REG2;
	err = lps25_prs_i2c_read(prs, x, 1);
	if (err < 0)
		return err;

	mutex_unlock(&prs->lock);

        return size;

}

static ssize_t attr_fifo_mode(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int err= -1;
	u8 new_val;
	struct lps25_prs_data *prs = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

#ifdef DEBUG
	pr_info("\n%s Valid val: %lu ", LPS25_PRS_DEV_NAME, val);
#endif

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

#ifdef DEBUG
	pr_info("\n%s Valid val: %lu ", LPS25_PRS_DEV_NAME, val);
#endif

	mutex_lock(&prs->lock);
	x[0] = FIFO_CTRL;
	err = lps25_prs_i2c_read(prs, x, 1);
	if (err < 0)
		return err;

//	new_val = ( ((u8)val << 5) | (x[0] & 0x1F) );	//
	new_val = ( ((u8)val << 5) | (x[0] & ~FIFO_MODE_MASK) );

//	x[0] = 0x2E;
	x[0] = FIFO_CTRL;
	x[1] = new_val;		
	err = lps25_prs_i2c_write(prs, x, 1);
	if (err < 0)
		return err;

	msleep(50);
//	x[0] = 0x2E;
	x[0] = FIFO_CTRL;
	err = lps25_prs_i2c_read(prs, x, 1);
	if (err < 0)
		return err;

	mutex_unlock(&prs->lock);
	/*TODO: error need to be managed */
	return size;
}

static ssize_t attr_set_samples_fifo(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int err= -1;
	u8 new_val;
	struct lps25_prs_data *prs = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

#ifdef DEBUG
	pr_info("\n%s Valid val: %lu ", LPS25_PRS_DEV_NAME, val);
#endif

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

#ifdef DEBUG
	pr_info("\n%s Valid val: %lu ", LPS25_PRS_DEV_NAME, val);
#endif

	mutex_lock(&prs->lock);
	x[0] = FIFO_CTRL;
	err = lps25_prs_i2c_read(prs, x, 1);
	if (err < 0)
		return err;

//	new_val = ( (u8)val | (x[0] & 0xE0 ) );	//
	new_val = ( (u8)val | (x[0] & FIFO_MODE_MASK ) );

	x[0] = FIFO_CTRL;
	x[1] = new_val;
	err = lps25_prs_i2c_write(prs, x, 1);
	if (err < 0)
		return err;
	mutex_unlock(&prs->lock);
	/*TODO: error need to be managed */
	return size;
}

#ifdef DEBUG
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct lps25_prs_data *prs = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&prs->lock);
	x[0] = prs->reg_addr;
	mutex_unlock(&prs->lock);
	x[1] = val;
	rc = lps25_prs_i2c_write(prs, x, 1);
	/*TODO: error need to be managed */
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct lps25_prs_data *prs = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&prs->lock);
	data = prs->reg_addr;
	mutex_unlock(&prs->lock);
	rc = lps25_prs_i2c_read(prs, &data, 1);
	/*TODO: error need to be managed */
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lps25_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;
	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&prs->lock);
	prs->reg_addr = val;
	mutex_unlock(&prs->lock);
	return size;
}
#endif



static struct device_attribute attributes[] = {
	__ATTR(poll_period_ms, 0666, attr_get_polling_rate,
							attr_set_polling_rate),
	__ATTR(enable_device, 0666, attr_get_enable, attr_set_enable),
	__ATTR(pressure_reference_level, 0664, attr_get_press_ref,
							attr_set_press_ref),
	__ATTR(temperature_reference_level, 0664, attr_get_temperature_ref,
						attr_set_temperature_ref),
	__ATTR(enable_autozero, 0220, NULL, attr_set_autozero),	
	__ATTR(compensation_param, 0444, attr_get_compensation_param, NULL),
	__ATTR(enable_fifo, 0220, NULL, attr_set_fifo),
	__ATTR(num_samples_fifo, 0220, NULL, attr_set_samples_fifo),
	__ATTR(fifo_mode, 0644, NULL, attr_fifo_mode),
#ifdef DEBUG
	__ATTR(reg_value, 0664, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0220, NULL, attr_addr_set),
#endif
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i, ret;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		ret = device_create_file(dev, attributes + i);
		if (ret < 0)
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return ret;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}


static void lps25_prs_input_work_func(struct work_struct *work)
{
	struct lps25_prs_data *prs = container_of(
			(struct delayed_work *)work,
			struct lps25_prs_data,
			input_work);

	static struct outputdata output;
	int err;

	mutex_lock(&prs->lock);
	err = lps25_prs_get_presstemp_data(prs, &output);
	if (err < 0)
		dev_err(&prs->client->dev, "get_pressure_data failed\n");
	else 
		lps25_prs_report_values(prs, &output);	

	schedule_delayed_work(&prs->input_work,
				msecs_to_jiffies(prs->pdata->poll_interval));
	mutex_unlock(&prs->lock);
}

int lps25_prs_input_open(struct input_dev *input)
{
	struct lps25_prs_data *prs = input_get_drvdata(input);

	return lps25_prs_enable(prs);
}

void lps25_prs_input_close(struct input_dev *dev)
{
	lps25_prs_disable(input_get_drvdata(dev));
}


static int lps25_prs_validate_pdata(struct lps25_prs_data *prs)
{
	/* checks for correctness of minimal polling period */
	prs->pdata->min_interval =
		max((unsigned int)LPS25_PRS_MIN_POLL_PERIOD_MS,
						prs->pdata->min_interval);

	prs->pdata->poll_interval = max(prs->pdata->poll_interval,
					prs->pdata->min_interval);

	/* Checks polling interval relative to minimum polling interval */
	if (prs->pdata->poll_interval < prs->pdata->min_interval) {
		dev_err(&prs->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lps25_prs_input_init(struct lps25_prs_data *prs)
{
	int err;

	INIT_DELAYED_WORK(&prs->input_work, lps25_prs_input_work_func);
	prs->input_dev_pres = input_allocate_device();
	if (!prs->input_dev_pres) {
		err = -ENOMEM;
		dev_err(&prs->client->dev, "input device allocate failed\n");
		goto err0;
	}

	prs->input_dev_pres->open = lps25_prs_input_open;
	prs->input_dev_pres->close = lps25_prs_input_close;
	prs->input_dev_pres->name = LPS25_PRS_DEV_NAME;
	prs->input_dev_pres->id.bustype = BUS_I2C;
	prs->input_dev_pres->dev.parent = &prs->client->dev;


	input_set_drvdata(prs->input_dev_pres, prs);

	set_bit(EV_ABS, prs->input_dev_pres->evbit);

	input_set_abs_params(prs->input_dev_pres, LPS25_ABS_PR,
			PR_ABS_MIN, PR_ABS_MAX, FUZZ, FLAT);
	input_set_abs_params(prs->input_dev_pres, LPS25_ABS_TEMP,
			TEMP_MIN, TEMP_MAX, FUZZ, FLAT);

	//prs->input_dev->name = "LPS25 barometer";

	err = input_register_device(prs->input_dev_pres);
	if (err) {
		dev_err(&prs->client->dev,
			"unable to register input polled device %s\n",
			prs->input_dev_pres->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(prs->input_dev_pres);
err0:
	return err;
}

static void lps25_prs_input_cleanup(struct lps25_prs_data *prs)
{
	input_unregister_device(prs->input_dev_pres);
	/* input_free_device(prs->input_dev);*/
}

static int lps25_prs_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct lps25_prs_data *prs;
	int err = -1;
	u8 buf[5];

	pr_info("%s: probe start.\n", LPS25_PRS_DEV_NAME);

#if 0
        u32 smbus_func = I2C_FUNC_SMBUS_BYTE_DATA |
                        I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK;
#endif
	prs = kzalloc(sizeof(struct lps25_prs_data), GFP_KERNEL);
	if (prs == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for module data: "
					"%d\n", err);
		goto err_exit_alloc_data_failed;
	}

        prs->use_smbus = 0;
        if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
                dev_warn(&client->dev, "client not i2c capable\n");
                if (i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA |
                        I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK)){
                        prs->use_smbus = 1;
                        dev_warn(&client->dev, "client using SMBUS\n");
                } else {
                        err = -ENODEV;
                        dev_err(&client->dev, "client nor SMBUS capable\n");
                        goto err_exit_check_functionality_failed;
                }
        }


#if 0
	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		err = -ENODATA;
		goto err_exit_check_functionality_failed;
	}
#endif
	msleep(100);

	mutex_init(&prs->lock);
	mutex_lock(&prs->lock);

	prs->client = client;
	i2c_set_clientdata(client, prs);

        prs->pdata = kmalloc(sizeof(*prs->pdata), GFP_KERNEL);
        if(prs->pdata == NULL) {
                err = -ENOMEM;
                dev_err(&client->dev,
                        "failed to allocate memory for pdata: %d\n", err);
                goto err_mutexunlockfreedata;
        }
 
        if (client->dev.platform_data == NULL) {
                memcpy(prs->pdata, &default_lps25_pdata,
                                                sizeof(*prs->pdata));
                dev_info(&client->dev, "using default plaform_data for "
                                                "lps25\n");
        } else {
                memcpy(prs->pdata, client->dev.platform_data,
                                                sizeof(*prs->pdata));
                dev_info(&client->dev, "using user plaform_data for "
                                                "lps25\n");
        }


        if (prs->pdata->init) {
                err = prs->pdata->init();
                if (err < 0) {
                        dev_err(&client->dev, "lps pdata init failed: "
                                                                "%d\n", err);
                        goto err_pdata_init;
                }
        }


	/* read chip id */
        buf[0] = 0x0f;
        err = lps25_prs_i2c_read(prs, buf, 1);
	msleep(60);

        if (err < 0) {
                dev_warn(&prs->client->dev, "Error reading WHO_AM_I: is device"
                " available/working?\n");
                goto err_mutexunlockfreedata;
        } else
                prs->hw_working = 1;
 
        if (buf[0] != 0xBD) {
        dev_err(&prs->client->dev,
                "device unknown. Expected: 0x%02x,"
                " Replies: 0x%02x\n", 0xBD, buf[0]);
                err = -1;
                goto err_mutexunlockfreedata;
        }

        pr_info("%s ID Chip OK \n", LPS25_PRS_DEV_NAME);
	msleep(60);

	err = lps25_prs_validate_pdata(prs);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto err_exit_kfree_pdata;
	}


	if (prs->pdata->init) {
		err = prs->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err_exit_pointer;
		}
	}

	memset(prs->resume_state, 0, ARRAY_SIZE(prs->resume_state));
	/* init registers which need values different from zero */
	prs->resume_state[RES_CTRL_REG1] =
		(
			(ENABLE_MASK & PM_NORMAL) |
			(ODR_MASK & ODR_1_1) |
			(BDU_MASK & BDU_ON)
		);

	prs->resume_state[RES_RESOL_CONF] = RES_MAX;
/*
	prs->resume_state[CTRL_REG1] =
				(PM_NORMAL | LPS25_PRS_ODR_1_1 |
					LPS25_PRS_BDU_ON));
	prs->resume_state[CTRL_REG2] = 0x00;
	prs->resume_state[CTRL_REG3] = 0x00;
	prs->resume_state[LPS25_RES_REF_P_L] = 0x00;
	prs->resume_state[LPS25_RES_REF_P_H] = 0x00;
	prs->resume_state[LPS25_RES_THS_P_L] = 0x00;
	prs->resume_state[LPS25_RES_THS_P_H] = 0x00;
	prs->resume_state[LPS25_RES_INT_CFG] = 0x00;
*/

	err = lps25_prs_device_power_on(prs);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err_exit_pointer;
	}

	atomic_set(&prs->enabled, 1);

	err = lps25_prs_update_odr(prs, prs->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto err_power_off;
	}


	err = lps25_prs_input_init(prs);
	if (err < 0) {
		dev_err(&client->dev, "input init failed\n");
		goto err_power_off;
	}

	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev,
			"device LPS25_PRS_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}


	lps25_prs_device_power_off(prs);

	/* As default, do not report information */
	atomic_set(&prs->enabled, 0);


	mutex_unlock(&prs->lock);

	dev_info(&client->dev, "%s: probed\n", LPS25_PRS_DEV_NAME);

	return 0;

/*
err_remove_sysfs_int:
	lps25_prs_remove_sysfs_interfaces(&client->dev);
*/
err_input_cleanup:
	lps25_prs_input_cleanup(prs);
err_power_off:
	lps25_prs_device_power_off(prs);
err_exit_pointer:
	if (prs->pdata->exit)
		prs->pdata->exit();
err_pdata_init:
	if (prs->pdata->exit)
		prs->pdata->exit();
err_exit_kfree_pdata:
	kfree(prs->pdata);

err_mutexunlockfreedata:
	mutex_unlock(&prs->lock);
	kfree(prs);
err_exit_alloc_data_failed:
err_exit_check_functionality_failed:
	pr_err("%s: Driver Init failed\n", LPS25_PRS_DEV_NAME);
	return err;
}

static int __devexit lps25_prs_remove(struct i2c_client *client)
{
	struct lps25_prs_data *prs = i2c_get_clientdata(client);

	lps25_prs_input_cleanup(prs);
	lps25_prs_device_power_off(prs);
	remove_sysfs_interfaces(&client->dev);

	if (prs->pdata->exit)
		prs->pdata->exit();
	kfree(prs->pdata);
	kfree(prs);

	return 0;
}

#ifdef CONFIG_PM

static int lps25_prs_resume(struct i2c_client *client)
{
	struct lps25_prs_data *prs = i2c_get_clientdata(client);

	if (prs->on_before_suspend)
		return lps25_prs_enable(prs);
	return 0;
}

static int lps25_prs_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct lps25_prs_data *prs = i2c_get_clientdata(client);

	prs->on_before_suspend = atomic_read(&prs->enabled);
	return lps25_prs_disable(prs);
}

#else


#endif /* CONFIG_PM */

#ifdef CONFIG_OF
static struct of_device_id lps25_match_table[] = {
	{ .compatible = LPS25_PRS_DEV_NAME,},
	{ },
};
#else
#define lps25_match_table NULL
#endif

static const struct i2c_device_id lps25_prs_id[] = { { LPS25_PRS_DEV_NAME, 0}, { },};
MODULE_DEVICE_TABLE(i2c, lps25_prs_id);

static struct i2c_driver lps25_prs_driver = {
	.driver = {
			.name = LPS25_PRS_DEV_NAME,
			.owner = THIS_MODULE,
        #ifdef CONFIG_OF
	        .of_match_table = lps25_match_table,
		#endif
	},
	.probe = lps25_prs_probe,
	.remove = __devexit_p(lps25_prs_remove),
	.id_table = lps25_prs_id,
	.resume = lps25_prs_resume,
	.suspend = lps25_prs_suspend,
};

static int __init lps25_prs_init(void)
{
#ifdef DEBUG
	pr_debug("%s barometer driver: init\n", LPS25_PRS_DEV_NAME);
#endif
	return i2c_add_driver(&lps25_prs_driver);
}

static void __exit lps25_prs_exit(void)
{
#ifdef DEBUG
	pr_debug("%s barometer driver exit\n", LPS25_PRS_DEV_NAME);
#endif
	i2c_del_driver(&lps25_prs_driver);
	return;
}

module_init(lps25_prs_init);
module_exit(lps25_prs_exit);

MODULE_DESCRIPTION("STMicrolelectronics lps25 pressure sensor sysfs driver");
MODULE_AUTHOR("STMicroelectronics");
MODULE_LICENSE("GPL");

