/*
 * Copyright (C) 2010 MEMSIC, Inc.
 *
 * Initial Code:
 *	Robbie Cao
 * 	Dale Hou
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

/* ------------------ */
/* - Include Files. - */
/* ------------------ */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include "mpu-dev.h"

#include <log.h>
#include <linux/mpu.h>
#include "mlsl.h"
#include "mldl_cfg.h"

#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-compass"

#define DEBUG		0

#define MMC328X_I2C_NAME	"mmc328x"
/*
 * This address comes must match the part# on your target.
 * Address to the sensor part# support as following list:
 *	MMC3280MS - 0110000b
 *	MMC3281MS - 0110001b
 *	MMC3282MS - 0110010b
 *	MMC3283MS - 0110011b
 *	MMC3284MS - 0110100b
 *	MMC3285MS - 0110101b
 *	MMC3286MS - 0110110b
 *	MMC3287MS - 0110111b
 * Please refer to sensor datasheet for detail.
 */
#define MMC328X_I2C_ADDR		0x30

/* MMC328X register address */
#define MMC328X_REG_CTRL		0x07
#define MMC328X_REG_DATA		0x00
#define MMC328X_REG_DS			0x06

/* MMC328X control bit */
#define MMC328X_CTRL_TM			0x01
#define MMC328X_CTRL_RM			0x20
#define MMC328X_CTRL_RRM			0x40

#define MAX_FAILURE_COUNT	3
#define READMD			1

#define MMC328X_DELAY_TM	10	/* ms */
#define MMC328X_DELAY_RM	10	/* ms */
#define MMC328X_DELAY_STDN	1	/* ms */

#define MMC328X_RETRY_COUNT	3
#define MMC328X_RESET_INTV	10

#define MMC328X_DEV_NAME	"mmc328x"

//static u32 read_idx = 0;
enum {
    READ_STAT_DATA = 0, //read data
    READ_STAT_RM,   //RM
    READ_STAT_TM    //TM
};

static int read_stat = READ_STAT_DATA;

//static int odata[3] = {4096, 4096, 4096}; //dongfeilong

static int reset_int = 100;//??1000;
static int read_count = 1;


int mmc328x_suspend(void *mlsl_handle,
		    struct ext_slave_descr *slave,
		    struct ext_slave_platform_data *pdata)
{
	int result = INV_SUCCESS;

	return result;
}

int mmc328x_resume(void *mlsl_handle,
		   struct ext_slave_descr *slave,
		   struct ext_slave_platform_data *pdata)
{	
	int result;
#if 1
	result =
	    inv_serial_single_write(mlsl_handle, pdata->address,
				  MMC328X_REG_CTRL, MMC328X_CTRL_RM);
	if (result) {
		LOG_RESULT_LOCATION(result);
		return result;
	}
	msleep(MMC328X_DELAY_RM);
	//inv_sleep(MMC328X_DELAY_RM);
	result =
	    inv_serial_single_write(mlsl_handle, pdata->address,
				  MMC328X_REG_CTRL, MMC328X_CTRL_RRM);
	if (result) {
		LOG_RESULT_LOCATION(result);
		return result;
	}
	msleep(MMC328X_DELAY_RM);
	//inv_sleep(MMC328X_DELAY_RM);
#endif
	result =
	    inv_serial_single_write(mlsl_handle, pdata->address,
				  MMC328X_REG_CTRL, MMC328X_CTRL_TM);
	if (result) {
		LOG_RESULT_LOCATION(result);
		return result;
	}
	read_count = 1;
	read_stat = READ_STAT_DATA;
	return INV_SUCCESS;
}

int mmc328x_read(void *mlsl_handle,
		 struct ext_slave_descr *slave,
		 struct ext_slave_platform_data *pdata,
		 unsigned char *data)
{
    int result, ii;
    short tmp[3];
    unsigned char tmpdata[6];

    if (read_count > 1000)
	read_count = 1;

    if (read_count % reset_int == 0) {
	if(READ_STAT_DATA == read_stat)
	{
	    read_stat = READ_STAT_RM;
	    /* RM */
	    /* not check return value here, assume it always OK */
	    result =
		inv_serial_single_write(mlsl_handle,
			pdata->address,
			MMC328X_REG_CTRL,
			MMC328X_CTRL_RM);
	    msleep(MMC328X_DELAY_RM);
	    /* RRM */
	    /* not check return value here, assume it always OK */
	    result =
		inv_serial_single_write(mlsl_handle,
			pdata->address,
			MMC328X_REG_CTRL,
			MMC328X_CTRL_RRM);		
	    read_stat = READ_STAT_TM;
	    /* wait external capacitor charging done for next RRM */
	    return INV_ERROR_COMPASS_DATA_NOT_READY;
	}
	else if (READ_STAT_TM == read_stat) //TM
	{
	    /* send TM cmd before read */
	    /* not check return value here, assume it always OK */
	    result =
		inv_serial_single_write(mlsl_handle,
			pdata->address,
			MMC328X_REG_CTRL,
			MMC328X_CTRL_TM);

	    read_stat = READ_STAT_DATA;
	    read_count ++; //break "if (read_count % reset_int == 0)"
	    /* wait TM done for coming data read */
	    return INV_ERROR_COMPASS_DATA_NOT_READY;
	}
    }

    result =
	inv_serial_read(mlsl_handle, pdata->address, MMC328X_REG_DS,
		1, (unsigned char *) data);
    if( 0 == (0x01&data[0]) )
    {
	read_count ++;
	return INV_ERROR_COMPASS_DATA_NOT_READY;
    }

    result =
	inv_serial_read(mlsl_handle, pdata->address, MMC328X_REG_DATA,
		6, (unsigned char *) data);
    if (result) {
	LOG_RESULT_LOCATION(result);
	return result;
    }
    for (ii = 0; ii < 6; ii++)
	tmpdata[ii] = data[ii];

    for (ii = 0; ii < 3; ii++) {
	tmp[ii] =
	    (short) ((tmpdata[2 * ii + 1] << 8) + tmpdata[2 * ii]);
//         odata[ii] += ((tmp[ii] - odata[ii]) >> 4);//dongfeilong
//         tmp[ii] = odata[ii];
	if(ii == 2)
		tmp[ii] = 4096 - tmp[ii] ; //?
	else 
		tmp[ii] = tmp[ii] - 4096;
	tmp[ii] = tmp[ii] * 16;   //?
    }
    printk(KERN_INFO"Compass_Data: %d,  %d, %d\n", tmp[0], tmp[1], tmp[2]);

    for (ii = 0; ii < 3; ii++) {
	data[2 * ii] = (unsigned char) (tmp[ii]);
	data[2 * ii + 1] = (unsigned char) (tmp[ii] >> 8);
    }
    /* send TM cmd before read */
    /* not check return value here, assume it always OK */
    result =
	inv_serial_single_write(mlsl_handle,
		pdata->address,
		MMC328X_REG_CTRL,
		MMC328X_CTRL_TM);

    read_stat = READ_STAT_DATA;
    read_count ++;

    return INV_SUCCESS;
}

struct ext_slave_descr mmc328x_descr = {
	/*.init             = */ NULL,
	/*.exit             = */ NULL,
	/*.suspend          = */ mmc328x_suspend,
	/*.resume           = */ mmc328x_resume,
	/*.read             = */ mmc328x_read,
	/*.config           = */ NULL,
	/*.get_config       = */ NULL,
	/*.name             = */ "mmc328xms",
	/*.type             = */ EXT_SLAVE_TYPE_COMPASS,
	/*.id               = */ COMPASS_ID_MMC328X,
	/*.reg              = */ 0x00,
	/*.len              = */ 6,
	/*.endian           = */ EXT_SLAVE_LITTLE_ENDIAN,
	/*.range            = */ {800, 0},
	/*.trigger          = */ NULL,
};

struct ext_slave_descr *mmc328x_get_slave_descr(void)
{
	return &mmc328x_descr;
}
/* -------------------------------------------------------------------------- */
struct mmc328x_mod_private_data {
	struct i2c_client *client;
	struct ext_slave_platform_data *pdata;
};

static unsigned short normal_i2c[] = { I2C_CLIENT_END };

/* OPPO zhuoyh Add begin for compatible with both the compass sensor of ak8975 and mmc328x */
#define MMC328X_I2C_ADRESS 0x30
#define WHO_AM_I_MMC328X 0x06
/* OPPO zhuoyh Add end */

static int mmc328x_mod_probe(struct i2c_client *client,
			   const struct i2c_device_id *devid)
{
	struct ext_slave_platform_data *pdata;
	struct mmc328x_mod_private_data *private_data;
	int result = 0;
       char compass_data;//zhuoyh add

	dev_info(&client->adapter->dev, "%s: %s\n", __func__, devid->name);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		result = -ENODEV;
		goto out_no_free;
	}

	pdata = client->dev.platform_data;
	if (!pdata) {
		dev_err(&client->adapter->dev,
			"Missing platform data for slave %s\n", devid->name);
		result = -EFAULT;
		goto out_no_free;
	}

	private_data = kzalloc(sizeof(*private_data), GFP_KERNEL);
	if (!private_data) {
		result = -ENOMEM;
		goto out_no_free;
	}

	i2c_set_clientdata(client, private_data);
	private_data->client = client;
	private_data->pdata = pdata;

/* OPPO 2012-05-02 dongfeilong Add begin for compatible with both the compass sensor of ak8975 and mmc328x */
	result = inv_serial_read(client->adapter, MMC328X_I2C_ADRESS, WHO_AM_I_MMC328X, 1,  &compass_data);
	if (result == 0) {
		printk(KERN_INFO"%s: the compass sensor is mmc328x!!!!!!!\n", __func__);
	} else {
		result = -EPERM;
		printk(KERN_ERR"%s: error:%d==========NO TO PROBE mmc328x========\n",__func__, result);
		goto out_free_memory;
	}
/* OPPO 2012-05-02 dongfeilong Add end */

	result = inv_mpu_register_slave(THIS_MODULE, client, pdata,
					mmc328x_get_slave_descr);
	if (result) {
		dev_err(&client->adapter->dev,
			"Slave registration failed: %s, %d\n",
			devid->name, result);
		goto out_free_memory;
	}

       printk(KERN_ERR "%s: probe ok!\n", __func__);//zhuoyh
	return result;

out_free_memory:
	kfree(private_data);
out_no_free:
	dev_err(&client->adapter->dev, "%s failed %d\n", __func__, result);
	return result;

}

static int mmc328x_mod_remove(struct i2c_client *client)
{
	struct mmc328x_mod_private_data *private_data =
		i2c_get_clientdata(client);

	dev_dbg(&client->adapter->dev, "%s\n", __func__);

	inv_mpu_unregister_slave(client, private_data->pdata,
				mmc328x_get_slave_descr);

	kfree(private_data);
	return 0;
}

static const struct i2c_device_id mmc328x_mod_id[] = {
	{ "mmc328x", COMPASS_ID_MMC328X },
	{}
};

MODULE_DEVICE_TABLE(i2c, mmc328x_mod_id);

static struct i2c_driver mmc328x_mod_driver = {
	.class = I2C_CLASS_HWMON,
	.probe = mmc328x_mod_probe,
	.remove = mmc328x_mod_remove,
	.id_table = mmc328x_mod_id,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "mmc328x_mod",
		   },
	.address_list = normal_i2c,
};

static int __init mmc328x_mod_init(void)
{
	int res = i2c_add_driver(&mmc328x_mod_driver);
	pr_info("%s: Probe name %s\n", __func__, "mmc328x_mod");
	if (res)
		pr_err("%s failed\n", __func__);
	return res;
}

static void __exit mmc328x_mod_exit(void)
{
	pr_info("%s\n", __func__);
	i2c_del_driver(&mmc328x_mod_driver);
}

module_init(mmc328x_mod_init);
module_exit(mmc328x_mod_exit);

MODULE_AUTHOR("Invensense Corporation");
MODULE_DESCRIPTION("Driver to integrate MMC328X sensor with the MPU");
MODULE_LICENSE("GPL");
MODULE_ALIAS("mmc328x_mod");
/**
 *  @}
**/
