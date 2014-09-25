/*
 $License:
    Copyright (C) 2012 Memsic INC., All Rights Reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
  $
 */



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

#ifdef MPU_SENSORS_MMC3416X 
#define MMC3416x	        1	
#endif

static int reset_int = 1000;
static int read_count = 1;
#define MMC3416X_I2C_ADDRESS          (0x30)
#define MMC3416X_REG_WHO_AM_I         (0x10)
#define MMC3416X_REG_CTRL             (0x07)
#define MMC3416X_REG_DS               (0x06)
#define MMC3416X_REG_DATA             (0x00)
#define MMC3416X_REG_BITS             (0x08)
#define MMC3416X_REG_PRODUCTID0       (0x10)
#define MMC3416X_REG_PRODUCTID1       (0x20)

#define MMC3416X_CTRL_TM	      (0x01)
#define MMC3416X_CTRL_CM              (0x02)

#define MMC3416X_CTRL_50HZ            (0x00)
#define MMC3416X_CTRL_25HZ            (0x04)
#define MMC3416X_CTRL_12HZ            (0x08)
#define MMC3416X_CTRL_NOBOOST         (0x10)
#define MMC3416X_CTRL_SET  	      (0x20)
#define MMC3416X_CTRL_RESET           (0x40)
#define MMC3416X_CTRL_REFILL          (0x80)

#define MMC3416X_BITS_SLOW_16         (0x00)
#define MMC3416X_BITS_FAST_16         (0x01)
#define MMC3416X_BITS_14              (0x02)

#define MMC3416X_DS_TM_READY	      (0x01)

#define MMC3416X_DELAY_TM	      (8)	/* ms */
#define MMC3416X_DELAY_SET	      (50)	/* ms */
#define MMC3416X_DELAY_RESET          (50)     /* ms */

int mmc3416x_suspend(void *mlsl_handle,
		    struct ext_slave_descr *slave,
		    struct ext_slave_platform_data *pdata)
{
	int result = INV_SUCCESS;
	// after data transfer, MMC sleeps automatically

	return result;
}

int mmc3416x_resume(void *mlsl_handle,
		   struct ext_slave_descr *slave,
		   struct ext_slave_platform_data *pdata)
{

	int result;
	result = inv_serial_single_write(mlsl_handle,
                                     pdata->address,
                                     MMC3416X_REG_CTRL,
                                     MMC3416X_CTRL_TM);


	read_count = 1;
	return INV_SUCCESS;
}

int mmc3416x_read(void *mlsl_handle,
		 struct ext_slave_descr *slave,
		 struct ext_slave_platform_data *pdata,
		 unsigned char *data)
{
	int result;
	unsigned char status = 0;
	int md_times = 0;

	if (read_count > 1000)
		read_count = 1;

    //instruct a measurement
    result = inv_serial_single_write(mlsl_handle,
                                     pdata->address,
                                     MMC3416X_REG_CTRL,
                                     MMC3416X_CTRL_TM);
	msleep(MMC3416X_DELAY_TM);

    //wait for the measurement is done
    do{
        result = inv_serial_read(mlsl_handle,
                                 pdata->address,
                                 MMC3416X_REG_DS,
                                 1, &status);


        md_times++;
        if (md_times > 3)
            return INV_ERROR_COMPASS_DATA_NOT_READY;
        msleep(1);
    } while ((status & MMC3416X_DS_TM_READY) != MMC3416X_DS_TM_READY);

	result =
	    inv_serial_read(mlsl_handle, pdata->address, MMC3416X_REG_DATA,
			    6, (unsigned char *)data);


	{
        short tmp[3];
        unsigned char tmpdata[6];
        int ii;

        for (ii = 0; ii < 6; ii++)
            tmpdata[ii] = data[ii];

        for (ii = 0; ii < 3; ii++) {
            tmp[ii] = (short)((tmpdata[2 * ii + 1 ] << 8) + tmpdata[2 * ii]);
            tmp[ii] = tmp[ii] - 32768;
            tmp[ii] = tmp[ii] * 2;   // ??? double check with INVN
        }

        for (ii = 0; ii < 3; ii++) {
            data[2 * ii] = (unsigned char)(tmp[ii] >> 8);
            data[2 * ii + 1] = (unsigned char)(tmp[ii]);
        }

    }

    // reset the RM periodically
	if (read_count % reset_int == 0) {
        result =
            inv_serial_single_write(mlsl_handle,
                        pdata->address,
                        MMC3416X_REG_CTRL,
                        MMC3416X_CTRL_REFILL);

        msleep(MMC3416X_DELAY_SET);
	  result =
            inv_serial_single_write(mlsl_handle,
                        pdata->address,
                        MMC3416X_REG_CTRL,
                        MMC3416X_CTRL_SET);

        msleep(1);
	  result =
            inv_serial_single_write(mlsl_handle,
                        pdata->address,
                        MMC3416X_REG_CTRL,
                        0);

        msleep(1);
        read_count++;
        return INV_ERROR_COMPASS_DATA_NOT_READY;
	}


	read_count++;

	return INV_SUCCESS;
}

struct ext_slave_descr mmc3416x_descr = {
	/*.init             = */ NULL,
	/*.exit             = */ NULL,
	/*.suspend          = */ mmc3416x_suspend,
	/*.resume           = */ mmc3416x_resume,
	/*.read             = */ mmc3416x_read,
	/*.config           = */ NULL,
	/*.get_config       = */ NULL,
	/*.name             = */ "mmc3416x",
	/*.type             = */ EXT_SLAVE_TYPE_COMPASS,
	/*.id               = */ COMPASS_ID_MMC3416X,
	/*.reg              = */ 0x01,
	/*.len              = */ 6,
	/*.endian           = */ EXT_SLAVE_BIG_ENDIAN,
	/*.range            = */ {400, 0},
	/*.trigger          = */ NULL,
};

struct ext_slave_descr *mmc3416x_get_slave_descr(void)
{
	return &mmc3416x_descr;
}

/* -------------------------------------------------------------------------- */
struct mmc3416x_mod_private_data {
	struct i2c_client *client;
	struct ext_slave_platform_data *pdata;
};

static unsigned short normal_i2c[] = { I2C_CLIENT_END };
static int mmc3416x_mod_probe(struct i2c_client *client,
			   const struct i2c_device_id *devid)
{
	struct ext_slave_platform_data *pdata;
	struct mmc3416x_mod_private_data *private_data;
	int result = 0;
       char compass_data;

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


	result = inv_serial_read(client->adapter, MMC3416X_I2C_ADDRESS, MMC3416X_REG_WHO_AM_I, 1,  &compass_data);
	if (result == 0) {
		printk(KERN_INFO"%s: the compass sensor is mmc3416X!!!!!!!\n", __func__);
	} else {
		result = -EPERM;
		printk(KERN_ERR"%s: error:%d==========NO TO PROBE mmc328x========\n",__func__, result);
		goto out_free_memory;
	}


	result = inv_mpu_register_slave(THIS_MODULE, client, pdata,
					mmc3416x_get_slave_descr);
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

static int mmc3416x_mod_remove(struct i2c_client *client){
	struct mmc3416x_mod_private_data *private_data =
		i2c_get_clientdata(client);

	dev_dbg(&client->adapter->dev, "%s\n", __func__);

	inv_mpu_unregister_slave(client, private_data->pdata,
				mmc3416x_get_slave_descr);

	kfree(private_data);
	return 0;
}   
static const struct i2c_device_id mmc3416x_mod_id[]={
	{"mmc3416x",COMPASS_ID_MMC3416X},
	{}
};
MODULE_DEVICE_TABLE(i2c,mmc3416x_mod_id);
static struct i2c_driver mmc3416x_mod_driver = {
	.class = I2C_CLASS_HWMON,
	.probe = mmc3416x_mod_probe,
	.remove = mmc3416x_mod_remove,
	.id_table = mmc3416x_mod_id,
	.driver = {
			.owner = THIS_MODULE,
			.name  = "mmc3416x_mod",
		},
	.address_list = normal_i2c,
};
static int __init mmc3416x_mod_init(void)
{
	int res = i2c_add_driver(&mmc3416x_mod_driver);
	pr_info("%s:Probe name %s\n",__func__,"mmc3416x_mod");
	if(res)
		pr_err("%s failed\n",__func__);
	return res;
}

static void __exit mmc3416x_mod_exit(void)
{
	pr_info("%s\n",__func__);
	i2c_del_driver(&mmc3416x_mod_driver);
}

module_init(mmc3416x_mod_init);
module_exit(mmc3416x_mod_exit);

MODULE_AUTHOR("Invensense Corporation");
MODULE_DESCRIPTION("Driver to integrate MMC3X416 sensor with the MPU");
MODULE_LICENSE("GPL");
MODULE_ALIAS("mmc3416x_mod");
/**
 *  @}
 */
