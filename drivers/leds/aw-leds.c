/*
 * Copyright (C) 2014 The CyanogenMod Project. All Rights Reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/leds-aw-leds.h>

#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES			5

#define AW_LED_RSTR    (0x00)
#define AW_LED_GCR     (0x01)
#define AW_LED_STATUS  (0x02)
#define AW_LED_EN      (0x30)
#define AW_LED_CTRL(x) (0x31 + x)
#define AW_LED_PWM(x)  (0x34 + x)
#define AW_LED_T0(x)   (0x37 + 3 * (x))
#define AW_LED_T1(x)   (0x38 + 3 * (x))
#define AW_LED_T2(x)   (0x39 + 3 * (x))

#define AW_LED_ENABLE(x) (1 << x)

#define Rise_time   0x00
#define Hold_time   0x01//holdtime 0.26s
#define Fall_time   0x00
#define Off_time    0x04//2.08s
#define Delay_time  0x20//0.26S
#define Period_Num  0x00

#define AW_LED_INVALID -1
#define AW_LED_RED      0
#define AW_LED_GREEN    1
#define AW_LED_BLUE     2

struct aw_leds_data {
	struct i2c_client *client;
	struct miscdevice dev;
	struct mutex mutex;
	u32 gpio;
	struct regulator *vana;
	u32 leds[3];
};

static int aw_leds_i2c_write_reg(struct i2c_client *client,
		u8 regnum, u8 value)
{
	int32_t ret = -1;
	uint8_t tries = 0;

	do{
		ret = i2c_smbus_write_byte_data(client, regnum, value);

		if (ret) {
			msleep_interruptible(I2C_RETRY_DELAY);
		}
	} while((ret != 0) && (++tries < I2C_RETRIES));

	return ret;
}

static int aw_leds_configure(struct aw_leds_data *aw_leds,
		struct aw_leds_control *cfg)
{
	struct i2c_client *client = aw_leds->client;
	u8 reg, en = 0;
	int i;

	dev_dbg(&client->dev,
		"%s: red %d, green %d, blue %d, blink %d\n",
		__func__, cfg->red, cfg->green, cfg->blue, cfg->blink);

	aw_leds_i2c_write_reg(client, AW_LED_EN, 0x00);
	aw_leds_i2c_write_reg(client, AW_LED_GCR, 0x01);

	if (cfg->red | cfg->green | cfg->blue) {
		for (i = 0; i < 3; i++) {
			//TODO: Fix blink.  If colors are set < 255,
			//the blink durations change
			aw_leds_i2c_write_reg(client, AW_LED_CTRL(i),
				cfg->blink ? 0x11 : 0x03);
			//TODO: Make these configurable
			aw_leds_i2c_write_reg(client, AW_LED_T0(i),
				Rise_time << 4 | Hold_time);
			aw_leds_i2c_write_reg(client, AW_LED_T1(i),
				Fall_time << 4 | Off_time);
			aw_leds_i2c_write_reg(client, AW_LED_T2(i),
				Delay_time);

			if (aw_leds->leds[i] == AW_LED_RED)
				reg = cfg->red;
			else if (aw_leds->leds[i] == AW_LED_GREEN)
				reg = cfg->green;
			else if (aw_leds->leds[i] == AW_LED_BLUE)
				reg = cfg->blue;
			else
				reg = 0;
			aw_leds_i2c_write_reg(client, AW_LED_PWM(i), reg);
			en |= reg ? AW_LED_ENABLE(i) : 0;
		}

		aw_leds_i2c_write_reg(client, AW_LED_EN, en);
	}
	return 0;
}

static long aw_leds_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	struct aw_leds_data *leds_data = container_of(file->private_data,
			struct aw_leds_data, dev);
	struct i2c_client *client = leds_data->client;
	struct aw_leds_control cfg;
	int ret;

	mutex_lock(&leds_data->mutex);
	switch(cmd) {
	case AW_LEDS_SET_LEDS:
		ret = copy_from_user(&cfg, (void __user *)arg,
						sizeof(struct aw_leds_control));
		if (ret)
			break;

		ret = aw_leds_configure(leds_data, &cfg);
		break;
	default:
		dev_err(&client->adapter->dev,
			"%s: Unknown cmd %x, arg %lu\n",
			__func__, cmd, arg);
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&leds_data->mutex);
	return ret;
}

static int aw_leds_open(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations aw_leds_fops = {
	.owner = THIS_MODULE,
	.open = aw_leds_open,
	.unlocked_ioctl = aw_leds_ioctl,
};

static int aw_leds_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;
	struct aw_leds_data *data;
	const struct device_node *np = client->dev.of_node;

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_WRITE_BYTE | I2C_FUNC_SMBUS_READ_BYTE_DATA))
	{
		dev_err(&client->dev, "need I2C_FUNC_SMBUS*.\n");
		return -ENODEV;
	}

	data = kzalloc(sizeof(struct aw_leds_data),GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}

	if (of_property_read_u32(np, "aw_leds,gpio", &data->gpio)) {
		dev_err(&client->dev, "failed to read aw_leds,gpio from dt\n");
		goto exit;
	}

	data->leds[0] = AW_LED_INVALID;
	if (of_property_read_u32(np, "aw_leds,led0", &data->leds[0])) {
		dev_warn(&client->dev, "failed to read aw_leds,led0 from dt\n");
	}

	data->leds[1] = AW_LED_INVALID;
	if (of_property_read_u32(np, "aw_leds,led1", &data->leds[1])) {
		dev_warn(&client->dev, "failed to read aw_leds,led1 from dt\n");
	}

	data->leds[2] = AW_LED_INVALID;
	if (of_property_read_u32(np, "aw_leds,led2", &data->leds[2])) {
		dev_warn(&client->dev, "failed to read aw_leds,led2 from dt\n");
	}

	dev_dbg(&client->dev, "leds: %d %d %d\n",
		data->leds[0], data->leds[1], data->leds[2]);

	ret = gpio_request(data->gpio, "aw_leds-gpio");
	if (ret) {
		dev_err(&client->dev, "failed to request gpio %d\n", data->gpio);
		goto exit;
	}

	gpio_direction_output(data->gpio, 1);
	dev_dbg(&client->dev, "using gpio %d\n", data->gpio);

	data->vana = regulator_get(&client->dev, "vdd_ana");
	if (IS_ERR(data->vana)) {
		ret = PTR_ERR(data->vana);
		dev_err(&client->dev, "regulator get vdd_ana failed\n");
		goto reg_failed;
	}

	ret = regulator_enable(data->vana);
	if (ret) {
		dev_err(&client->dev, "regulator vdd_ana enable failed\n");
		goto reg_en_failed;
	}

	data->client = client;
	i2c_set_clientdata(client, data);
	mutex_init(&data->mutex);

	data->dev.minor = MISC_DYNAMIC_MINOR;
	data->dev.name = "aw-leds";
	data->dev.fops = &aw_leds_fops;
	ret = misc_register(&data->dev);
	if (ret < 0) {
		dev_err(&client->adapter->dev, "misc_register returned %d\n", ret);
		goto register_failed;
	}

	dev_dbg(&client->dev, "success\n");
	return 0;
register_failed:
	mutex_destroy(&data->mutex);
reg_en_failed:
	regulator_put(data->vana);
reg_failed:
	gpio_free(data->gpio);
exit:
	kfree(data);
	return ret;
}

static int aw_leds_remove(struct i2c_client *client)
{
	struct aw_leds_data *data = i2c_get_clientdata(client);
	dev_dbg(&client->adapter->dev, "%s\n", __func__);
	misc_deregister(&data->dev);
	mutex_destroy(&data->mutex);
	regulator_put(data->vana);
	gpio_free(data->gpio);
	kfree(data);
	return 0;
}

static const struct i2c_device_id aw_leds_id[]= {
	{"aw-leds", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c,aw_leds_id);

static const struct of_device_id aw_of_match[] = {
	{ .compatible = "aw-leds", },
	{ },
};
MODULE_DEVICE_TABLE(of, aw_of_match);

static struct i2c_driver aw_leds_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "aw-leds",
		.of_match_table = aw_of_match,
	},
	.id_table = aw_leds_id,
	.probe = aw_leds_probe,
	.remove = aw_leds_remove,

};

static int __init aw_leds_init(void)
{
	return i2c_add_driver(&aw_leds_driver);
}
static void __exit aw_leds_exit(void)
{
	i2c_del_driver(&aw_leds_driver);
}

module_init(aw_leds_init);
module_exit(aw_leds_exit);

MODULE_DESCRIPTION("AW RGB LED driver");
MODULE_AUTHOR("Cyanogen Inc.");
MODULE_LICENSE("GPL");
