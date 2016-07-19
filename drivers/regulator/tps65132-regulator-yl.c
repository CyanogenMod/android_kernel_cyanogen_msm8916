#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/types.h>
#include <linux/vmalloc.h>
#include <linux/input.h>
#include <linux/i2c.h>

#define TPS_DRIVER_NAME "tps651321"
#define COMPATIBLE_NAME "qcom,tps-651321"
struct i2c_client* client1 = NULL;

static int tps65132_i2c_write(struct i2c_client* client, uint8_t reg, uint8_t data)
{	
	unsigned char buffer[2];
	buffer[0] = reg;
	buffer[1] = data;
	pr_debug("--tps--%s--reg=0x%x--data=0x%x--\n", __func__, reg, data);
	if(2!= i2c_master_send(client, buffer, 2))
	{	
		printk("--tps--tps65132_i2c_write fail--!!!--\n");
		return -1;
	}
	
	return 0;
}

void tps65132_config_init(void)
{
	tps65132_i2c_write(client1, 0x00, 0x0A);	//0x0A : +5V, 0x0F : +5.5V
	msleep(1);
	tps65132_i2c_write(client1, 0x01, 0x0A);	//0x0A : -5V, 0x0F : -5.5V
}

void tps65132_config_set_to_tablet_mode(void)
{
	tps65132_i2c_write(client1, 0x03, 0x43);	//set to tablet mode
}

void tps65132_config_proc(void)
{
	tps65132_i2c_write(client1, 0x00, 0x00);	//0x00 : +4V
	tps65132_i2c_write(client1, 0x01, 0x00);	//0x00 : -4V
	msleep(3);
	tps65132_i2c_write(client1, 0x00, 0x0A);	//0x0A : +5V
	tps65132_i2c_write(client1, 0x01, 0x0A);	//0x0A : -5V
}

static int tps_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	client1 = client;
	printk("--tps65132--enter--TPS65132 I2C Address:0x%02x--!\n", client->addr);

	tps65132_config_init();

	
	return 0;
}

//static __devexit int tps65132_remove(struct i2c_client *client)
//{
//	return 0;
//}

static struct of_device_id tps_match_table[] = {
	{.compatible = COMPATIBLE_NAME,},
	{ },
};
static struct i2c_device_id tps_i2c_id[] = {
	{ TPS_DRIVER_NAME, 0 },
	{ }
};


MODULE_DEVICE_TABLE(i2c, tps_i2c_id);

static struct i2c_driver tps65132_driver = {
	.driver = {
		.name = TPS_DRIVER_NAME,
		.of_match_table = tps_match_table,
	},
	.probe = tps_i2c_probe,
	.id_table = tps_i2c_id,
	//.remove = __devexit_p(tps65132_remove),
};

static int __init tps65132_init(void)
{
	int r;

	printk("--tps--: %s\n", __func__);

	r = i2c_add_driver(&tps65132_driver);
	if (r) {
		pr_err("--tps--:tps65132 driver registration failed\n");
		return r;
	}

	return 0;
}

static void __exit tps65132_exit(void)
{
	printk("--tps--: %s\n", __func__);
	i2c_del_driver(&tps65132_driver);
}

module_init(tps65132_init);
module_exit(tps65132_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
