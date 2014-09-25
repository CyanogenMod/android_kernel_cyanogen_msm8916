/*!
 * @section LICENSE
 * (C) Copyright 2013 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename bme280_spi.c
 * @date     "Wed Feb 19 16:27:25 2014 +0800"
 * @id       "7f466fb"
 *
 * @brief
 * This file implements moudle function, which add
 * the driver to SPI core.
*/

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include "bme280_core.h"
#include "bs_log.h"

/*! @defgroup bme280_spi_src
 *  @brief bme280 spi driver module
 @{*/
/*! the maximum of transfer buffer size */
#define BME_MAX_BUFFER_SIZE      32

static struct spi_device *bme_spi_client;

/*!
 * @brief define spi wirte function
 *
 * @param dev_addr sensor device address
 * @param reg_addr register address
 * @param data the pointer of data buffer
 * @param len block size need to write
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static char bme_spi_write_block(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	struct spi_device *client = bme_spi_client;
	u8 buffer[BME_MAX_BUFFER_SIZE + 1];
	struct spi_transfer xfer = {
		.tx_buf     = buffer,
		.len        = len + 1,
	};
	struct spi_message msg;

	if (len > BME_MAX_BUFFER_SIZE)
		return -EINVAL;

	buffer[0] = reg_addr&0x7F;/* write: MSB = 0 */
	memcpy(&buffer[1], data, len);

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	return spi_sync(client, &msg);
}

/*!
 * @brief define spi read function
 *
 * @param dev_addr sensor device address
 * @param reg_addr register address
 * @param data the pointer of data buffer
 * @param len block size need to read
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static char bme_spi_read_block(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	struct spi_device *client = bme_spi_client;
	u8 reg = reg_addr | 0x80;/* read: MSB = 1 */
	struct spi_transfer xfer[2] = {
		[0] = {
			.tx_buf = &reg,
			.len = 1,
		},
		[1] = {
			.rx_buf = data,
			.len = len,
		}
	};
	struct spi_message msg;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer[0], &msg);
	spi_message_add_tail(&xfer[1], &msg);
	return spi_sync(client, &msg);
}

/*!
 * @brief spi bus operation
*/
static const struct bme_bus_ops bme_spi_bus_ops = {
	/**< spi block write pointer */
	.bus_write  = bme_spi_write_block,
	/**< spi block read pointer */
	.bus_read   = bme_spi_read_block
};

/*!
 * @brief BME probe function via spi bus
 *
 * @param client the pointer of spi client
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static int __devinit bme_spi_probe(struct spi_device *client)
{
	int status;
	struct bme_data_bus data_bus = {
		.bops = &bme_spi_bus_ops,
		.client = client
	};

	if (NULL == bme_spi_client)
		bme_spi_client = client;
	else{
		PERR("This driver does not support multiple clients!\n");
		return -EINVAL;
	}

	client->bits_per_word = 8;
	status = spi_setup(client);
	if (status < 0) {
		PERR("spi_setup failed!\n");
		return status;
	}

	return bme_probe(&client->dev, &data_bus);
}

/*!
 * @brief shutdown bme device in spi driver
 *
 * @param client the pointer of spi client
 *
 * @return no return value
*/
static void bme_spi_shutdown(struct spi_device *client)
{
#ifdef CONFIG_PM
	bme_disable(&client->dev);
#endif
}

/*!
 * @brief remove bme spi client
 *
 * @param client the pointer of spi client
 *
 * @return zero
 * @retval zero
*/
static int bme_spi_remove(struct spi_device *client)
{
	return bme_remove(&client->dev);
}

#ifdef CONFIG_PM
/*!
 * @brief suspend bme device in spi driver
 *
 * @param dev the pointer of device
 *
 * @return zero
 * @retval zero
*/
static int bme_spi_suspend(struct device *dev)
{
	return bme_disable(dev);
}

/*!
 * @brief resume bme device in spi driver
 *
 * @param dev the pointer of device
 *
 * @return zero
 * @retval zero
*/
static int bme_spi_resume(struct device *dev)
{
	return bme_enable(dev);
}

/*!
 * @brief register spi device power manager hooks
*/
static const struct dev_pm_ops bme_spi_pm_ops = {
	/**< device suspend */
	.suspend = bme_spi_suspend,
	/**< device resume */
	.resume  = bme_spi_resume
};
#endif

/*!
 * @brief register spi device id
*/
static const struct spi_device_id bme_id[] = {
	{ BME_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, bme_id);

/*!
 * @brief register spi driver hooks
*/
static struct spi_driver bme_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = BME_NAME,
#ifdef CONFIG_PM
		.pm = &bme_spi_pm_ops,
#endif
	},
	.id_table = bme_id,
	.probe    = bme_spi_probe,
	.shutdown = bme_spi_shutdown,
	.remove   = __devexit_p(bme_spi_remove)
};

/*!
 * @brief initialize bme spi module
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static int __init bme_spi_init(void)
{
	return spi_register_driver(&bme_spi_driver);
}

/*!
 * @brief remove bme spi module
 *
 * @return no return value
*/
static void __exit bme_spi_exit(void)
{
	spi_unregister_driver(&bme_spi_driver);
}


MODULE_DESCRIPTION("BME280 SPI BUS DRIVER");
MODULE_LICENSE("GPL v2");

module_init(bme_spi_init);
module_exit(bme_spi_exit);
/*@}*/
