/*!
 * @section LICENSE
 * (C) Copyright 2013 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename bme280_core.h
 * @date     "Wed Feb 19 16:27:25 2014 +0800"
 * @id       "7f466fb"
 *
 * @brief
 * The head file of BME280 device driver core code
*/
#ifndef _BME280_CORE_H
#define _BME280_CORE_H

#include "bme280.h"
#include "bs_log.h"

/*! @defgroup bme280_core_inc
 *  @brief The head file of BME280 device driver core code
 @{*/
/*! define BME device name */
#define BME_NAME "bme280"

/*! define BME register name according to API */
#define BME_REG_NAME(name) BME280_##name
/*! define BME value name according to API */
#define BME_VAL_NAME(name) BME280_##name
/*! define BME hardware-related function according to API */
#define BME_CALL_API(name) bme280_##name
/*! only for debug */
#define DEBUG_BME280

/*!
 * @brief bus communication operation
*/
struct bme_bus_ops {
	/*!write pointer */
	BME280_WR_FUNC_PTR;
	/*!read pointer */
	BME280_RD_FUNC_PTR;
};

/*!
 * @brief bus data client
*/
struct bme_data_bus {
	/*!bus communication operation */
	const struct bme_bus_ops *bops;
	/*!bme client */
	void *client;
};

int bme_probe(struct device *dev, struct bme_data_bus *data_bus);
int bme_remove(struct device *dev);
#ifdef CONFIG_PM
int bme_enable(struct device *dev);
int bme_disable(struct device *dev);
#endif

#endif/*_BME280_CORE_H*/
/*@}*/
