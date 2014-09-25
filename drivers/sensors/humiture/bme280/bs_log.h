/*!
 * @section LICENSE
 * (C) Copyright 2013 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename bs_log.h
 * @date     "Fri Dec 13 14:33:28 2013 +0800"
 * @id       "ba28356"
 *
 * @brief
 * The head file of BOSCH SENSOR LOG
*/

#ifndef __BS_LOG_H
#define __BS_LOG_H

#include <linux/kernel.h>

/*! @defgroup bme280_core_src
 *  @brief The core code of BME280 device driver
 @{*/
/*! ERROR LOG LEVEL */
#define LOG_LEVEL_E 3
/*! NOTICE LOG LEVEL */
#define LOG_LEVEL_N 5
/*! INFORMATION LOG LEVEL */
#define LOG_LEVEL_I 6
/*! DEBUG LOG LEVEL */
#define LOG_LEVEL_D 7

#ifndef LOG_LEVEL
/*! LOG LEVEL DEFINATION */
#define LOG_LEVEL LOG_LEVEL_D
#endif

#ifndef MODULE_TAG
/*! MODULE TAG DEFINATION */
#define MODULE_TAG "BME280:YLLOG:"
#endif

#if (LOG_LEVEL >= LOG_LEVEL_E)
/*! print error message */
#define PERR(fmt, args...) \
	printk(KERN_INFO "\n" "[E]" KERN_ERR MODULE_TAG \
	"<%s><%d>" fmt "\n", __func__, __LINE__, ##args)
#else
/*! invalid message */
#define PERR(fmt, args...)
#endif

#if (LOG_LEVEL >= LOG_LEVEL_N)
/*! print notice message */
#define PNOTICE(fmt, args...) \
	printk(KERN_INFO "\n" "[N]" KERN_NOTICE MODULE_TAG \
	"<%s><%d>" fmt "\n", __func__, __LINE__, ##args)
#else
/*! invalid message */
#define PNOTICE(fmt, args...)
#endif

#if (LOG_LEVEL >= LOG_LEVEL_I)
/*! print information message */
#define PINFO(fmt, args...) printk(KERN_INFO "\n" "[I]" KERN_INFO MODULE_TAG \
	"<%s><%d>" fmt "\n", __func__, __LINE__, ##args)
#else
/*! invalid message */
#define PINFO(fmt, args...)
#endif

#if (LOG_LEVEL >= LOG_LEVEL_D)
/*! print debug message */
#define PDEBUG(fmt, args...) printk(KERN_INFO "\n" "[D]" KERN_DEBUG MODULE_TAG \
	"<%s><%d>" fmt "\n", __func__, __LINE__, ##args)
#else
/*! invalid message */
#define PDEBUG(fmt, args...)
#endif

#endif/*__BS_LOG_H*/
/*@}*/
