/**
 * Hillcrest SensorHub driver
 *
 * Copyright (C) 2013-14 Hillcrest Labs, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>
 */


#ifndef __SENSORHUB__
#define __SENSORHUB__

#ifdef __KERNEL__
#include <linux/ioctl.h>
#else
#include <sys/ioctl.h>
#endif


/**
 * Each read() returns a (variable length) sensorhub_input_event structure if
 * SH_IOC_SET_REPORT_METADATA is enabled. The HID input report is placed at the
 * end of the header section and can be accessed with the msg field.
 */
struct sensorhub_input_event {
	// event timestamp in nanoseconds
	__u64 timestamp;
	// variable length msg array
	__u32 len;
	__u8 msg[];
} __attribute__((packed));
/* ioctl Interface */
/* Get sysfs path where attributes are defined */
#define SH_IOC_GET_SYSFS_PATH(len) _IOC(_IOC_READ, 'S', 0x01, len)

/**
 *  Get and Set feature reports
 *  The first byte of the buffer for each op should be the report id
 *  The ioctl returns the length of the bytes read/written from/to the
 *  device.
 */
#define SH_IOC_GET_FEATURE(len)    _IOC(_IOC_READ | _IOC_WRITE, 'S', 0x02, len)
#define SH_IOC_SET_FEATURE(len)    _IOC(_IOC_READ | _IOC_WRITE, 'S', 0x03, len)

/**
 * Enable/disable report metadata.
 * When enabled, each read will be pre-fixed with a struct sensorhub_input_event
 */
#define SH_IOC_SET_REPORT_METADATA _IOW('S', 0x04, int)

#ifdef __KERNEL__

#define SENSORHUB_I2C_ID "sensorhub"
#define SENSORHUB_I2C_ADDR 0x48

struct sensorhub_platform_data {
	/**
	 * The IRQ number for the sensorhub interrupt
	 */
	int irq;
	/**
	 * The GPIO corresponding to the interrupt
	 * Must be configured as an input and pull-up enabled
	 */
	int gpio_interrupt;
	/**
	 * A GPIO to reset the hardware
	 * Must be configured as an output
	 */
	int gpio_reset;
	/**
	 * An optional debug (output) GPIO used by the driver
	 * Must be configured as an output
	 */
	int gpio_debug;
	/**
	 * A wakeup pin for the Sensorhub.
	 * Must be configured as an output.
	 */
	int gpio_wakeup;
	/**
	 * Callback function for any platform initialization/de-init
	 * @param enabled = 1 on first hardware access, 0 on shutdown
	 */
	int (* setup)(struct device *dev, int enabled);
};

#endif /* __KERNEL__ */

#endif  /* __SENSORHUB__ */
