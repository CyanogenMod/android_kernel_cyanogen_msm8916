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

#ifndef __SENSORHUB_MAIN__
#define __SENSORHUB_MAIN__

#include <linux/i2c.h>
#include <linux/kfifo.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/sensors/sensorhub.h>
#include <linux/time.h>

#define SENSORHUB_FIFO_REC_SIZE 1
#define SENSORHUB_FIFO_SIZE PAGE_SIZE /* has to be power of 2 */

#define SENSORHUB_MAX_REPORT_DESCRIPTOR_SIZE 8 * 1024  /* 8k is more than enough for any of us*/

/* Flag is set while a reset is pending */
#define SENSOR_HUB_RESET_PENDING        1
/* Flag is set while driver is suspended */
#define SENSOR_HUB_SUSPENDED            2
/* Flag is set after userspace interface is shutdown and no longer accepting connectionts */
#define SENSOR_HUB_DISCONNECTED         3
#define SENSOR_HUB_DFU_IN_FLIGHT        16
#define SENSOR_HUB_FLAGS_LAST           SENSOR_HUB_DFU_IN_FLIGHT

enum sensorhub_report_type {
	SENSORHUB_REPORT_INPUT,
	SENSORHUB_REPORT_OUTPUT,
	SENSORHUB_REPORT_FEATURE
};

enum sensorhub_state {
	SENSORHUB_STATE_UNKNOWN,
	SENSORHUB_STATE_APP,
	SENSORHUB_STATE_BOOTLOADER,
	SENSORHUB_STATE_BOOTLOADER_NOIMAGE,
	SENSORHUB_STATE_BOOTLOADER_HUNG_APP,
	SENSORHUB_STATE_DOWNLOAD,
	SENSORHUB_STATE_DOWNLOAD_ENTER,
	SENSORHUB_STATE_DOWNLOAD_FAILED,
};

enum sensorhub_reset {
	SENSORHUB_RESET_SOFT,
	SENSORHUB_RESET_HARD
};


struct sensorhub_drv_data;
struct sensorhub_transport_callbacks;
struct sensorhub_client;

/**
 * Register a new sensorhub device
 *
 * @param  ddata location to save ddata pointer to
 * @return       0 on success, 1 if buffers were full and a message was dropped, -errno on error
 */
int sensorhub_core_register(struct sensorhub_drv_data * ddata);

/**
 * Unregister a sensorhub device
 *
 * @param  ddata Driver data
 */
void sensorhub_core_unregister(struct sensorhub_drv_data * ddata);


/**
 * Deliver a single input packet to listeners
 * The wait queue has to be woken after input(s) are delivered
 * @param  ddata Driver data
 * @param  input input event *metadata only* (pass message buffer as next argument)
 * @param  buf   the input report, the first byte must be the report id
 * @param  len   the length of report
 * @return       0 on success, 1 if buffers were full and (older) message(s) were
 *               dropped, -errno on error
 */
int sensorhub_core_notify_input(struct sensorhub_drv_data * ddata,
                                const struct sensorhub_input_event * input,
                                const u8 * buf, size_t len);

/**
 * Call when a hardware reset is detected
 * @param  ddata Driver data
 * @return       0 on success, -errno on error
 */
int sensorhub_core_notify_reset(struct sensorhub_drv_data * ddata);

/**
 * Resets the hardware
 * @param  ddata Driver data
 * @param  reset Reset method
 * @param  wait  Wait upto predefined timeout for hwreset to complete
 * @return       0 on success, -errno on error
 */
int sensorhub_core_reset(struct sensorhub_drv_data * ddata, enum sensorhub_reset reset, bool wait);

/**
 * Used during module unload (and debug), forces a hangup of all clients/users
 * @param  ddata Driver data
 * @return       0 on success, -errno on error
 */
int sensorhub_core_hangup(struct sensorhub_drv_data * ddata);

/**
 * Initiate a firmware update on the hardware
 *
 * @param  ddata Driver data
 * @return       0 on success, -errno on error
 */
int sensorhub_core_dfu_request(struct sensorhub_drv_data * ddata);

/**
 * Common sensorhub driver data structure
 */
struct sensorhub_drv_data {

	// These fields can be accessed by core and transport driver
	char name[16];
	unsigned long flags;
	enum sensorhub_state state;

	/**
	 * wait queue for any message from the device
	 */
	wait_queue_head_t wait;
	struct {
		// TODO add a spin lock to access these stats
		unsigned int received;
		unsigned int receivedBytes;
		unsigned int sent;
		unsigned int sentBytes;
		unsigned int dropped;
		unsigned int resetCount;
		unsigned int errorCount;
		unsigned int interrupts;
		unsigned int suspendCount;
		unsigned int retries;
	} stats;

	// The following fields should be considered private
	struct miscdevice misc;
	struct sensorhub_platform_data pdata;
	struct list_head clients;
	struct mutex clients_lock; /* take this lock before accessing clients */

	// The remainging fields below must be set by the transport driver
	// before sensorhub_core_register is called
	unsigned int vendorID;
	unsigned int productID;
	unsigned int versionID;
	/**
	 * Shortcut to misc.this_device
	 */
	struct device * dev;
	/**
	 * Callback functions to the underlying transport driver
	 * Set this field before calling register
	 */
	const struct sensorhub_transport_callbacks * transport;

	/**
	 * The HID report descriptor.
	 **/
	struct {
		size_t len;
		void * buf;
	} report_descriptor;
};

/**
 * Strucure associated with each open file/client
 */
struct sensorhub_client {
	spinlock_t lock; /* take this lock before acessing fields other than ddata, list and queue */
	struct sensorhub_drv_data * ddata;
	struct fasync_struct *fasync;

	struct list_head list;
	STRUCT_KFIFO_REC_1(SENSORHUB_FIFO_SIZE) queue;
	unsigned int resetCount;
	bool report_metadata;
};

/**
 * The sensorhub hardware interface must implement these callback functions
 * for use by sensorhub core.
 */
struct sensorhub_transport_callbacks {
	/**
	 * Get a report
	 * Called only from process context.
	 * @param  ddata   Driver data
	 * @param  type    The report type (input or feature)
	 * @param  id      The report id
	 * @param  report  Buffer to get feature report to
	 * @param  len     Length available in buffer
	 * @return         length of report data, -errno otherwise
	 */
	ssize_t (* get_report)(struct sensorhub_drv_data * ddata,
						enum sensorhub_report_type type, u8 id,
						u8 * report, size_t len);

	/**
	 * Set a report
	 * Called only from process context.
	 * @param  ddata   Driver data
	 * @param  type    The report type (output or feature)
	 * @param  id      The report id
	 * @param  report  Buffer containing report to set
	 * @param  len     Length available in buffer
	 * @return         length of report data, -errno otherwise
	 */
	ssize_t (* set_report)(struct sensorhub_drv_data * ddata,
						enum sensorhub_report_type type, u8 id,
						const u8 * report, size_t len);

	/**
	 * Reset the device.
	 * The driver must clear the SENSOR_HUB_RESET_PENDING flag when the
	 * device has completed the reset and wake the wait queue.
	 *
	 * Called only from process context.
	 *
	 * @param  ddata Driver data
	 * @param  reset Reset method
	 * @return       0 on success, -errno otherwise
	 */
	int (* reset)(struct sensorhub_drv_data * ddata, enum sensorhub_reset reset);

	/**
	 * Perform a firmware download to the device.
	 * Validation of the firmware file format header is performed by core
	 * and dfu is called only after common checks are passed. The dfu function
	 * can perform further device specific checks.
	 *
	 * This operation is not interruptible and will block until the download
	 * completes.
	 *
	 * This function must ensure that the application is running before returning
	 *
	 * Called only from process context.
	 *
	 * @param  ddata Driver data
	 * @param  fw    The device specific firmware data
	 * @param  len   Length of firmware
	 * @return       0 on success, -errno otherwise
	 */
	int (* dfu_perform)(struct sensorhub_drv_data * ddata, const u8 * fw, size_t len);

	/**
	 * The use cancelled a dfu after calling prepare()
	 * Update ddata->state with an appropriate mode after transitioning from the
	 * download state.
	 *
	 * @param  ddata Driver data
	 * @return       0 on success, -errno otherwise
	 */
	void (* dfu_abort)(struct sensorhub_drv_data * ddata);

	/**
	 * Validates a firmware file before starting a download
	 * @param  fw    The device specific format
	 * @param  len   Length of firmware
	 * @return       0 on success, -errno otherwise
	 */
	int (* dfu_validate)(struct sensorhub_drv_data * ddata, const u8 * fw, size_t len);
};

/**
 * The header format for a Hillcrest firmware image
 */
struct sensorhub_dfu_image_header {
	u32 id;
	u32 sz;
	u32 ffVer;
	u32 payload;
	char metadata[];
} __packed;


/**
 * The trailer format for a Hillcrest firmware image
 */
struct sensorhub_dfu_image_footer {
	u32 crc32;
} __packed;

#define HCREST_DFU_IMAGE_FILE_ID 0x6572d028
#define HCREST_DFU_IMAGE_FF_VER  4

/**
 * sensorhub_get_time_ns() - utility function for timestamps
 **/
static inline __u64 sensorhub_get_time_ns(void)
{
	struct timespec ts;
	ktime_get_ts(&ts);

	return timespec_to_ns(&ts);
}

#endif
