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

#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>

#ifdef CONFIG_OF
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

#include "sensorhub-core.h"
#include "sensorhub-i2c.h"

#undef pr_fmt
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#define CHECK_LEN(req) \
	if (len < req) return -ETOOSMALL;

#define SENSORHUB_SYSFS_DEBUG

#define GET_REPORT_BUFSIZE (64)
#define SET_REPORT_BUFSIZE (64)
#define MAX_INPUT_REPORT_BUFSIZE (64)

 // Duration for SH-1 app to launch and be ready to serve hid desc
#define APP_STARTUP_MAX_DELAY_MS       2500
#define APP_STARTUP_CHECK_INTERVAL_MS  25

// Duration for keeping reset low per reset
#define RESET_TOGGLE_US 1000

inline struct sensorhub_i2c_data * __i2c(struct sensorhub_drv_data * _ddata) {
	return container_of(_ddata, struct sensorhub_i2c_data, ddata);
}

inline void sensorhub_notify_state(struct sensorhub_drv_data * ddata) {
	sysfs_notify(&ddata->dev->kobj, NULL, "state");
}

// Write u16 val to u8 pointer with offset *idx
inline void cpu_to_le16_u8p(u16 val, u8 * p, size_t * idx) {
	val = cpu_to_le16(val);
	*((u16 *) (p + *idx)) = val;
	*idx += 2;
}

#define set_debug_gpio(val)\
	if (ddata->pdata.gpio_debug) { \
		gpio_set_value(ddata->pdata.gpio_debug, val); \
	}

// Forward declarations:
static int sensorhub_i2c_test_and_read_locked(struct sensorhub_i2c_data * i2cdata);
static int sensorhub_i2c_toggle_reset(struct sensorhub_drv_data * ddata,
									unsigned int reps,
									unsigned int toggle_us,
									unsigned int rep_ms);

static int sensorhub_i2c_poll_handshake(struct sensorhub_i2c_data * i2cdata);
static int sensorhub_i2c_init_app(struct sensorhub_i2c_data * i2cdata);
// return 0 if successfully exited bootloader
//      > 0 if bootloader is missing or there is no application image
//      < 0 of an error or unexpected condition occured
static int sensorhub_i2c_bl_detect_and_exit(struct sensorhub_i2c_data * i2cdata);

static const struct sensorhub_transport_callbacks i2c_sensorhub_transport;
#ifdef SENSORHUB_SYSFS_DEBUG
static const struct attribute_group sensorhub_i2c_attr_group;
#endif

#ifdef CONFIG_SENSOR_HUB_I2C_ATMEL_DFU
#define ATMEL_BL_I2C_ADDR               0x26
#define ATMEL_BL_STS_INIT_MASK          0xC0
#define ATMEL_BL_STS_WAIT_CMD_1         0xC0
#define ATMEL_BL_STS_APP_CRC_FAIL       0x40
#define ATMEL_BL_STS_WAIT_FRM_1_MASK    0x80
#define ATMEL_BL_STS_FRM_CRC_CHK_1      0x02
#define ATMEL_BL_STS_FRM_CRC_FAIL       0x03
#define ATMEL_BL_STS_FRM_CRC_PASS       0x04
#define ATMEL_BL_MAX_FRAME_SIZE          532

// Duration for bootloader to assert the chg pin
// The max is set for a timeout long enough to calculate the app crc at startup
#define BOOTLOADER_READY_MAX_DELAY_MS       100
#define BOOTLOADER_READY_CHECK_INTERVAL_MS  5

// Number of times the board has to be reset to launch bootloader
#define BOOTLOADER_RESET_REPS 10
// Bootloader execution min time
#define BOOTLOADER_CYCLE_DELAY_MS 50

// Timeout on waiting for a bootloader state change
#define DFU_STATE_CHANGE_TIMEOUT_MS 5000
// Number of times a frame is re-sent on a CRC error
#define DFU_FRAME_SEND_NUM_RETRY 2

static const u8 ATMEL_BL_UNLOCK_CMD[] = {0xDC, 0xAA};
#endif

#ifdef CONFIG_OF
static int sensorhub_i2c_parse_dt(struct device *dev, struct sensorhub_platform_data *pdata) {
    struct device_node *np = dev->of_node;
    enum of_gpio_flags flags;
    int gpio_gyro;
    int gpio_mag;

    /* reset, irq gpio info */
    pdata->gpio_interrupt = of_get_named_gpio_flags(np, "G51,gpio-irq", 0, &flags);
    pdata->gpio_reset = of_get_named_gpio_flags(np, "G51,gpio-rst", 0, &flags);

    pdata->gpio_wakeup = of_get_named_gpio_flags(np, "G51,gpio-sa0", 0, &flags);
    gpio_gyro = of_get_named_gpio_flags(np, "G51,gpio-gyro", 0, &flags);
    gpio_mag = of_get_named_gpio_flags(np, "G51,gpio-mag", 0, &flags);

    printk("gpio-irq = %d\n", pdata->gpio_interrupt);
    printk("gpio-rst = %d\n", pdata->gpio_reset);
    printk("gpio-sa0 = %d\n", pdata->gpio_wakeup);
    printk("gpio-gyro = %d\n", gpio_gyro);
    printk("gpio-mag = %d\n", gpio_mag);
    if (pdata->gpio_interrupt < 0 || pdata->gpio_reset < 0 ||
            pdata->gpio_wakeup < 0 ) {
        return -EINVAL;
    }

    gpio_request(pdata->gpio_interrupt, "sensorhub-irq");
    gpio_request(pdata->gpio_reset, "sensorhub-rst");
    gpio_request(pdata->gpio_wakeup, "sensorhub-sa0");

    gpio_direction_input(pdata->gpio_interrupt);
    gpio_direction_output(pdata->gpio_reset, 1);
    gpio_direction_output(pdata->gpio_wakeup, 0);

    if(gpio_gyro > 0){
        gpio_request(gpio_gyro, "sensorhub-gyro");
        gpio_direction_input(gpio_gyro);
    }
    if(gpio_mag > 0){
        gpio_request(gpio_mag, "sensorhub-mag");
        gpio_direction_input(gpio_mag);
    }

    return 0;
}
#endif

// Prepare a command buf with a cmd addr + reset command
static int sensorhub_i2c_prepare_request_reset(struct sensorhub_i2c_data * i2cdata,
	u8 * buf, size_t len) {

	size_t i = 0;

	CHECK_LEN(4);
	cpu_to_le16_u8p(i2cdata->desc.wCommandRegister, buf, &i);
	cpu_to_le16_u8p(SH_HID_OP_CODE_RESET, buf, &i);

	return i;
}

// Prepare a command buf with a cmd addr + set power command
static int sensorhub_i2c_prepare_request_set_power(struct sensorhub_i2c_data * i2cdata,
	enum sensorhub_i2c_hid_power_state s,
	u8 * buf, size_t len) {

	size_t i = 0;

	if (s != SH_HID_PWR_ON && s != SH_HID_PWR_SLEEP) {
		return -EINVAL;
	}

	CHECK_LEN(4);
	cpu_to_le16_u8p(i2cdata->desc.wCommandRegister, buf, &i);
	cpu_to_le16_u8p(SH_HID_OP_CODE_SET_POWER | s, buf, &i);

	return i;
}

// Prepare a command buf with a cmd addr + get/set report command
static int sensorhub_i2c_prepare_request_getset(struct sensorhub_i2c_data * i2cdata,
	enum sensorhub_i2c_hid_op_code op, enum sensorhub_i2c_hid_report_type type, u8 id,
	u8 * buf, size_t len) {

	size_t i = 0;
	u16 cmd;

	if (op == SH_HID_OP_CODE_GET_REPORT) {
		if (type != SH_HID_REPORT_TYPE_INPUT && type != SH_HID_REPORT_TYPE_FEATURE) {
			return -EINVAL;
		}
	} else if (op == SH_HID_OP_CODE_SET_REPORT) {
		if (type != SH_HID_REPORT_TYPE_OUTPUT && type != SH_HID_REPORT_TYPE_FEATURE) {
			return -EINVAL;
		}
	} else {
		return -EINVAL;
	}

	CHECK_LEN(6);
	cpu_to_le16_u8p(i2cdata->desc.wCommandRegister, buf, &i);
	if (id < SH_HID_OP_CODE_REPORT_ID_SENTINEL) {
		cmd = op | type | id;
		cpu_to_le16_u8p(cmd, buf, &i);
	} else {
		CHECK_LEN(7);
		cmd = op | type | SH_HID_OP_CODE_REPORT_ID_SENTINEL;
		cpu_to_le16_u8p(cmd, buf, &i);
		buf[i++] = id;
	}

	cpu_to_le16_u8p(i2cdata->desc.wDataRegister, buf, &i);
	return i;
}

static int wakeup_delay_us = 25;
module_param(wakeup_delay_us, int, S_IRUGO | S_IWUSR);
static int wakeup_clear_delay_us = 25;
module_param(wakeup_clear_delay_us, int, S_IRUGO | S_IWUSR);

// Perform a write operation containing a command cmd of length cmdlen,
// followed by a read to data of length datalen
// returns 0 on success
static int sensorhub_i2c_command_read_locked(struct sensorhub_i2c_data * i2cdata,
	u8 * cmd, size_t cmdlen,
	u8 * data, size_t datalen) {

	struct sensorhub_drv_data * const ddata = &i2cdata->ddata;
	struct i2c_client * const client = i2cdata->client;
	struct i2c_adapter * const adap = client->adapter;
	struct i2c_msg msg[2];
	int num_msgs = 1;
	int rc;
	bool retry;

	// print_hex_dump(KERN_DEBUG, "CMD: ", DUMP_PREFIX_OFFSET, 16, 1, cmd, cmdlen, true);
	if (ddata->pdata.gpio_wakeup) {
		gpio_set_value(ddata->pdata.gpio_wakeup, 0);
		udelay(wakeup_delay_us);
		retry = true;
	} else {
		retry = false;
	}

do_transfer:
	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = cmdlen;
	msg[0].buf = cmd;

	if (data) {
#ifdef DEBUG
		memset(data, 0xcc, datalen);
#endif
		msg[1].addr = client->addr;
		msg[1].flags = (client->flags & I2C_M_TEN) | I2C_M_RD;
		msg[1].len = datalen;
		msg[1].buf = data;
		num_msgs = 2;
	}

	rc = i2c_transfer(adap, msg, num_msgs);
	if (rc < 0) {
		if (retry) {
			// dummy write and retry
			char dummy[] = {0, 0};
			++ddata->stats.retries;
			udelay(wakeup_clear_delay_us);
			msg[0].addr = client->addr;
			msg[0].flags = client->flags & I2C_M_TEN;
			msg[0].len = 2;
			msg[0].buf = dummy;
			retry = false;
			i2c_transfer(adap, msg, 1);
			goto do_transfer;
		}

		++ddata->stats.errorCount;
		goto exit;
	}

	if (rc != num_msgs) {
		++ddata->stats.errorCount;
		rc = -EIO;
		goto exit;
	}

	ddata->stats.sent++;
	ddata->stats.sentBytes += cmdlen;
	if (data) {
		ddata->stats.received++;
		ddata->stats.receivedBytes += datalen;
	}
	rc = 0;
	// fall-thru
exit:
	if (ddata->pdata.gpio_wakeup) {
		gpio_set_value(ddata->pdata.gpio_wakeup, 1);
	}
	return rc;
}

// Perform a (16-bit address) register read
static int sensorhub_i2c_reg_read_locked(struct sensorhub_i2c_data * i2cdata,
										 u16 reg, u8 * buf, size_t len) {
	reg = cpu_to_le16(reg);
	return sensorhub_i2c_command_read_locked(i2cdata, (u8 *) &reg, sizeof(reg), buf, len);
}

// Get a feature report from the device
static int sensorhub_i2c_get_report(struct sensorhub_drv_data * ddata,
						enum sensorhub_report_type type, u8 id,
						u8 * report, size_t len) {
	int ret;

	u8 command[7];
	u8 buf[GET_REPORT_BUFSIZE];
	size_t responseSize = len + 2;
#ifdef DEBUG
	memset(command, 0xaa, sizeof(command));
	memset(buf, 0xcc, sizeof(buf));
#endif

	if (type != SENSORHUB_REPORT_FEATURE) {
		// don't support get input report yet
		return -EINVAL;
	}

	if (WARN_ON(responseSize > sizeof(buf))) {
		return -ENOBUFS;
	}

	if (test_bit(SENSOR_HUB_SUSPENDED, &ddata->flags)) {
		return -ENXIO;
	}

	ret = sensorhub_i2c_prepare_request_getset(
		__i2c(ddata), SH_HID_OP_CODE_GET_REPORT, SH_HID_REPORT_TYPE_FEATURE, id,
		command, sizeof(command)
	);

	if (WARN_ON(ret < 0)) {
		return ret;
	}

	ret = sensorhub_i2c_command_read_locked(__i2c(ddata), command, ret, buf, responseSize);
	if (ret < 0) {
		dev_err(ddata->dev, "i2c transfer failed for get feature request - %d\n", ret);
		return ret;
	}

	ret = le16_to_cpup((u16 *) &buf[0]);
	if (ret < 0 || ret > 0xffff) {
		dev_dbg(ddata->dev, "invalid length received - %d\n", ret);
		return -EPROTO;
	}

	if (ret - 2 > len) {
		// the report buffer is too small to hold the report
		return -EMSGSIZE;
	}

	// copy only the data to the output buffer
	memcpy(report, buf + 2, ret - 2);
	return ret - 2;
}

// Set a report
// type - feature/output
// id - the report id
// report - repory payload excluding id
// reportLen - length of payload
static ssize_t sensorhub_i2c_set_report(struct sensorhub_drv_data * ddata,
								 enum sensorhub_report_type type, u8 id,
								 const u8 * report, size_t reportLen) {
	int ret;
	u8 cmd[SET_REPORT_BUFSIZE];
	size_t offset = 0;
	enum sensorhub_i2c_hid_report_type _type;

#ifdef DEBUG
	memset(cmd, 0xcc, sizeof(cmd));
#endif

	if (type == SENSORHUB_REPORT_OUTPUT) {
		_type = SH_HID_REPORT_TYPE_OUTPUT;
	} else if (type == SENSORHUB_REPORT_FEATURE) {
		_type = SH_HID_REPORT_TYPE_FEATURE;
	} else {
		return -EINVAL;
	}

	if (test_bit(SENSOR_HUB_SUSPENDED, &ddata->flags)) {
		return -ENXIO;
	}

	ret = sensorhub_i2c_prepare_request_getset(
		__i2c(ddata), SH_HID_OP_CODE_SET_REPORT, _type, id,
		cmd, sizeof(cmd)
	);

	if (WARN_ON(ret < 0)) {
		return ret;
	}

	offset += ret;
	if (reportLen > (sizeof(cmd) - offset - 2)) {
		return -EMSGSIZE;
	}

	cpu_to_le16_u8p((u16) reportLen + 2, cmd, &offset);
	/* copy report in to cmd bufer */
	memcpy(cmd + offset, report, reportLen);
	offset += reportLen;
	ret = sensorhub_i2c_command_read_locked(__i2c(ddata), cmd, offset, NULL, 0);
	if (ret == 0) {
		// success
		return reportLen;
	}

	return ret;
}

// Reset the hub
static int sensorhub_i2c_reset(struct sensorhub_drv_data * ddata, enum sensorhub_reset reset) {
	u8 cmd[4];
	int ret = 0;
	struct sensorhub_i2c_data * i2cdata = __i2c(ddata);

	if (reset == SENSORHUB_RESET_HARD) {
		return sensorhub_i2c_toggle_reset(ddata, 1, RESET_TOGGLE_US, 0);;
	}

	if (reset != SENSORHUB_RESET_SOFT) {
		return -EINVAL;
	}

	ret = sensorhub_i2c_prepare_request_reset(i2cdata, cmd, sizeof(cmd));
	WARN_ON(ret < 0);
	mutex_lock(&i2cdata->hw_lock);
	ret = sensorhub_i2c_command_read_locked(i2cdata, cmd, sizeof(cmd), NULL, 0);
	mutex_unlock(&i2cdata->hw_lock);

	return ret;
}


static int sensorhub_i2c_setpower_locked(struct sensorhub_drv_data * ddata,
										 enum sensorhub_i2c_hid_power_state s) {
	u8 cmd[4];
	int ret = 0;
	int suspend = (s == SH_HID_PWR_SLEEP);
	struct sensorhub_i2c_data * i2cdata = __i2c(ddata);

	ret = sensorhub_i2c_prepare_request_set_power(i2cdata, s, cmd, sizeof(cmd));
	WARN_ON(ret < 0);

	if (suspend) {
		set_bit(SENSOR_HUB_SUSPENDED, &ddata->flags);
		ret = sensorhub_i2c_command_read_locked(i2cdata, cmd, sizeof(cmd), NULL, 0);
	} else {
		ret = sensorhub_i2c_command_read_locked(i2cdata, cmd, sizeof(cmd), NULL, 0);
		clear_bit(SENSOR_HUB_SUSPENDED, &ddata->flags);
		sensorhub_i2c_test_and_read_locked(__i2c(ddata));
	}

	return ret;
}

static int sensorhub_i2c_setpower(struct sensorhub_drv_data * ddata,
								  enum sensorhub_i2c_hid_power_state s) {
	int ret;
	struct sensorhub_i2c_data * i2cdata = __i2c(ddata);

	mutex_lock(&i2cdata->hw_lock);
	ret = sensorhub_i2c_setpower_locked(ddata, s);
	mutex_unlock(&i2cdata->hw_lock);

	return ret;
}

// Get a input report from the device
static int sensorhub_i2c_getinput_locked(struct sensorhub_i2c_data * i2cdata) {

	int rc;
	u16 len = 0xffff;
	const size_t read_size = i2cdata->desc.wMaxInputLength;
	struct sensorhub_drv_data * const ddata = &i2cdata->ddata;
	u8 buf[MAX_INPUT_REPORT_BUFSIZE];
	struct sensorhub_input_event event;

	// record the timestamp at the start of this function because the
	// primary isr can fire before i2c_master_recv() exits, overwriting
	// the timestamp for this report
	u64 host_int_timestamp = i2cdata->irq.timestamp;

	if (test_bit(SENSOR_HUB_SUSPENDED, &ddata->flags)) {
		return -ENXIO;
	}

	if (ddata->state != SENSORHUB_STATE_APP) {
		return -ENODEV;
	}

#ifdef DEBUG
	memset(buf, 0xcc, sizeof(buf));
#endif
	set_debug_gpio(1);
	rc = i2c_master_recv(i2cdata->client, buf, read_size);
	set_debug_gpio(0);

	if (unlikely(rc < 0)) {
		++ddata->stats.errorCount;
		goto exit;
	}

	if (WARN_ON(rc != read_size)) {
		++ddata->stats.errorCount;
		goto exit;
	}

	len = le16_to_cpup((u16 *) &buf[0]);
	if (len > 0 && len <= sizeof(buf)) {
		// a complete message was read in, send event to clients
		event.timestamp = host_int_timestamp;
		event.len = len - 2;
		rc = sensorhub_core_notify_input(ddata, &event, buf + 2, len - 2);
	} else if (len == 0) {
		// power on message recieved
		rc = sensorhub_core_notify_reset(ddata);
	} else {
		dev_err(ddata->dev, "message too long: msg len = %d\n", (int) len);
		rc = -ENOBUFS;
		goto exit;
	}

	wake_up_interruptible(&ddata->wait);

exit:
	return rc;
}

// primary isr, take a timestamp and queue process isr to run
static irqreturn_t sensorhub_i2c_isr_primary(int irq, void * irqdata) {
	struct sensorhub_i2c_data * i2cdata = (struct sensorhub_i2c_data *) irqdata;
	struct sensorhub_drv_data * ddata = &i2cdata->ddata;
	if (ddata->state == SENSORHUB_STATE_APP) {
		i2cdata->irq.timestamp = sensorhub_get_time_ns();
	}

	return IRQ_WAKE_THREAD;
}

// data ready interrupt service routine
static irqreturn_t sensorhub_i2c_isr(int irq, void * irqdata) {
	struct sensorhub_i2c_data * i2cdata = (struct sensorhub_i2c_data *) irqdata;
	struct sensorhub_drv_data * ddata = &i2cdata->ddata;
	++ddata->stats.interrupts;

	if (ddata->state == SENSORHUB_STATE_APP) {
		if (!test_bit(SENSOR_HUB_SUSPENDED, &ddata->flags)) {
			mutex_lock(&i2cdata->hw_lock);
			sensorhub_i2c_getinput_locked(i2cdata);
			mutex_unlock(&i2cdata->hw_lock);
		}
	}

#ifdef CONFIG_SENSOR_HUB_I2C_ATMEL_DFU
	else if (ddata->state == SENSORHUB_STATE_DOWNLOAD) {
		set_debug_gpio(1);
		complete(&i2cdata->bl.compl);
	}
#endif

	return IRQ_HANDLED;
}

// checks the interrupt gpio and services if low
static int sensorhub_i2c_test_and_read_locked(struct sensorhub_i2c_data * i2cdata) {
	struct sensorhub_drv_data * const ddata = &i2cdata->ddata;
	if (gpio_get_value(i2cdata->ddata.pdata.gpio_interrupt) == 0) {
		dev_dbg(ddata->dev, "GPIO is low, getting input\n");
		return sensorhub_i2c_getinput_locked(i2cdata);
	}

	dev_dbg(ddata->dev, "GPIO is high, no data to read\n");
	return 1;
}

// Bring up Sensorhub device
static int sensorhub_i2c_handshake(struct sensorhub_i2c_data * i2cdata) {

	int rc;
	struct sensorhub_drv_data * const ddata = &i2cdata->ddata;
	struct device * dev = ddata->dev;

	BUILD_BUG_ON(sizeof(i2cdata->desc) != SH_HID_DESC_V1_LEN);
	mutex_lock(&i2cdata->hw_lock);
	rc = sensorhub_i2c_reg_read_locked(
			i2cdata, SH_HID_REG_HID_DESCRIPTOR,
			(uint8_t *) &i2cdata->desc, sizeof(i2cdata->desc)
	);

	if (rc < 0) {
		goto exit;
	}

	dev_dbg(dev,
			"I2C Hid Descriptor:\n"
			"    wHIDDescLength            = %04x\n"
			"    bcdVersion                = %04x\n"
			"    wReportDescriptorLength   = %04x\n"
			"    wReportDescriptorRegister = %04x\n"
			"    wInputRegister            = %04x\n"
			"    wMaxInputLength           = %04x\n"
			"    wOutputRegister           = %04x\n"
			"    wMaxOutputLength          = %04x\n"
			"    wCommandRegister          = %04x\n"
			"    wDataRegister             = %04x\n"
			"    wVendorID                 = %04x\n"
			"    wProductID                = %04x\n"
			"    wVersionID                = %04x\n"
			, (unsigned int) i2cdata->desc.wHIDDescLength
			, (unsigned int) i2cdata->desc.bcdVersion
			, (unsigned int) i2cdata->desc.wReportDescriptorLength
			, (unsigned int) i2cdata->desc.wReportDescriptorRegister
			, (unsigned int) i2cdata->desc.wInputRegister
			, (unsigned int) i2cdata->desc.wMaxInputLength
			, (unsigned int) i2cdata->desc.wOutputRegister
			, (unsigned int) i2cdata->desc.wMaxOutputLength
			, (unsigned int) i2cdata->desc.wCommandRegister
			, (unsigned int) i2cdata->desc.wDataRegister
			, (unsigned int) i2cdata->desc.wVendorID
			, (unsigned int) i2cdata->desc.wProductID
			, (unsigned int) i2cdata->desc.wVersionID
	);

	i2cdata->desc.wHIDDescLength            = le16_to_cpu(i2cdata->desc.wHIDDescLength);
	i2cdata->desc.bcdVersion                = le16_to_cpu(i2cdata->desc.bcdVersion);
	i2cdata->desc.wReportDescriptorLength   = le16_to_cpu(i2cdata->desc.wReportDescriptorLength);
	i2cdata->desc.wReportDescriptorRegister = le16_to_cpu(i2cdata->desc.wReportDescriptorRegister);
	i2cdata->desc.wInputRegister            = le16_to_cpu(i2cdata->desc.wInputRegister);
	i2cdata->desc.wMaxInputLength           = le16_to_cpu(i2cdata->desc.wMaxInputLength);
	i2cdata->desc.wOutputRegister           = le16_to_cpu(i2cdata->desc.wOutputRegister);
	i2cdata->desc.wMaxOutputLength          = le16_to_cpu(i2cdata->desc.wMaxOutputLength);
	i2cdata->desc.wCommandRegister          = le16_to_cpu(i2cdata->desc.wCommandRegister);
	i2cdata->desc.wDataRegister             = le16_to_cpu(i2cdata->desc.wDataRegister);
	i2cdata->desc.wVendorID                 = le16_to_cpu(i2cdata->desc.wVendorID);
	i2cdata->desc.wProductID                = le16_to_cpu(i2cdata->desc.wProductID);
	i2cdata->desc.wVersionID                = le16_to_cpu(i2cdata->desc.wVersionID);

	if (i2cdata->desc.wHIDDescLength != SH_HID_DESC_V1_LEN) {
		dev_err(dev, "Invalid descriptor length = %d, expecting %d\n",
			i2cdata->desc.wHIDDescLength, SH_HID_DESC_V1_LEN);
		rc = -ENODEV;
		goto exit;
	}

	if (i2cdata->desc.bcdVersion != SH_HID_DESC_V1_BCD) {
		dev_err(dev, "Incompatible bcdVersion = %4x, expecting %4x\n",
			i2cdata->desc.bcdVersion, SH_HID_DESC_V1_BCD);
		rc = -ENODEV;
		goto exit;
	}

	if (i2cdata->desc.wMaxInputLength > MAX_INPUT_REPORT_BUFSIZE) {
		dev_warn(dev, "Input message size exceeds max length = %4x.\n",
			i2cdata->desc.wMaxInputLength);
		rc = -ENODEV;
		goto exit;
	}

	if (!i2cdata->desc.wReportDescriptorLength ||
		 i2cdata->desc.wReportDescriptorLength > SENSORHUB_MAX_REPORT_DESCRIPTOR_SIZE) {
		dev_err(dev, "Unexpected reportDescriptorLength=%d\n",
			i2cdata->desc.wReportDescriptorLength);
		rc = -ENODEV;
		goto exit;
	}

	if (ddata->report_descriptor.buf) {
		kfree(ddata->report_descriptor.buf);
		ddata->report_descriptor.len = 0;
	}

	ddata->report_descriptor.buf = kzalloc(i2cdata->desc.wReportDescriptorLength, GFP_KERNEL);
	ddata->report_descriptor.len = i2cdata->desc.wReportDescriptorLength;

	if (!ddata->report_descriptor.buf) {
		dev_err(dev, "Failed allocating report descriptor\n");
		rc = -ENOMEM;
		goto exit;
	}

	rc = sensorhub_i2c_reg_read_locked(
		i2cdata, i2cdata->desc.wReportDescriptorRegister,
		ddata->report_descriptor.buf, i2cdata->desc.wReportDescriptorLength
	);
	if (rc) {
		dev_err(dev, "Failed reading report descriptor, rc=%d\n", rc);
		kfree(ddata->report_descriptor.buf);
		ddata->report_descriptor.buf = 0;
		ddata->report_descriptor.len = 0;
		goto exit;
	}

	ddata->vendorID  = i2cdata->desc.wVendorID;
	ddata->productID = i2cdata->desc.wProductID;
	ddata->versionID = i2cdata->desc.wVersionID;
	ddata->state = SENSORHUB_STATE_APP;

	sensorhub_i2c_test_and_read_locked(i2cdata);
	rc = 0;
	goto exit;

exit:
	mutex_unlock(&i2cdata->hw_lock);
	return rc;
}

// try to establish comms with the device every APP_STARTUP_CHECK_INTERVAL_MS
// and give up after APP_STARTUP_MAX_DELAY_MS
static int sensorhub_i2c_poll_handshake(struct sensorhub_i2c_data * i2cdata) {
	struct timespec end, now;
	end = CURRENT_TIME;
	now = end;

	end = CURRENT_TIME;
	timespec_add_ns(&end, APP_STARTUP_MAX_DELAY_MS * 1000ull * 1000ull);
	while (timespec_compare(&now, &end) < 0) {
		if (sensorhub_i2c_handshake(i2cdata) == 0) {
			return 0;
		}
		now = CURRENT_TIME;
		msleep(APP_STARTUP_CHECK_INTERVAL_MS);
	}
	return -ETIMEDOUT;
}

static int sensorhub_i2c_toggle_reset(struct sensorhub_drv_data * ddata,
									unsigned int reps,
									unsigned int toggle_us,
									unsigned int rep_ms) {
	if (ddata->pdata.gpio_reset == 0) {
		return -EOPNOTSUPP;
	}

	if (reps == 1) {
		gpio_set_value(ddata->pdata.gpio_reset, 0);
		udelay(toggle_us);
		gpio_set_value(ddata->pdata.gpio_reset, 1);
	} else {
		while (reps--) {
			gpio_set_value(ddata->pdata.gpio_reset, 0);
			udelay(toggle_us);
			gpio_set_value(ddata->pdata.gpio_reset, 1);
			if (reps) { // exit immediately after last
				msleep(rep_ms);
			}
		}
	}

	return 0;
}

#ifdef CONFIG_SENSOR_HUB_I2C_ATMEL_DFU

static int sensorhub_i2c_dfu_bl_read(struct sensorhub_i2c_data * i2cdata, u8 * buf, size_t len) {
	struct i2c_msg msg[1];
	int rc;
	//struct sensorhub_drv_data * const ddata = &i2cdata->ddata;
	struct i2c_client * const client = i2cdata->client;
	struct i2c_adapter * const adap = client->adapter;

	msg[0].addr = ATMEL_BL_I2C_ADDR;
	msg[0].flags = (i2cdata->client->flags & I2C_M_TEN) | I2C_M_RD;
	msg[0].len = len;
	msg[0].buf = buf;

	rc = i2c_transfer(adap, msg, 1);
	return rc == 1 ? 0 : rc;
}

#ifdef CONFIG_ARCH_MSM8939
#define DFU_BUF_LEN 59

static int sensorhub_i2c_dfu_bl_write_nshift(struct sensorhub_i2c_data * i2cdata, const u8 * buf, size_t len) {
    struct i2c_msg msg[1];
    int rc;
    //struct sensorhub_drv_data * const ddata = &i2cdata->ddata;
    struct i2c_client * const client = i2cdata->client;
    struct i2c_adapter * const adap = client->adapter;

    msg[0].addr = ATMEL_BL_I2C_ADDR;
    msg[0].flags = (i2cdata->client->flags & I2C_M_TEN);
    msg[0].len = len;
    msg[0].buf = (u8 *) buf; // discard const

    rc = i2c_transfer(adap, msg, 1);
    return rc == 1 ? 0 : rc;
}

static int sensorhub_i2c_dfu_write_multi_block(struct sensorhub_i2c_data * i2cdata, const u8 * buf, size_t len)
{
	int times = len / DFU_BUF_LEN;
	int s = len - (times*DFU_BUF_LEN);
	int i = 0;
	int ret = 0;

	for(i=0; i<times; i++){
		ret += sensorhub_i2c_dfu_bl_write_nshift(i2cdata, buf+(i*DFU_BUF_LEN), DFU_BUF_LEN);
		usleep(50);
	}
	if(s>0){
		ret += sensorhub_i2c_dfu_bl_write_nshift(i2cdata, buf+(times*DFU_BUF_LEN), s);
	}
	//pr_info("%s : i =%d, ret=%d, len=%d.\n", __func__, i, ret, len);

	return ret;
}
#endif

static int sensorhub_i2c_dfu_bl_write(struct sensorhub_i2c_data * i2cdata, const u8 * buf, size_t len) {
    struct i2c_msg msg[1];
    int rc;
    //struct sensorhub_drv_data * const ddata = &i2cdata->ddata;
    struct i2c_client * const client = i2cdata->client;
    struct i2c_adapter * const adap = client->adapter;

#ifdef CONFIG_ARCH_MSM8939
    if(len > DFU_BUF_LEN){
        return sensorhub_i2c_dfu_write_multi_block(i2cdata, buf, len);
    }
#endif
    msg[0].addr = ATMEL_BL_I2C_ADDR;
    msg[0].flags = (i2cdata->client->flags & I2C_M_TEN);
    msg[0].len = len;
    msg[0].buf = (u8 *) buf; // discard const

    rc = i2c_transfer(adap, msg, 1);
    return rc == 1 ? 0 : rc;
}

// poll the chg interrupt until it asserts (low) or a timeout occurs
// return 0 if chg is asserted or -ETIMEDOUT
static int sensorhub_i2c_bl_poll_chg(struct sensorhub_drv_data * ddata) {
    struct timespec end, now;;
    end = CURRENT_TIME;
    now = end;
    timespec_add_ns(&end, BOOTLOADER_READY_MAX_DELAY_MS * 1000 * 1000);
    while (timespec_compare(&now, &end) < 0) {
        if (gpio_get_value(ddata->pdata.gpio_interrupt) == 0) {
            return 0;
        }
        now = CURRENT_TIME;
        msleep(BOOTLOADER_READY_CHECK_INTERVAL_MS);
    }
    return -ETIMEDOUT;
}

static int sensorhub_i2c_dfu_reset_to_bl(struct sensorhub_i2c_data * i2cdata) {
    struct sensorhub_drv_data * ddata = &i2cdata->ddata;
    int rc;
    char buf[3];

    sensorhub_i2c_toggle_reset(ddata, BOOTLOADER_RESET_REPS, RESET_TOGGLE_US, BOOTLOADER_CYCLE_DELAY_MS);
    if (sensorhub_i2c_bl_poll_chg(ddata)) {
        dev_dbg(ddata->dev, "reset2bl: bootloader did not respond (timeout waiting for CHG)\n");
        return -ENODEV;
    }

    rc = sensorhub_i2c_dfu_bl_read(i2cdata, buf, sizeof(buf));
    if (rc != 0) {
        dev_err(ddata->dev, "reset2bl: bootloader read failed\n");
        return -ENODEV;
    }

    dev_dbg(ddata->dev, "reset2bl: bootloader status=%02x id=%02x ver=%02x\n",
            buf[0], buf[1], buf[2]);
    if ((buf[0] & ATMEL_BL_STS_INIT_MASK) == ATMEL_BL_STS_APP_CRC_FAIL) {
        dev_warn(ddata->dev, "reset2bl: bootloader reports app CRC fail\n");
        ddata->state = SENSORHUB_STATE_BOOTLOADER_NOIMAGE;
        return 0;
    }

    if ((buf[0] & ATMEL_BL_STS_INIT_MASK) == ATMEL_BL_STS_WAIT_CMD_1) {
        dev_info(ddata->dev, "reset2bl: bootloader ready\n");
        ddata->state = SENSORHUB_STATE_BOOTLOADER;
        return 0;
    }
    dev_err(ddata->dev, "reset2bl: bootloader response with unexpected status=%02x\n", buf[0]);
    return -ENODEV;
}

static void sensorhub_i2c_dfu_abort(struct sensorhub_drv_data * ddata) {
    sensorhub_i2c_init_app(__i2c(ddata));
}

struct sensorhub_atmel_dfu_context {
    struct sensorhub_i2c_data * i2cdata;

    struct timespec startTime;

    size_t len;
    size_t sentBytes;
    int sentFramesCount;
};

// return negative number on error or the status as a u8
static int sensorhub_i2c_dfu_wait_status(struct sensorhub_atmel_dfu_context * ctx,
        unsigned long timeout) {

    struct sensorhub_i2c_data * i2cdata = ctx->i2cdata;
    struct sensorhub_drv_data * ddata = &i2cdata->ddata;
    u8 status;

    long rc = wait_for_completion_interruptible_timeout(&i2cdata->bl.compl, timeout);
    set_debug_gpio(0);
    if (rc < 0) {
        dev_err(ddata->dev, "interrupted while waiting for status\n");
        return -EINTR;
    } else if (rc == 0) {
        dev_err(ddata->dev, "time out waiting for status\n");
        return -ETIMEDOUT;
    }

    rc = sensorhub_i2c_dfu_bl_read(i2cdata, &status, 1);
    if (rc < 0) {
        dev_err(ddata->dev, "error reading status: %ld\n", rc);
        return rc;
    }

    return status;
}

static int sensorhub_i2c_dfu_fw_validate(struct sensorhub_drv_data * ddata,
        const u8 * fw, size_t len) {

    struct sensorhub_dfu_image_header const * header;
    struct sensorhub_dfu_image_footer const * footer;
    u32 crc_cmp;
    const u8 * p;

    if (IS_ERR_OR_NULL(fw) || !len ||
            len < (sizeof(struct sensorhub_dfu_image_header) +
                sizeof(struct sensorhub_dfu_image_footer))) {
        return -EINVAL;
    }

    header = (struct sensorhub_dfu_image_header const *) fw;
    if (htonl(header->id)      != HCREST_DFU_IMAGE_FILE_ID ||
            htonl(header->ffVer)   != HCREST_DFU_IMAGE_FF_VER  ||
            htonl(header->sz)      != len ||
            htonl(header->payload) >= len) {
        return -EINVAL;
    }

#ifdef DEBUG
    dev_dbg(ddata->dev, "DFU: Image details -\n%-*s",
            htonl(header->payload) - sizeof(*header),
            header->metadata);
#endif

    // calculate footer
    p = fw + len - sizeof(struct sensorhub_dfu_image_footer);
    footer = (struct sensorhub_dfu_image_footer const *) p;

    crc_cmp = ~crc32(0xffffffff , fw, len - 4);
    if (htonl(footer->crc32) != crc_cmp) {
        dev_err(ddata->dev, "CRC check failed\n");
        return -EINVAL;
    }

    // check that payload contains valid atmel image format frames
    p = fw + htonl(header->payload);
    while (p < (u8 *) footer) {
        size_t frame_len = p[1] | (p[0] << 8);
        size_t xfer_len = frame_len + 2; // + Frame CRC
        if (frame_len > ATMEL_BL_MAX_FRAME_SIZE) {
            dev_err(ddata->dev, "frame exceeds max size: %zu/%zu\n",
                    frame_len, ATMEL_BL_MAX_FRAME_SIZE);
            return -EINVAL;
        }

        if ((p + xfer_len) > (u8 *) footer) {
            dev_err(ddata->dev, "frame exceed fw image bounds\n");
            return -EINVAL;
        }

        p += xfer_len;
    }

    // the last frame should have aligned p with the footer
    return (p != (u8 *) footer) ? -EINVAL : 0;
}

static int sensorhub_i2c_dfu_perform(struct sensorhub_drv_data * ddata, const u8 * fw, size_t len) {

    int rc;
    struct sensorhub_i2c_data * i2cdata = __i2c(ddata);
    struct sensorhub_atmel_dfu_context * ctx;
    struct sensorhub_dfu_image_header const * header = (struct sensorhub_dfu_image_header const *) fw;

    const u8 * const fw_begin = fw + htonl(header->payload);
    const u8 * const fw_end = fw + len - 4;
    const u8 * p = fw_begin;
    size_t payload_len = fw_end - fw_begin;

    int frame_retry = DFU_FRAME_SEND_NUM_RETRY;

    // print progress updates up to 8 times or after 4k chunks
    const size_t progress_incr = (len >> 3) > 4096 ? (len >> 3) : 4096;
    size_t next_print = progress_incr;

    BUG_ON(IS_ERR_OR_NULL(ddata));
    BUG_ON(IS_ERR_OR_NULL(ddata->dev));

    if (WARN_ON(test_and_set_bit(SENSOR_HUB_DFU_IN_FLIGHT, &ddata->flags))) {
        return -EINVAL;
    }

    // enter the download state
    if (ddata->state != SENSORHUB_STATE_BOOTLOADER ||
            ddata->state != SENSORHUB_STATE_BOOTLOADER_NOIMAGE) {
        ddata->state = SENSORHUB_STATE_DOWNLOAD_ENTER;
        sensorhub_notify_state(ddata);
        if ((rc = sensorhub_i2c_dfu_reset_to_bl(i2cdata))) {
            clear_bit(SENSOR_HUB_DFU_IN_FLIGHT, &ddata->flags);
            return rc;
        }
    }
    ddata->state = SENSORHUB_STATE_DOWNLOAD;
    sensorhub_notify_state(ddata);

    ctx = kzalloc(sizeof(struct sensorhub_atmel_dfu_context), GFP_KERNEL);
    if (IS_ERR_OR_NULL(ctx)) {
        rc = -ENOMEM;
        goto exit;
    }

    // initialize our context
    init_completion(&i2cdata->bl.compl);
    ctx->len = len;
    ctx->i2cdata = i2cdata;
    ctx->startTime = CURRENT_TIME;

    // send the unlock command
    rc = sensorhub_i2c_dfu_bl_write(i2cdata,
            ATMEL_BL_UNLOCK_CMD, ARRAY_SIZE(ATMEL_BL_UNLOCK_CMD));
    if (rc) {
        goto exit_free_ctx;
    }

    while (p < (fw_end - 1)) {
        // extra two bytes for CRC
        size_t xfer_len = (p[1] | (p[0] << 8)) + 2;
        long t;

        rc = sensorhub_i2c_dfu_wait_status(ctx, DFU_STATE_CHANGE_TIMEOUT_MS);
        if (rc < 0) {
            goto exit_free_ctx;
        }

        if (!(rc & ATMEL_BL_STS_WAIT_FRM_1_MASK)) {
            dev_err(ddata->dev, "unexpected status=%02x Expect WAIT_FRAME_1\n", rc);
            rc = EINVAL;
            goto exit_free_ctx;
        }

        // CHG is toggled after the previous read
        t = wait_for_completion_interruptible_timeout(&i2cdata->bl.compl, DFU_STATE_CHANGE_TIMEOUT_MS);
        set_debug_gpio(0);
        if (t < 0) {
            dev_err(ddata->dev, "interrupted while waiting for bogus status\n");
            rc = -EINTR;
            goto exit_free_ctx;
        }
        // try the read after the timeout anyway

        rc = sensorhub_i2c_dfu_bl_write(i2cdata, p, xfer_len);
        if (rc) {
            dev_err(ddata->dev, "error sending frame: %d\n", rc);
            goto exit_free_ctx;
        }

        rc = sensorhub_i2c_dfu_wait_status(ctx, DFU_STATE_CHANGE_TIMEOUT_MS);
        if (rc < 0) {
            goto exit_free_ctx;
        }
        if (rc != ATMEL_BL_STS_FRM_CRC_CHK_1) {
            dev_err(ddata->dev, "unexpected status=%02x expect CRC_CHK_1\n", rc);
            rc = EINVAL;
            goto exit_free_ctx;
        }

        rc = sensorhub_i2c_dfu_wait_status(ctx, DFU_STATE_CHANGE_TIMEOUT_MS);
        if (rc == ATMEL_BL_STS_FRM_CRC_PASS) {
            p += xfer_len;
            ++ctx->sentFramesCount;
            ctx->sentBytes += xfer_len;
            frame_retry = DFU_FRAME_SEND_NUM_RETRY;
            if ((p - fw) >= next_print) {
                struct timespec elapsed = timespec_sub(CURRENT_TIME, ctx->startTime);
                dev_dbg(ddata->dev, "DFU: sent %5zu/%zu bytes %3d frames %d.%03d s elapsed\n",
                        ctx->sentBytes, payload_len, ctx->sentFramesCount,
                        (int) elapsed.tv_sec, (int)(elapsed.tv_nsec / 1000 / 1000));
                next_print += progress_incr;
            }
        } else if (rc == ATMEL_BL_STS_FRM_CRC_FAIL) {
            if (frame_retry--) {
                dev_warn(ddata->dev, "DFU: status=CRC failed; frame=%d offset=%zu; retrying.\n",
                        ctx->sentFramesCount, ctx->sentBytes);
            } else {
                dev_err(ddata->dev, "DFU: failed CRC check\n");
                rc = -EINVAL;
                goto exit_free_ctx;
            }
        } else {
            dev_err(ddata->dev, "DFU: unexpected status from device: %02x\n", rc);
            rc = -EINVAL;
            goto exit_free_ctx;
        }
    }

    {
        struct timespec elapsed = timespec_sub(CURRENT_TIME, ctx->startTime);
        dev_info(ddata->dev, "DFU: sent %5zu/%zu bytes %3d frames %d.%03d s elapsed\n",
                ctx->sentBytes, payload_len, ctx->sentFramesCount,
                (int) elapsed.tv_sec, (int)(elapsed.tv_nsec / 1000 / 1000));
        dev_info(ddata->dev, "DFU: download finished\n");
    }

    if ((rc = sensorhub_i2c_poll_handshake(i2cdata))) {
        char buf[3];
        dev_warn(ddata->dev, "DFU: application did not start after successful download.\n");

        // try bootloader
        rc = sensorhub_i2c_dfu_bl_read(i2cdata, buf, sizeof(buf));
        if (rc == 0) {
            if ((buf[0] & ATMEL_BL_STS_INIT_MASK) == ATMEL_BL_STS_APP_CRC_FAIL) {
                ddata->state = SENSORHUB_STATE_BOOTLOADER_NOIMAGE;
                dev_err(ddata->dev, "DFU: download succeeded, but bootloader reports app CRC fail\n");
            } else {
                ddata->state = SENSORHUB_STATE_DOWNLOAD_FAILED;
                dev_dbg(ddata->dev, "DFU: got bootloader response after DFU. status=%02x id=%02x ver=%02x\n",
                        buf[0], buf[1], buf[2]);
            }

            rc = -EREMOTEIO;
        } else {
            dev_warn(ddata->dev, "DFU: resetting and trying again\n");
            sensorhub_i2c_reset(ddata, SENSORHUB_RESET_HARD);
            if ((rc = sensorhub_i2c_poll_handshake(i2cdata))) {
                ddata->state = SENSORHUB_STATE_DOWNLOAD_FAILED;
                if ((rc = sensorhub_i2c_dfu_reset_to_bl(i2cdata))) {
                    dev_err(ddata->dev, "DFU: download succeeded, but no response from app or bootloader now\n");
                    rc = -ENODEV;
                } else {
                    dev_err(ddata->dev, "DFU: download succeeded, but no response from app\n");
                    rc = -EREMOTEIO;
                }
            }
        }


    }
exit_free_ctx:
    clear_bit(SENSOR_HUB_DFU_IN_FLIGHT, &ddata->flags);
    kfree(ctx);
exit:
    return rc;
}

static int sensorhub_i2c_bl_detect_and_exit(struct sensorhub_i2c_data * i2cdata) {
    char buf[3];
    char reset_cmd[] = {0x00, 0x00};
    int rc;

    struct sensorhub_drv_data * ddata = &i2cdata->ddata;

    if (sensorhub_i2c_dfu_bl_read(i2cdata, buf, sizeof(buf)) != 0) {
        // bootloader did not respond
        return 1;
    }
    dev_dbg(ddata->dev, "init: bootloader status=%02x id=%02x ver=%02x\n",
            buf[0], buf[1], buf[2]);

    // got a response from the bootloader
    if ((buf[0] & ATMEL_BL_STS_INIT_MASK) == ATMEL_BL_STS_APP_CRC_FAIL) {
        dev_warn(ddata->dev, "init: app crc failure\n");
        // no way to exit from bl when crc is failed
        return 1;
    }

    // waiting for unlock?
    if ((buf[0] & ATMEL_BL_STS_INIT_MASK) != ATMEL_BL_STS_WAIT_CMD_1) {
        dev_warn(ddata->dev, "init: unexpected bootloader status: %02x\n", buf[0]);
        return -ENODEV;
    }

    dev_dbg(ddata->dev, "init: bootloader state=waiting; try bootloader exit\n");
    // send the unlock command
    rc = sensorhub_i2c_dfu_bl_write(i2cdata,
            ATMEL_BL_UNLOCK_CMD, ARRAY_SIZE(ATMEL_BL_UNLOCK_CMD));
    if (rc) {
        dev_dbg(ddata->dev, "init: bootloader unlock command write failed (%d)\n", rc);
        return rc;
    }

    if (sensorhub_i2c_bl_poll_chg(ddata) != 0) {
        set_debug_gpio(1);
        dev_err(ddata->dev, "init: bootloader timedout\n");
        return -ENODEV;
    }

    if (sensorhub_i2c_dfu_bl_read(i2cdata, buf, 1) != 0) {
        dev_dbg(ddata->dev, "init: no response from bootloader after unlock\n");
        return -ENODEV;

    }

    if (!(buf[0] & ATMEL_BL_STS_WAIT_FRM_1_MASK)) {
        dev_warn(ddata->dev, "init: unexpected bootloader status: %02x\n", buf[0]);
        return -ENODEV;
    }

    // wait for CHG to assert (again)
    if (sensorhub_i2c_bl_poll_chg(ddata) != 0) {
        dev_err(ddata->dev, "init: bootloader timedout\n");
        return -ENODEV;
    }

    // the bootloader should be unlocked now.
    // a zero length frame _should_ cause it to reset
    dev_dbg(ddata->dev, "init: bootloader unlocked. sending reset command\n");
    rc = sensorhub_i2c_dfu_bl_write(i2cdata, reset_cmd, sizeof(reset_cmd));
    if (rc) {
        dev_dbg(ddata->dev, "init: bootloader unlock reset write failed (%d)\n", rc);
        return rc;
    }

    return rc;
}
#else // CONFIG_SENSOR_HUB_I2C_ATMEL_DFU

static int sensorhub_i2c_bl_detect_and_exit(struct sensorhub_i2c_data * i2cdata) {
    return 1;

}
#endif // CONFIG_SENSOR_HUB_I2C_ATMEL_DFU

// Contact the sensorhub application and determine if it's running
// Peform any first time intiialization
// return 0 if application is started (and running) or -ENODEV
static int sensorhub_i2c_init_app(struct sensorhub_i2c_data * i2cdata) {
    struct sensorhub_drv_data * ddata = &i2cdata->ddata;
    int rc;

    if (sensorhub_i2c_handshake(i2cdata) == 0) {
        // all is good, no need for anything fancy
        return 0;
    }

    dev_info(ddata->dev, "init: app did not respond, retrying after reset\n");
    sensorhub_i2c_reset(ddata, SENSORHUB_RESET_HARD);
    if (sensorhub_i2c_poll_handshake(i2cdata) == 0) {
        return 0;
    }

    // no response from the app, maybe it's stuck in the bootloader?
    rc = sensorhub_i2c_bl_detect_and_exit(i2cdata);
    if (rc < 0) {
        // something went wrong
        return rc;
    }

    if (rc > 0) {
        // an unrecovarable error occred; we can't boot in to application
        return -ENODEV;
    }

    // we exited from a stuck bootloader
    // allow the app to startup one last time
    return sensorhub_i2c_poll_handshake(i2cdata);
}

#ifdef DEBUG
inline void _gpio_export(int gpio) {
    if (gpio > 0) {
        gpio_export(gpio, 0);
    }
}

inline void _gpio_unexport(int gpio) {
    if (gpio > 0) {
        gpio_unexport(gpio);
    }
}
#endif


// Allocate Driver instance
static int sensorhub_i2c_probe(struct i2c_client * client,
        const struct i2c_device_id * id) {
#ifdef CONFIG_OF
    struct sensorhub_platform_data * pdata = NULL;
#endif
    struct sensorhub_i2c_data * i2cdata;
    struct sensorhub_drv_data * ddata;
    int err = 0;

    dev_info(&client->dev, "i2c-client %s on %s #%d\n",
            client->name, client->adapter->name, client->adapter->nr);

#ifdef CONFIG_OF
    if (client->dev.of_node) {
        pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
        if (!pdata) {
            dev_err(&client->dev, "Failed to allocate memory\n");
            return -ENOMEM;
        }

        err = sensorhub_i2c_parse_dt(&client->dev, pdata);
        if (err) {
            dev_err(&client->dev, "Failed to parse dt\n");
            kfree(pdata);
            return -ENODEV;
        }
        pdata->irq = client->irq;
        client->dev.platform_data = (void *)pdata;
    }
#endif

    if (client->dev.platform_data == NULL) {
        dev_err(&client->dev, "No platform_data\n");
        err = -EINVAL;
        goto exit;
    }

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        err = -ENODEV;
        goto exit;
    }

    i2cdata = kzalloc(sizeof(struct sensorhub_i2c_data), GFP_KERNEL);
    if (!i2cdata) {
        dev_err(&client->dev, "could not allocate private data memory\n");
        err = -ENOMEM;
        goto exit;
    }

    ddata = &i2cdata->ddata;
    i2cdata->client = client;
    i2c_set_clientdata(client, ddata);
    mutex_init(&i2cdata->hw_lock);

    ddata->pdata = *((struct sensorhub_platform_data *) client->dev.platform_data);
    if (!ddata->pdata.gpio_interrupt || !ddata->pdata.irq) {
        dev_err(&client->dev, "irq/gpio not set in platform config\n");
        err = -EINVAL;
        goto exit_free_data;
    }
#ifdef CONFIG_SENSOR_HUB_I2C_ATMEL_DFU
    if (!ddata->pdata.gpio_reset) {
        dev_err(&client->dev, "reset gpio not set in platform config\n");
        err = -EINVAL;
        goto exit_free_data;
    }
#endif

    set_debug_gpio(0);

    if (ddata->pdata.setup) {
        err = ddata->pdata.setup(&client->dev, 1);
        if (err) {
            dev_err(&client->dev, "platform setup failed\n");
            goto exit_free_data;
        }
    }

    ddata->transport = &i2c_sensorhub_transport;

    /* register with core */
    err = sensorhub_core_register(ddata);
    if (err) {
        goto exit_power_down;
    }

    dev_info(ddata->dev, "driver version: " SENSORHUB_VERSION);
    dev_dbg(ddata->dev,
            "I2C Platform Configuration:\n"
            "    irq            = %d\n"
            "    gpio_interrupt = %d\n"
            "    gpio_reset     = %d\n"
            "    gpio_debug     = %d\n"
            "    gpio_wakeup    = %d\n"
            "    setup          = %-p\n"
            , ddata->pdata.irq
            , ddata->pdata.gpio_interrupt
            , ddata->pdata.gpio_reset
            , ddata->pdata.gpio_debug
            , ddata->pdata.gpio_wakeup
            , ddata->pdata.setup
           );

#ifdef DEBUG
    _gpio_export(ddata->pdata.gpio_interrupt);
    _gpio_export(ddata->pdata.gpio_reset);
    _gpio_export(ddata->pdata.gpio_debug);
    _gpio_export(ddata->pdata.gpio_wakeup);
#endif

    err = request_threaded_irq(
            ddata->pdata.irq,
            sensorhub_i2c_isr_primary,
            sensorhub_i2c_isr,
	#ifdef CONFIG_ARCH_MSM8939
            IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
	#else
            IRQF_TRIGGER_FALLING,
	#endif
            "sensorhub-hid",
            i2cdata
            );
    if (err < 0) {
        dev_dbg(ddata->dev, "request_irq failed\n");
        goto exit_unregister;
    }

    err = sensorhub_i2c_init_app(i2cdata);
#ifdef CONFIG_SENSOR_HUB_I2C_ATMEL_DFU
    if (err) {
        err = sensorhub_i2c_dfu_reset_to_bl(i2cdata);
        if (!err) {
            if (ddata->state == SENSORHUB_STATE_BOOTLOADER) {
                msleep(1000);
                if (sensorhub_i2c_bl_detect_and_exit(i2cdata) == 0) {
                    // we exited the bootloader succesfully
                    err = sensorhub_i2c_handshake(i2cdata);
                    if (err) {
                        ddata->state = SENSORHUB_STATE_BOOTLOADER_HUNG_APP;
                    }
                };
            }
        } else {
            // hosed
            goto exit_free_irq;
        }
    }
#else
    if (err) {
        dev_err(ddata->dev, "error initializing i2c device: %d\n", err);
        goto exit_free_irq;
    }
#endif

#ifdef SENSORHUB_SYSFS_DEBUG
    err = sysfs_create_group(&ddata->dev->kobj, &sensorhub_i2c_attr_group);
    if (err) {
        dev_err(ddata->dev, "error initializing i2c sysfs attrs: %d\n", err);
        goto exit_free_irq;
    }
#endif

    sensorhub_notify_state(ddata);
    dev_info(ddata->dev, "successfully initialized %s (misc=%d)\n",
            ddata->name, ddata->misc.minor);
    return err;

exit_free_irq:
    free_irq(ddata->pdata.irq, i2cdata);
exit_unregister:
    sensorhub_core_unregister(ddata);
exit_power_down:
    if (ddata->pdata.setup) {
        ddata->pdata.setup(&client->dev, 0);
    }
exit_free_data:
    kfree(i2cdata);
exit:
#ifdef CONFIG_OF
    if (client->dev.of_node) {
        kfree(pdata);
    }
#endif
    return err;
}

// Remove a driver instance
static int sensorhub_i2c_remove(struct i2c_client * client) {
    struct sensorhub_drv_data * const ddata = i2c_get_clientdata(client);
    struct sensorhub_i2c_data * const i2cdata = __i2c(ddata);

    dev_info(ddata->dev, "remove\n");
    while (test_bit(SENSOR_HUB_DFU_IN_FLIGHT, &ddata->flags)) {
        dev_dbg(&client->dev, "waiting for DFU to complete - flags=%lu\n",
                (unsigned long) ddata->flags);
        msleep(500);
    }


#ifdef DEBUG
    _gpio_unexport(ddata->pdata.gpio_wakeup);
    _gpio_unexport(ddata->pdata.gpio_debug);
    _gpio_unexport(ddata->pdata.gpio_reset);
    _gpio_unexport(ddata->pdata.gpio_interrupt);
#endif

#ifdef SENSORHUB_SYSFS_DEBUG
    sysfs_remove_group(&ddata->dev->kobj,  &sensorhub_i2c_attr_group);
#endif
    sensorhub_core_unregister(ddata);
    if (ddata->report_descriptor.buf) {
        kfree(ddata->report_descriptor.buf);
        ddata->report_descriptor.buf = NULL;
    }
    free_irq(ddata->pdata.irq, i2cdata);
    if (ddata->pdata.setup) {
        ddata->pdata.setup(&client->dev, 0);
    }
    kfree(i2cdata);

    return 0;
}

#ifdef CONFIG_PM

static int sensorhub_i2c_resume(struct device *dev) {
    struct sensorhub_drv_data * const ddata = dev_get_drvdata(dev);

    if (sensorhub_i2c_setpower(ddata, SH_HID_PWR_ON)) {
        ++ddata->stats.errorCount;
    }
    return 0;
}

static int sensorhub_i2c_suspend(struct device *dev) {
    struct sensorhub_drv_data * const ddata = dev_get_drvdata(dev);

    ++ddata->stats.suspendCount;
    if (sensorhub_i2c_setpower(ddata, SH_HID_PWR_SLEEP)) {
        ++ddata->stats.errorCount;
    }
    return 0;
}

    static const struct dev_pm_ops sensorhub_drv_pmops = {
        SET_SYSTEM_SLEEP_PM_OPS(sensorhub_i2c_suspend, sensorhub_i2c_resume)
    };

#define SH_PM_OPS (&sensorhub_drv_pmops)

#else

#define SH_PM_OPS NULL

#endif /* CONFIG_PM */

#ifdef SENSORHUB_SYSFS_DEBUG
inline struct sensorhub_drv_data * __misc(struct device *dev) {
    struct miscdevice * misc = (struct miscdevice *) dev_get_drvdata(dev);
    return container_of(misc, struct sensorhub_drv_data, misc);
}

static ssize_t sensorhub_i2c_sysfs_store_suspend(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count) {

    struct sensorhub_drv_data * ddata = __misc(dev);
    int rc = 0;
    if (strncmp(buf, "1\n", count) == 0 || strncmp(buf, "1", count) == 0) {
        sensorhub_i2c_setpower(ddata, SH_HID_PWR_SLEEP);
    } else {
        sensorhub_i2c_setpower(ddata, SH_HID_PWR_ON);
    }

    return rc ? rc : count;
}

static DEVICE_ATTR(suspend, S_IWUSR, NULL, sensorhub_i2c_sysfs_store_suspend);
static struct attribute * sensorhub_i2c_drv_attrs[] = {
    &dev_attr_suspend.attr,
    NULL /* keep last*/
};

static const struct attribute_group sensorhub_i2c_attr_group = {
    .attrs = sensorhub_i2c_drv_attrs,
};
#endif

#ifdef CONFIG_OF
static struct of_device_id sensorhub_i2c_match_table[] = {
    {.compatible = "sensorhub",},
    { },
};
#else
#define sensorhub_i2c_match_table NULL
#endif

// TODO -- is id table useful? what to do with it?
static const struct i2c_device_id sensorhub_i2c_id[] = {
    { "sensorhub", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, sensorhub_i2c_id);

static const struct sensorhub_transport_callbacks i2c_sensorhub_transport = {
    .get_report = sensorhub_i2c_get_report,
    .set_report = sensorhub_i2c_set_report,
    .reset      = sensorhub_i2c_reset,

#ifdef CONFIG_SENSOR_HUB_I2C_ATMEL_DFU
    .dfu_abort    = sensorhub_i2c_dfu_abort,
    .dfu_perform  = sensorhub_i2c_dfu_perform,
    .dfu_validate = sensorhub_i2c_dfu_fw_validate
#else
        .dfu_abort    = NULL,
    .dfu_perform  = NULL,
    .dfu_validate = NULL
#endif
};

static struct i2c_driver sensorhub_i2c_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name  = "sensorhub",
        .pm    = SH_PM_OPS,
#ifdef CONFIG_OF
        .of_match_table = sensorhub_i2c_match_table,
#endif
    },

    .id_table = sensorhub_i2c_id,
    .probe    = sensorhub_i2c_probe,
    .remove   = sensorhub_i2c_remove,

};

static int sensorhub_i2c_init(void) {
    return i2c_add_driver(&sensorhub_i2c_driver);
}

static void sensorhub_i2c_exit(void) {
    i2c_del_driver(&sensorhub_i2c_driver);
}

module_init(sensorhub_i2c_init);
module_exit(sensorhub_i2c_exit);

MODULE_DESCRIPTION("Freespace SensorHub Driver");
MODULE_AUTHOR("Khalid Zubair <kzubair@hcrest.com>");
MODULE_VERSION(SENSORHUB_VERSION);
MODULE_LICENSE("GPL");
