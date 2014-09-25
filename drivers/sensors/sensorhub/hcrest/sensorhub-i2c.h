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

#ifndef __SENSORHUB_I2C__
#define __SENSORHUB_I2C__ __FILE__

#include "sensorhub-core.h"

#define SH_HID_DESC_V1_LEN                      30
#define SH_HID_DESC_V1_BCD                      0x0100
#define SH_HID_REG_HID_DESCRIPTOR               0x01
#define SH_HID_OP_CODE_REPORT_ID_SENTINEL       0xf

struct sensorhub_i2c_hid_descriptor {
	uint16_t wHIDDescLength;
	uint16_t bcdVersion;
	uint16_t wReportDescriptorLength;
	uint16_t wReportDescriptorRegister;
	uint16_t wInputRegister;
	uint16_t wMaxInputLength;
	uint16_t wOutputRegister;
	uint16_t wMaxOutputLength;
	uint16_t wCommandRegister;
	uint16_t wDataRegister;
	uint16_t wVendorID;
	uint16_t wProductID;
	uint16_t wVersionID;
	uint8_t reserved[4];
} __packed;

enum sensorhub_i2c_hid_op_code { /* cmd = 0000 XXXX 0000 0000 */
	SH_HID_OP_CODE_RESET           = 0x0100,
	SH_HID_OP_CODE_GET_REPORT      = 0x0200,
	SH_HID_OP_CODE_SET_REPORT      = 0x0300,
	SH_HID_OP_CODE_GET_IDLE        = 0x0400,
	SH_HID_OP_CODE_SET_IDLE        = 0x0500,
	SH_HID_OP_CODE_GET_PROTOCOL    = 0x0600,
	SH_HID_OP_CODE_SET_PROTOCOL    = 0x0700,
	SH_HID_OP_CODE_SET_POWER       = 0x0800,
	SH_HID_OP_CODE_VENDOR_RESERVED = 0x0e00,
};

enum sensorhub_i2c_hid_report_type { /* cmd = 0000 0000 00XX 0000 */
	SH_HID_REPORT_TYPE_INPUT       = 0x0010,
	SH_HID_REPORT_TYPE_OUTPUT      = 0x0020,
	SH_HID_REPORT_TYPE_FEATURE     = 0x0030,
};

enum sensorhub_i2c_hid_power_state { /* cmd = 0000 0000 0000 000X */
	SH_HID_PWR_ON                  = 0,
	SH_HID_PWR_SLEEP               = 1,
};

struct sensorhub_i2c_data {
	/* take this lock before accessing hw (client) and modifying transfer stats */
	struct mutex hw_lock;
	struct i2c_client * client;

	struct sensorhub_i2c_hid_descriptor desc;
	struct sensorhub_drv_data ddata;

	struct {
		u64 timestamp;
	} irq;

	struct {
		struct completion compl;
	} bl;
};

#endif
