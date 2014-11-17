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

#ifndef _LEDS_AW_LEDS_H
#define _LEDS_AW_LEDS_H

#include <linux/ioctl.h>
#include <linux/types.h>

struct aw_leds_control {
	__u8 red;
	__u8 green;
	__u8 blue;
	__u8 blink;
	__u32 rise_time;
	__u32 fall_time;
	__u32 hold_time;
	__u32 off_time;
}__packed;

#define AW_LEDS_SET_LEDS	_IOW('S', 42, struct aw_leds_control)
#endif						/* _LEDS_AW_LEDS_H */
