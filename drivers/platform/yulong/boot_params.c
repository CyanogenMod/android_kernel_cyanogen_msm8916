/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <asm/setup.h>

#include <linux/export.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include <linux/boot_params.h>

#ifdef CONFIG_YL_POWEROFF_ALARM
static unsigned long hex2unsignedlong(const char *x)
{
	unsigned long n = 0;

	while (*x) {
		switch(*x) {
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
			n = (n << 4) | (*x - '0');
			break;
		case 'a':
		case 'b':
		case 'c':
		case 'd':
		case 'e':
		case 'f':
			n = (n << 4) | (*x - 'a' + 10);
			break;
		case 'A':
		case 'B':
		case 'C':
		case 'D':
		case 'E':
		case 'F':
			n = (n << 4) | (*x - 'A' + 10);
			break;
		default:
			return n;
		}
		x++;
	}

	return n;
}

static int poweroff_alarm_flag = 0;
int get_poweroff_alarm_flag(void)
{
	return poweroff_alarm_flag;
}
EXPORT_SYMBOL(get_poweroff_alarm_flag);

static int __init init_poweroff_alarm_flag(char *s)
{
	if (!strncmp(s, "0", 1))
		poweroff_alarm_flag = 0;
	else if (!strncmp(s, "1", 1))
		poweroff_alarm_flag = 1;

	return 0;
}
__setup("poweroff_alarm_flag=", init_poweroff_alarm_flag);

static unsigned long poweroff_alarm_secs = 0;
unsigned long get_poweroff_alarm_time(void)
{
	return poweroff_alarm_secs;
}
EXPORT_SYMBOL(get_poweroff_alarm_time);

static int __init init_poweroff_alarm_time(char *s)
{
	char time[16];

	memset(time, 0, 16);
	strlcpy(time, s, sizeof(time));
	poweroff_alarm_secs = hex2unsignedlong(time);

	return 0;
}
__setup("poweroff_alarm_time=", init_poweroff_alarm_time);
#endif

static int boot_mode = 0;
int get_boot_mode(void)
{
	return boot_mode;
}
EXPORT_SYMBOL(get_boot_mode);

static int __init init_boot_mode(char *s)
{
	if (!strncmp(s, "charger", 7)) {
		boot_mode = BOOT_MODE_CHARGER;
	} else if (!strncmp(s, "recovery", 8)) {
		boot_mode = BOOT_MODE_RECOVERY;
	} else {
		boot_mode = BOOT_MODE_SYSTEM;
	}

	return 0;
}
__setup("androidboot.mode=", init_boot_mode);

static int pon_batt_volt = 0;
int get_pon_batt_volt(void)
{
	return pon_batt_volt;
}
EXPORT_SYMBOL(get_pon_batt_volt);

static int __init init_pon_batt_volt(char *s)
{
	pon_batt_volt = (int) simple_strtol(s, NULL, 10);

	return 0;
}
__setup("pon_vol=", init_pon_batt_volt);
