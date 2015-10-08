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

#ifndef _BOOT_PARAMS_H
#define _BOOT_PARAMS_H

enum {
	BOOT_MODE_SYSTEM = 0,
	BOOT_MODE_CHARGER,
	BOOT_MODE_RECOVERY
};

#ifdef CONFIG_YL_POWEROFF_ALARM
int get_poweroff_alarm_flag(void);
unsigned long get_poweroff_alarm_time(void);
#endif
int get_boot_mode(void);
int get_pon_batt_volt(void);

#endif /* _BOOT_PARAMS_H */
