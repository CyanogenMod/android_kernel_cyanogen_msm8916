#ifndef __BQ27541_FUELGAUGER_YL_H__
#define __BQ27541_FUELGAUGER_YL_H__

struct yl_battery_gauge {
	const char *name;
	int (*get_batt_mvolts) (void);
	int (*get_batt_temperature) (void);
	int (*get_batt_current) (void);
	int (*get_batt_capacity) (void);
	int (*get_batt_full_capacity) (void);
	int (*get_batt_rm_capacity) (void);
	int (*get_batt_status)(void);
	int (*is_batt_present) (void);
	int (*is_batt_temp_within_range) (void);
	int (*is_batt_id_valid) (void);
	int (*check_batt_present) (void);
};

void yl_battery_gauge_register(struct yl_battery_gauge *batt_gauge);
void yl_battery_gauge_unregister(struct yl_battery_gauge *batt_gauge);

#endif
