#ifndef __YL_PM8916_VBUS_H__
#define __YL_PM8916_VBUS_H__

int get_yl_pm8916_vbus_status(void);
int get_yl_pm8916_batt_mvol(void);
int get_yl_pm8916_batt_temp(void);
int read_yl_pm8916_batt_id(void);
int is_battery_present(void);
int yl_backup_ocv_soc(int ocv_uv, int soc);
int yl_read_shutdown_ocv_soc(int * ocv_uv,int * soc);

#endif /* __YL_PM8916_VBUS_H__ */
