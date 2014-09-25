#ifndef __LC709203_FUELGAUGER_YL_H__
#define __LC709203_FUELGAUGER_YL_H__
/* LC709203 standard data commands */
#define LC709203_REG_SET_THERMB	0x06
#define LC709203_REG_INIT_RSOC		0x07
#define LC709203_REG_CELL_TEMP		0x08
#define LC709203_REG_CELL_VOLT		0x09
#define LC709203_REG_ADJ_APPLI		0x0B
#define LC709203_REG_ADJ_THERM		0x0C
#define LC709203_REG_RSOC			0x0D
#define LC709203_REG_RTIME			0x0F
#define LC709203_REG_IC_VER			0x11
#define LC709203_REG_RSOC_ALM	0x13
#define LC709203_REG_LOW_CELL_VOL_ALM   0x14
#define LC709203_REG_PW_Mode	0x15
#define LC709203_REG_EN_NTC			0x16

extern int lc709203_get_batt_property(unsigned char command, int *data);
#ifdef CONFIG_CHARGER_HAVE_LC709203_FUEL
void lc709203_hw_init_by_battery(void);
#endif
#endif
