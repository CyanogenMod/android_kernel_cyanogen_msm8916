#ifndef _COLOR_ENHANCEMENT_H
#define _COLOR_ENHANCEMENT_H

#include <linux/miscdevice.h>
#include <linux/ioctl.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>

struct color_enhancement_t {
	struct ce_impl_ops_t *impl_ops;
	void *impl_data;
	bool initialized;
};

struct ce_impl_ops_t {
	int (*init)(struct color_enhancement_t *);
	int (*enable)(struct color_enhancement_t *);
	int (*disable)(struct color_enhancement_t *);
	int (*set_level)(struct color_enhancement_t *, int level);
	int (*restore_default)(struct color_enhancement_t *);
	int (*cabc_off)(struct color_enhancement_t *);
	int (*ui_mode)(struct color_enhancement_t *);
	int (*still_mode)(struct color_enhancement_t *);
	int (*moving_mode)(struct color_enhancement_t *);
	int (*aco)(struct color_enhancement_t *, int on);
	int (*osc)(struct color_enhancement_t *, int weak);
	int (*init_setting)(struct color_enhancement_t *, int setting);
};

#define CE_IOC_BASE 'C'

#define CE_ENABLE			_IO(CE_IOC_BASE,0)
#define CE_DISABLE			_IO(CE_IOC_BASE,1)
#define CE_SET_LEVEL			_IOR(CE_IOC_BASE,2,int)
#define CE_SET_CABC_OFF			_IO(CE_IOC_BASE,3)
#define CE_SET_CABC_UI_MODE		_IO(CE_IOC_BASE,4)
#define CE_SET_CABC_STILL_MODE		_IO(CE_IOC_BASE,5)
#define CE_SET_CABC_MOVING_MODE		_IO(CE_IOC_BASE,6)
#define CE_RESTORE_DEFAULT		_IO(CE_IOC_BASE,7)
#define CE_SET_DAYLIGHT_MODE		_IO(CE_IOC_BASE,8)
#define CE_SET_ACO_ON			_IO(CE_IOC_BASE,9)
#define CE_SET_ACO_OFF			_IO(CE_IOC_BASE,10)
#define CE_SET_OSC_ON			_IO(CE_IOC_BASE,11)
#define CE_SET_OSC_OFF			_IO(CE_IOC_BASE,12)
#define CE_INIT_SETTING			_IOR(CE_IOC_BASE,13,int)


#define EN		0x0001
#define DEFAULT		0x0002
#define LEVEL1		0x0004
#define LEVEL2		0x0008
#define CE_MASK		0x000E
#define CABC_MOVING	0x0010
#define ACO_ON		0x0100
#define OSC_WEAK	0x1000

#define DISABLE		CABC_MOVING
#define STANDARD	(EN | DEFAULT | OSC_WEAK | CABC_MOVING)
#define PICTURE		(EN | LEVEL1 | OSC_WEAK | ACO_ON)
#define MOVIE		(EN | LEVEL2 | CABC_MOVING)
#define DEFAULT_SETTING	(EN | DEFAULT | CABC_MOVING | OSC_WEAK | 0x8000)

#endif
