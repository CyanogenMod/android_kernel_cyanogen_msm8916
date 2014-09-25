#ifndef __BMA250E_H__
#define __BMA250E_H__

#include <linux/sensors/accel_common.h>

#define BMA2x2_DEVICE_NAME			"yl_acc_sensor"
#define BMA2x2_INPUT_NAME			"bma2x2"

#define BMA2x2_IOCTL_APP_SET_AFLAG	ACCEL_IOCTL_ACTIVE
#define BMA2x2_IOCTL_APP_SET_DELAY	ACCEL_IOCTL_SET_DELAY
#define BMA2x2_IOCTL_APP_CALIBRATE	ACCEL_IOCTL_CALIBRATE
#define BMA2x2_IOCTL_APP_OFFSET	    ACCEL_IOCTL_SETOFFSET //modifed by wudongxing for factory pattern,2013.11.7

struct bosch_accel_platform_data {
	/* Although the accelerometer can perform at high ODR,
	 * there is a need to keep the maximum ODR to a lower
	 * value due to power consumption or other concern.
	 * Use this variable to set the minimum allowable
	 * interval for data to be reported from the
	 * accelerometer. Unit is measured in milli-
	 * seconds. Recommended value is 5ms. */
	unsigned int min_interval;
	/* Use this variable to set the default interval for
	 * data to be reported from the accelerometer. This
	 * value will be used during driver setup process,
	 * but can be changed by the system during runtime via
	 * sysfs control. Recommended value is 200ms.*/
	unsigned int poll_interval;

	/* This variable controls the corresponding direction
	  */
	unsigned char axis_map_x;
	unsigned char axis_map_y;
	unsigned char axis_map_z;

	unsigned char negative_x;
	unsigned char negative_y;
	unsigned char negative_z;

	/* Use this variable to control the G range of
	 * the accelerometer output. Use the macro definition
	 * to select the desired G range.*/

	unsigned char acc_range;
};

struct bma2x2acc {
	short x,
		  y,
		  z;
} ;



#endif
