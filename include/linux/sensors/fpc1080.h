#ifndef LINUX_SPI_FPC1080_H
#define LINUX_SPI_FPC1080_H

struct fpc1080_nav_settings {
	u8 finger_down_min;
	u8 finger_down_mid;
	u8 finger_down_max;
	u8 finger_detect_thr;
	u8 finger_lost_thr;
	u8 dx_thr;
	u8 dy_thr;
	u8 nav_cntr;
	u8 off_x_axis_thr;
	u8 off_y_axis_thr;
	u8 update;
	u8 enabled;
};

struct fpc1080_adc_setup {
	u8 gain;
	u8 offset;
	u8 pxl_setup;
};

struct fpc1080_platform_data {
	int irq_gpio;
	int reset_gpio;
	struct fpc1080_adc_setup adc_setup;
	struct fpc1080_nav_settings nav;
};

#endif
