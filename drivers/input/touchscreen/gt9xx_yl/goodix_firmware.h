#ifndef LINUX_GT9XX_FIRMWARE_H
#define LINUX_GT9XX_FIRMWARE_H

#ifdef HEADER_UPDATE_DATA
#undef HEADER_UPDATE_DATA
#endif

#define HEADER_UPDATE_DATA goodix_firmware

#ifdef CONFIG_MACH_CP8675
const unsigned char goodix_firmware[] = {
#ifdef CONFIG_TOUCHSCREEN_GT9XX_YL_COVER_WINDOW_CFG
	#include "GT970_1039_DACA"
#else
	#include "GT970_1030_5A1F"
#endif
};
#else
const unsigned char goodix_firmware[] = {
	#include "GT9158_1034_A9D0"
};	
#endif


#endif
