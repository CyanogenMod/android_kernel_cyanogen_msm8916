#ifndef LINUX_GT9XX_FIRMWARE_H
#define LINUX_GT9XX_FIRMWARE_H

#ifdef HEADER_UPDATE_DATA
#undef HEADER_UPDATE_DATA
#endif

#define HEADER_UPDATE_DATA goodix_firmware

#ifdef CONFIG_MACH_CP8675
const unsigned char goodix_firmware[] = {
	#include "GT970_1030(5A1F)"
};
#else
const unsigned char goodix_firmware[] = {
	#include "GT9158_1034_A9D0"
};	
#endif


#endif
