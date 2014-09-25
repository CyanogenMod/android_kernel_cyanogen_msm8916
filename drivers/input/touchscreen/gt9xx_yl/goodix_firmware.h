#ifndef LINUX_GT9XX_FIRMWARE_H
#define LINUX_GT9XX_FIRMWARE_H

#ifdef HEADER_UPDATE_DATA
#undef HEADER_UPDATE_DATA
#endif

#define HEADER_UPDATE_DATA goodix_firmware

#if defined(CONFIG_BOARD_CP8675_C00)||defined(CONFIG_BOARD_CP8675_W00)||defined(CONFIG_BOARD_CP8675_T00)||defined(CONFIG_BOARD_CP8675_A)||defined(CONFIG_BOARD_CP8675_I01)
const unsigned char goodix_firmware[] = {
	#include "GT970_1030(5A1F)"
};
#else
const unsigned char goodix_firmware[] = {
	#include "GT9158_1034_A9D0"
};	
#endif


#endif
