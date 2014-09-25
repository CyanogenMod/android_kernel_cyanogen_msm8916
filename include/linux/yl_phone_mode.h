#ifndef YL_PHONE_MODE_H
#define YL_PHONE_MODE_H

#define COMM_RUN_MODE_RELEASE   '0'
#define COMM_RUN_MODE_RUWANG    '1'
#define COMM_RUN_MODE_XIANWANG  '2'
#define COMM_RUN_MODE_RF        '3'
#define COMM_RUN_MODE_DEBUG     '4'

int yl_phone_mode_is_xianwang(void);

#endif
