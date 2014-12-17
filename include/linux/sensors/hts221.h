/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
*
* File Name          : hts221.c
* Authors            : Motion MEMS
*                      Morris Chen (morris.chen@st.com)
* Version            : V.1.0.2
* Date               : 08/11/2013
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
********************************************************************************
********************************************************************************
Version History.
 
Revision 1-0-0 06/02/2013
 first revision
 
Revision 1-0-1 07/23/2013
 Optimization
 
Revision 1-0-2 8/11/2013
 Increase Accuracy
   a. multiplying by 8000 instead of 1000
   b. the conversion of the read values,  by shifting 4 (H case) or 6 (T) bit 
*******************************************************************************/
 
#ifndef __HTS221_H__
#define __HTS221_H__
 
#define HTS221_DEV_NAME  "hts221"

#define HTS221_DEV_NAME_H   "hts221_humidity"
#define HTS221_DEV_NAME_T   "hts221_temperature"
 
/************************************************/
/*      Output data                             */
/*************************************************
humidity: relative humidity (m RH)
temperature: celsius degree (m DegC)
*************************************************/
 
/************************************************/
/*      sysfs data                              */
/*************************************************
        - pollrate->ms
humidity:
        - humidity_resolution->number
temperature:
        - temperature_resolution->number
*************************************************/
 
 
/* Humidity Sensor Resolution */
#define HTS221_H_RESOLUTION_4    (0x00)  /* Resolution set to 0.4 %RH */
#define HTS221_H_RESOLUTION_8    (0x01)  /* Resolution set to 0.3 %RH */
#define HTS221_H_RESOLUTION_16   (0x02)  /* Resolution set to 0.2 %RH */
#define HTS221_H_RESOLUTION_32   (0x03)  /* Resolution set to 0.15 %RH */
#define HTS221_H_RESOLUTION_64   (0x04)  /* Resolution set to 0.1 %RH */
#define HTS221_H_RESOLUTION_128  (0x05)  /* Resolution set to 0.07 %RH */
#define HTS221_H_RESOLUTION_256  (0x06)  /* Resolution set to 0.05 %RH */
#define HTS221_H_RESOLUTION_512  (0x07)  /* Resolution set to 0.03 %RH */
 
/* Temperature Sensor Resolution */
#define HTS221_T_RESOLUTION_2    (0x00)  /* Resolution set to 0.08 DegC */
#define HTS221_T_RESOLUTION_4    (0x08)  /* Resolution set to 0.05 DegC */
#define HTS221_T_RESOLUTION_8    (0x10)  /* Resolution set to 0.04 DegC */
#define HTS221_T_RESOLUTION_16   (0x18)  /* Resolution set to 0.03 DegC */
#define HTS221_T_RESOLUTION_32   (0x20)  /* Resolution set to 0.02 DegC */
#define HTS221_T_RESOLUTION_64   (0x28)  /* Resolution set to 0.015 DegC */
#define HTS221_T_RESOLUTION_128  (0x30)  /* Resolution set to 0.01 DegC */
#define HTS221_T_RESOLUTION_256  (0x38)  /* Resolution set to 0.007 DegC */
 
 
#ifdef  __KERNEL__
 
#define DEFAULT_INT1_GPIO               (-EINVAL)
#define DEFAULT_INT2_GPIO               (-EINVAL)
 
#define HTS221_MIN_POLL_PERIOD_MS        100
#define HTS221_DEFAULT_POLL_PERIOD_MS	2000
 
struct hts221_platform_data {
 
        unsigned int poll_interval;
        unsigned int min_interval;
 
        u8 h_resolution;
        u8 t_resolution;
 
        int (*init)(void);
        void (*exit)(void);
        int (*power_on)(void);
        int (*power_off)(void);
};
 
#endif  /* __KERNEL__ */
#endif  /* __HTS221_H__ */
