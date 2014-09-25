/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef MSM_LC898212_FOXCON_H
#define MSM_LC898212_FOXCON_H
 
#include <linux/i2c.h>
#include <linux/gpio.h>
//#include <mach/camera2.h>
#include <soc/qcom/camera2.h>
#include <media/v4l2-subdev.h>
#include <media/msmb_camera.h>
#include "msm_camera_i2c.h"
#include "msm_actuator.h"

typedef struct STMVPAR {
	uint16_t UsSmvSiz ;
	uint16_t UcSmvItv ;
	uint16_t UcSmvEnb ;
} stSmvPar ;

#define SENSOR_KERNEL_DEBUG 1

#if SENSOR_KERNEL_DEBUG
struct test_info_t {
	struct msm_actuator_ctrl_t *cn_a_ctrl;
	int msleep_time;
	int step_interval;
	int flag_debug_foc;
	int pos_high;
	int pos_low;
	int set_lens_pos;
	int enc_set_lens_pos;
	int enc_actual_lens_pos;
};
#endif

#define   	WAIT				0xFF      	// Wait command
#define REG_ADDR_START  0x80		// REG Start address
#define	abs_closeloop(x)	((x) < 0 ? -(x) : (x))

/*----------------------------------------------------------
		Initial data table
-----------------------------------------------------------*/
struct INIDATA {
	unsigned short	addr ;
	unsigned short	data ;
};	

/*
function call flow:

void main(void){
    stSmvPar StSmvPar;
    int HallOff = 0x80;	 	// Please Read Offset from EEPROM or OTP
    int HallBiase = 0x80;   // Please Read Bias from EEPROM or OTP
    msm_LC898212_init(a_ctrl,HallOff, HallBiase );	// Initialize driver IC
    // Step move parameter set
    StSmvPar.UsSmvSiz	= STMV_SIZE ;
    StSmvPar.UcSmvItv	= STMV_INTERVAL ;
    StSmvPar.UcSmvEnb	= STMCHTG_SET | STMSV_SET | STMLFF_SET ;
    msm_LC898212_set(a_ctrl,StSmvPar ) ;
    msm_LC898212_on(a_ctrl);	// Close loop ON
    msm_LC898212_moveto(a_ctrl,msm_LC898212_convert(500) ) ;	// Move to Target Position
}
*/
//********************************************************************************
// Function Name 	: msm_LC898212_convert
// Retun Value		: NON
// Argment Value	: code
// Explanation		: Af code convert
// History			: 				
//********************************************************************************   
int msm_LC898212_convert(uint16_t code);
//********************************************************************************
// Function Name 	: msm_LC898212_init
// Retun Value		: NON
// Argment Value	: Hall Bias, Hall Offset
// Explanation		: Af Initial Function
// History			: 				
//********************************************************************************   
void 	msm_LC898212_init(struct msm_actuator_ctrl_t *a_ctrl,uint16_t hall_bias, uint16_t hall_off );
/*-----------------------------------------------------------
    Function Name   : StmvSet, StmvTo, StmvEnd
	Description     : StepMove Setting, Execute, Finish, Current Limit 
    Arguments       : Stepmove Mode Parameter
	Return			: Stepmove Mode Parameter
-----------------------------------------------------------*/
//StmvSet -> StmvTo -> StmvEnd -> StmvTo -> StmvEnd ->ÅEÅEÅE

//********************************************************************************
// Function Name 	: msm_LC898212_set
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Stpmove parameter Setting Function
// History			: 
// History			: 
//********************************************************************************
void msm_LC898212_set(struct msm_actuator_ctrl_t *a_ctrl,stSmvPar StSetSmv );
//********************************************************************************
// Function Name 	: msm_LC898212_on
// Retun Value		: NON
// Argment Value	: None
// Explanation		: Servo On function
// History			: 
//********************************************************************************
void msm_LC898212_on(struct msm_actuator_ctrl_t *a_ctrl);
//********************************************************************************
// Function Name 	: msm_LC898212_end
// Retun Value		: Stepmove Parameter
// Argment Value	: Stepmove Parameter
// Explanation		: Stpmove Finish Check Function
// History			: First edition 					
// History			: Changes					
//********************************************************************************
uint16_t msm_LC898212_end(struct msm_actuator_ctrl_t *a_ctrl, uint16_t UcParMod);
//********************************************************************************
// Function Name 	: StmvTo
// Retun Value		: Stepmove Parameter
// Argment Value	: Stepmove Parameter, Target Position
// Explanation		: Stpmove Function
// History			: 
// History			: 
//********************************************************************************
uint16_t msm_LC898212_moveto(struct msm_actuator_ctrl_t *a_ctrl,uint16_t SsSmvEnd );
//********************************************************************************
// Function Name 	: msm_LC898212_InitForPlt
// Retun Value		: 
// Argment Value	: 
// Explanation		: 
// History			: First edition 					
// History			: Changes					
//********************************************************************************
void  msm_LC898212_InitForQualcomm(struct msm_actuator_ctrl_t *a_ctrl);
#endif
