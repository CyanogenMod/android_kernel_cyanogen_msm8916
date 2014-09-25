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
#include <linux/module.h>
#include "msm_sd.h"
#include "msm_cci.h"
#include "msm_LC898212_foxconn.h"
#include "msm_LC898212_foxconn_data.h"

//#define LOGCL(fmt, args...) printk("[CL] %s() "fmt"\n", __func__, ##args)
#define LOGCL(fmt, args...) do{ }while(0)

extern uint16_t closeloop_HallOff;	// see msm_sensor.c
extern uint16_t closeloop_HallBiase;	// see msm_sensor.c
extern uint16_t closeloop_HallMax;	// see msm_sensor.c
extern uint16_t  closeloop_HallMin;	// see msm_sensor.c
extern struct test_info_t tinfo;	//msm_actuator.c
static 	stSmvPar StSmvPar;

int msm_LC898212_convert(uint16_t code){
	int HallPos;
    int	HallLow = (closeloop_HallMin < 0x8001) ? -closeloop_HallMin : 0x10000 - closeloop_HallMin;		
    int	HallUp   = (closeloop_HallMax > 0x7FFF) ? closeloop_HallMax - 0x10000 : closeloop_HallMax;   
    int	HallRange = HallLow + HallUp;
    //int	PosRange = Max_Pos - Min_Pos;
    int	PosRange = abs_closeloop(tinfo.pos_high - tinfo.pos_low);
#if 0  	// 1: Move with up direction; //QC8974 use 0 as Near and 1023 as Far.
    //LOGCL("check 2 dir pos, HallPos: %d, %d.\n", (((code - tinfo.pos_low) * HallRange / PosRange) + closeloop_HallMin), (((tinfo.pos_high - code) * HallRange / PosRange) + closeloop_HallMin));
    //HallPos = ((position - Min_Pos) * HallRange / PosRange) + Hall_Min;
    HallPos = ((code - tinfo.pos_low) * HallRange / PosRange) + closeloop_HallMin;
#else  // 0: Move with down direction
    //HallPos = ((Max_Pos - code) * HallRange / PosRange) + closeloop_HallMin;
    HallPos = ((tinfo.pos_high - code) * HallRange / PosRange) + closeloop_HallMin;
#endif
    LOGCL(" closeloop_HallMin: 0x%x, closeloop_HallMax: 0x%x, HallLow: 0x%x, HallUp: 0x%x, HallRange: 0x%x, PosRange: 0x%x.\n", closeloop_HallMin, closeloop_HallMax, HallLow, HallUp, HallRange, PosRange);
/*
	if(HallPos > 0x10000){
       HallPos = HallPos - 0x10000;
    }
*/
    return HallPos & 0xFFFF; 	
}

//********************************************************************************
// Function Name 	: msm_LC898212_init
// Retun Value		: NON
// Argment Value	: Hall Bias, Hall OffSet
// Explanation		: Af Initial Function
// History			: 				
//********************************************************************************   
void msm_LC898212_init(struct msm_actuator_ctrl_t *a_ctrl,uint16_t hall_bias, uint16_t hall_off )
{
    uint8_t i;
    //uint16_t regData;
    int err;
    uint16_t pos;
    LOGCL("[FUN] %s() +++++\n", __func__);
    //write Init_Table
    for(i = 0; i < DataLen; i++){
        if(Init_Table[i].addr == WAIT) {
        	msleep(Init_Table[i].data);
        	continue;
       }
    if(Init_Table[i].addr >= REG_ADDR_START){
      err = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client, Init_Table[i].addr,(Init_Table[i].data & 0x00ff), MSM_CAMERA_I2C_BYTE_DATA);
    }else{
      err = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client,Init_Table[i].addr,(Init_Table[i].data), MSM_CAMERA_I2C_WORD_DATA);    
    } 
   }

    a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client,0x28,closeloop_HallOff/*hall_off*/, MSM_CAMERA_I2C_BYTE_DATA);//0x41    
    a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client,0x29,closeloop_HallBiase/*hall_bias*/, MSM_CAMERA_I2C_BYTE_DATA);//0x32    
    a_ctrl->i2c_client.i2c_func_tbl->i2c_read(&a_ctrl->i2c_client, 0x3C,&pos, MSM_CAMERA_I2C_WORD_DATA);
    LOGCL("Initial, read from 0x3C, data is: 0x%x", pos);                                      
    a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client,0x04,pos, MSM_CAMERA_I2C_WORD_DATA);    
    a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client,0x18,pos, MSM_CAMERA_I2C_WORD_DATA);

    LOGCL("msm_LC898212_init LC898212 exit\n");
}

//********************************************************************************
// Function Name 	: msm_LC898212_on
// Retun Value		: NON
// Argment Value	: None
// Explanation		: Servo On function
// History			: 
//********************************************************************************
void msm_LC898212_on(struct msm_actuator_ctrl_t *a_ctrl){
    LOGCL("[FUN] %s() +++++\n", __func__);
    LOGCL("msm_LC898212_on LC898212 enter\n");
    msleep(5);
    a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client,0x87,0x85, MSM_CAMERA_I2C_BYTE_DATA);    
    LOGCL("msm_LC898212_on LC898212 exit\n");
}
/*-----------------------------------------------------------
    Function Name   : StmvSet, StmvTo, StmvEnd
	Description     : StepMove Setting, Execute, Finish, Current Limit 
    Arguments       : Stepmove Mode Parameter
	Return			: Stepmove Mode Parameter
-----------------------------------------------------------*/
//StmvSet -> StmvTo -> StmvEnd -> StmvTo -> StmvEnd ->...

//********************************************************************************
// Function Name 	: msm_LC898212_set
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Stpmove parameter Setting Function
// History			: First edition 	
// History			: Changes	
//********************************************************************************
void msm_LC898212_set(struct msm_actuator_ctrl_t *a_ctrl,stSmvPar StSetSmv )
{
	uint16_t UcSetEnb;
	uint16_t UcSetSwt;
	uint16_t UsParSiz;
	uint16_t UcParItv;
	uint16_t SsParStt;	// StepMove Start Position

	StSmvPar.UsSmvSiz = StSetSmv.UsSmvSiz;
	StSmvPar.UcSmvItv = StSetSmv.UcSmvItv;
	StSmvPar.UcSmvEnb = StSetSmv.UcSmvEnb;
	LOGCL("%s LC898212 enter\n",__func__);
        LOGCL("[FUN] %s() +++++\n", __func__);

	// StepMove Enable Bit Clear
	a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client,AFSEND_211, 0x00,MSM_CAMERA_I2C_BYTE_DATA);
        a_ctrl->i2c_client.i2c_func_tbl->i2c_read(&a_ctrl->i2c_client,ENBL_211,&UcSetEnb,MSM_CAMERA_I2C_BYTE_DATA);  
	UcSetEnb  &= 0xFD ;//xxxx xx0x
	// Measuremenet Circuit1 Off
	a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client,ENBL_211,UcSetEnb,MSM_CAMERA_I2C_BYTE_DATA);
	a_ctrl->i2c_client.i2c_func_tbl->i2c_read(&a_ctrl->i2c_client,SWTCH_211,&UcSetSwt,MSM_CAMERA_I2C_BYTE_DATA);  
        UcSetSwt  &= 0x7F ;//0xxx xxxx
	// RZ1 Switch Cut Off
        LOGCL(" A .write to SWTCH_211(0x%x) 0x%x.\n.", SWTCH_211, UcSetSwt);
	a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client,SWTCH_211,UcSetSwt,MSM_CAMERA_I2C_BYTE_DATA);
	// Get Start Position
        a_ctrl->i2c_client.i2c_func_tbl->i2c_read(&a_ctrl->i2c_client,RZ_211H,&SsParStt,MSM_CAMERA_I2C_WORD_DATA); //read Reg0x04
        LOGCL("Set parameters, read from 0x04, data is: 0x%x", SsParStt);
        //a_ctrl->i2c_client.i2c_func_tbl->i2c_read(&a_ctrl->i2c_client,MS1Z22_211H,&SsParStt,MSM_CAMERA_I2C_WORD_DATA);   //read Reg0x18
        //LOGCL("Set parameters, read from 0x18, data is: 0x%x", SsParStt);
	// Get StepSize
	UsParSiz =  StSetSmv.UsSmvSiz ;
	// Get StepInterval
	UcParItv =  StSetSmv.UcSmvItv ;
	UcParItv =  tinfo.step_interval;//cn test
	// Set Coefficient Value For StepMove
	a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client,ms11a_211H,0x0800,MSM_CAMERA_I2C_WORD_DATA);	
	// Set Start Positon
	a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client,MS1Z22_211H,SsParStt,MSM_CAMERA_I2C_WORD_DATA);
	// Set StepSize
	a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client,MS1Z12_211H,UsParSiz,MSM_CAMERA_I2C_WORD_DATA);
	// Set StepInterval
        printk(" write to STMINT_211(0x%x) 0x%x.\n.", STMINT_211, UcParItv);
	a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client,STMINT_211,UcParItv,MSM_CAMERA_I2C_BYTE_DATA);
	UcSetSwt  |= 0x80;//1xxx xxxx
	// RZ1 Switch ON
        LOGCL(" B .write to SWTCH_211(0x%x) 0x%x.\n.", SWTCH_211, UcSetSwt);
	a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client,SWTCH_211,UcSetSwt,MSM_CAMERA_I2C_BYTE_DATA);

       LOGCL("%s LC898212 exit\n",__func__);
}

//********************************************************************************
// Function Name 	: msm_LC898212_end
// Retun Value		: Stepmove Parameter
// Argment Value	: Stepmove Parameter
// Explanation		: Stpmove Finish Check Function
// History			: First edition 					
// History			: Changes					
//********************************************************************************
uint16_t msm_LC898212_end(struct msm_actuator_ctrl_t *a_ctrl, uint16_t UcParMod){
     uint16_t UcChtGst;
     //uint16_t UcParMod_t;
     uint8_t  i = 0;
     uint16_t regPos;
     LOGCL("[FUN] %s() +++++\n", __func__);
     LOGCL("msm_LC898212_end LC898212 enter, 1, get initial UcParMod:0x%x.\n", UcParMod);
     // Wait StepMove operation end
     while( (UcParMod & STMVEN_ON ) && (i++ < 100)){
        a_ctrl->i2c_client.i2c_func_tbl->i2c_read(&a_ctrl->i2c_client,STMVEN_211 ,&UcParMod,MSM_CAMERA_I2C_BYTE_DATA);  
        LOGCL(" 2. check stepmove if it is end: 0x%x, UcParMod=0x%x.\n", UcParMod & STMVEN_ON, UcParMod);
     }
     LOGCL(" 3. after while, UcParMod=0x%x.\n", UcParMod);
     a_ctrl->i2c_client.i2c_func_tbl->i2c_read(&a_ctrl->i2c_client,0x3C, &regPos,MSM_CAMERA_I2C_WORD_DATA);
     LOGCL(" then read Reg0x3C: 0x%x.\n", regPos);

     if(STMCHTG_ON == (UcParMod & 0x08 )){		// If Convergence Judgement is Enabled
       for(i=0; i<CHTGOKN_WAIT; i++){
         a_ctrl->i2c_client.i2c_func_tbl->i2c_read(&a_ctrl->i2c_client,MSSET_211 ,&UcChtGst,MSM_CAMERA_I2C_BYTE_DATA);  
        LOGCL(" check closeloop if it converge\n");
         if(!(UcChtGst & 0x01))	break;
         msleep(1);	
       }
    }
    if(UcChtGst & 0x01 ){
       // STMV Success But Settling Time Over
       UcParMod |= 0x80 ;									
       //PIOA_ClearOutput(_PIO_PA29);											// Monitor I/O Port
   }else{
       //STMV Success 
       UcParMod &= 0x7F ;									
   }
   LOGCL("msm_LC898212_end LC898212 exit : 0x%x\n",UcParMod);
   // Bit0:0 Successful convergence Bit0:1 Time Over
   return UcParMod;															
}
//********************************************************************************
// Function Name 	: msm_LC898212_moveto
// Retun Value		: Stepmove Parameter
// Argment Value	: Stepmove Parameter, Target Position
// Explanation		: Stpmove Function
// History			: 
// History			: 
//********************************************************************************
uint16_t msm_LC898212_moveto(struct msm_actuator_ctrl_t *a_ctrl,uint16_t SsSmvEnd ){
	uint16_t UsSmvTim;
	uint16_t UsSmvDpl;//@cn
	uint16_t SsParStt;	// StepMove Start Position
        uint16_t regPos;
	//PIOA_SetOutput(_PIO_PA29);													// Monitor I/O Port
    LOGCL("[FUN] %s() +++++\n", __func__);
	LOGCL("moveto LC898212 enter : 0x%x\n",SsSmvEnd);
	// Get Start Position
	a_ctrl->i2c_client.i2c_func_tbl->i2c_read(&a_ctrl->i2c_client,RZ_211H ,&SsParStt,MSM_CAMERA_I2C_WORD_DATA);  //read Reg0x04
        LOGCL("Moveto, read from 0x04, data is: 0x%x", SsParStt);
	//a_ctrl->i2c_client.i2c_func_tbl->i2c_read(&a_ctrl->i2c_client,MS1Z22_211H ,&SsParStt,MSM_CAMERA_I2C_WORD_DATA);//read Reg0x18
        //LOGCL("Moveto, read from 0x18, data is: 0x%x", SsParStt);
	UsSmvDpl = abs_closeloop( SsParStt - SsSmvEnd );
        LOGCL("  %s,UsSmvDpl = 0x%x,SsParStt = 0x%x, SsSmvEnd = 0x%x,{UsSmvSiz: 0x%x, UcSmvItv: 0x%x, UcSmvEnb: 0x%x}.\n",__func__,UsSmvDpl, SsParStt, SsSmvEnd, StSmvPar.UsSmvSiz, StSmvPar.UcSmvItv, StSmvPar.UcSmvEnb);
	if( ( UsSmvDpl <= StSmvPar.UsSmvSiz ) && (( StSmvPar.UcSmvEnb & STMSV_ON ) == STMSV_ON ) ) {
           LOGCL(" path 1.\n");
           if( StSmvPar.UcSmvEnb & STMCHTG_ON ){
	      a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client, MSSET_211 /* SWTCH_211 */, (INI_MSSET_211 | 0x01),MSM_CAMERA_I2C_BYTE_DATA);
           }
	   // Handling Single Step For ES1
	   a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client,MS1Z22_211H, SsSmvEnd,MSM_CAMERA_I2C_WORD_DATA);
	  // Combine StepMove Enable Bit & StepMove Mode Bit
	  StSmvPar.UcSmvEnb |= STMVEN_ON;										
	} else {
           LOGCL(" path 2.\n");
	   // Check StepMove Direction
           if( SsParStt < SsSmvEnd ){
	       a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client,MS1Z12_211H, StSmvPar.UsSmvSiz,MSM_CAMERA_I2C_WORD_DATA);
           } else if( SsParStt > SsSmvEnd ){
	      a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client,MS1Z12_211H, 0xFFFF-StSmvPar.UsSmvSiz, MSM_CAMERA_I2C_WORD_DATA);
           }
	   // Set StepMove Target Positon
	   a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client,STMVENDH_211, SsSmvEnd,MSM_CAMERA_I2C_WORD_DATA);
	   a_ctrl->i2c_client.i2c_func_tbl->i2c_read(&a_ctrl->i2c_client,STMVENDH_211, &regPos,MSM_CAMERA_I2C_WORD_DATA);
           LOGCL(" write Reg0xA1: 0x%x, then read:0x%x. diff:0x%x.\n", SsSmvEnd, regPos, abs_closeloop(SsSmvEnd-regPos));
	   a_ctrl->i2c_client.i2c_func_tbl->i2c_read(&a_ctrl->i2c_client,0x3C, &regPos,MSM_CAMERA_I2C_WORD_DATA);
           LOGCL(" then read Reg0x3C: 0x%x, diff:0x%x.\n", regPos, abs_closeloop(SsSmvEnd-regPos));

	   // Combine StepMove Enable Bit & StepMove Mode Bit
	   StSmvPar.UcSmvEnb |= STMVEN_ON;	
	   // Start StepMove
	   a_ctrl->i2c_client.i2c_func_tbl->i2c_write(&a_ctrl->i2c_client,STMVEN_211, StSmvPar.UcSmvEnb,MSM_CAMERA_I2C_BYTE_DATA);
        }
	// Stepmove Operation time
	UsSmvTim=(UsSmvDpl/STMV_SIZE)*((STMV_INTERVAL+1)*10000 / LC898211_fs);			
	msleep(UsSmvTim);
        LOGCL(" after write position and before check, sleep %dms.\n", UsSmvTim);
	LOGCL("moveto LC898212 exit\n");
        LOGCL(" before msm_LC898212_end, {UsSmvSiz: 0x%x, UcSmvItv: 0x%x, UcSmvEnb: 0x%x}.\n", StSmvPar.UsSmvSiz, StSmvPar.UcSmvItv, StSmvPar.UcSmvEnb);
	return msm_LC898212_end(a_ctrl, StSmvPar.UcSmvEnb);
}

void msm_LC898212_InitForQualcomm(struct msm_actuator_ctrl_t *a_ctrl){ //main_part1
	//extern int closeloop_HallOff;	 	// see msm_sensor.c
	//extern int closeloop_HallBiase;       // see msm_sensor.c
	// Initialize driver IC
    LOGCL("[FUN] %s() +++++\n", __func__);
	msm_LC898212_init(a_ctrl, closeloop_HallOff, closeloop_HallBiase );	
	LOGCL(" closeloop_HallOff: 0x%x, closeloop_HallBiase:0x%x\n.", closeloop_HallOff, closeloop_HallBiase);
	// Step move parameter set
	StSmvPar.UsSmvSiz   = STMV_SIZE ;
	StSmvPar.UcSmvItv   = STMV_INTERVAL ;
	StSmvPar.UcSmvEnb  = STMCHTG_SET | STMSV_SET | STMLFF_SET ;
	msm_LC898212_set(a_ctrl, StSmvPar) ;
	// Close loop ON
	msm_LC898212_on(a_ctrl);	
}

