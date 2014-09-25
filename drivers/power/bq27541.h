/****************************************************************************/
/*                                                                       	*/
/* Copyright (c) 2010-2020  YULONG Company             ¡¡¡¡¡¡¡¡¡¡¡¡¡¡     	*/
/*                 ÓîÁú¼ÆËã»úÍ¨ÐÅ¿Æ¼¼£¨ÉîÛÚ£©ÓÐÏÞ¹«Ë¾  °æÈ¨ËùÓÐ 2010-2020  	*/
/*                                                                          */
/* PROPRIETARY RIGHTS of YULONG Company are involved in the  ¡¡¡¡¡¡¡¡¡¡¡¡	*/
/* subject matter of this material.  All manufacturing, reproduction, use, 	*/
/* and sales rights pertaining to this subject matter are governed by the 	*/
/* license agreement.  The recipient of this software implicitly accepts 	*/ 
/* the terms of the license.                                             	*/
/* ±¾Èí¼þÎÄµµ×ÊÁÏÊÇÓîÁú¹«Ë¾µÄ×Ê²ú,ÈÎºÎÈËÊ¿ÔÄ¶ÁºÍÊ¹ÓÃ±¾×ÊÁÏ±ØÐë»ñµÃ   		*/
/* ÏàÓ¦µÄÊéÃæÊÚÈ¨,³Ðµ£±£ÃÜÔðÈÎºÍ½ÓÊÜÏàÓ¦µÄ•¨ÂÉÔ¼Êø.                      	*/
/*                                                                       	*/
/****************************************************************************/
 
/**************************************************************************
**  Copyright (C), 2012, Yulong Tech. Co., Ltd.
**  FileName:          tps80032.h
**  Author:            zhangzhe
**  Version :          1.00
**  Date:              2012-08-10
**  Description:       head file for BQ27541 fuel gauge           
**  History:         
**  <author>	  <date>	<version> 	<desc>
**  zhangzhe	2010-08-10	  1.00 		create                                       
**************************************************************************/


#ifndef ZZ_BQ27541_H
#define ZZ_BQ27541_H

#include <linux/power_supply.h>

/* Bq27541 standard data commands */
#define BQ27541_REG_CNTL	0x00
#define BQ27541_REG_AR		0x02
#define BQ27541_REG_ARTTE	0x04
#define BQ27541_REG_TEMP	0x06
#define BQ27541_REG_VOLT	0x08
#define BQ27541_REG_FLAGS	0x0A
#define BQ27541_REG_NAC		0x0C
#define BQ27541_REG_FAC		0x0e
#define BQ27541_REG_RM		0x10
#define BQ27541_REG_FCC		0x12
#define BQ27541_REG_AI		0x14
#define BQ27541_REG_TTE		0x16
#define BQ27541_REG_TTF		0x18
#define BQ27541_REG_SI		0x1a
#define BQ27541_REG_STTE	0x1c
#define BQ27541_REG_MLI		0x1e
#define BQ27541_REG_MLTTE	0x20
#define BQ27541_REG_AE		0x22
#define BQ27541_REG_AP		0x24
#define BQ27541_REG_TTECP	0x26
#define BQ27541_REG_SOH		0x28
#define BQ27541_REG_SOC		0x2c
#define BQ27541_REG_NIC		0x2e
#define BQ27541_REG_ICR		0x30
#define BQ27541_REG_LOGIDX	0x32
#define BQ27541_REG_LOGBUF	0x34

#define BQ27541_FLAG_DSG	BIT(0)
#define BQ27541_FLAG_CHG	BIT(8)
#define BQ27541_FLAG_FC		BIT(9)
#define BQ27541_FLAG_XCHG	BIT(10)
#define BQ27541_FLAG_INH	BIT(11)

#define BQ27541_CS_DLOGEN	BIT(15)
#define BQ27541_CS_SS		BIT(13)

/* Control subcommands */
#define BQ27541_SUBCMD_CTNL_STATUS  	0x0000
#define BQ27541_SUBCMD_DEVCIE_TYPE  	0x0001
#define BQ27541_SUBCMD_FW_VER  		0x0002
#define BQ27541_SUBCMD_HW_VER  		0x0003
#define BQ27541_SUBCMD_DF_CSUM  	0x0004
#define BQ27541_SUBCMD_PREV_MACW   	0x0007
#define BQ27541_SUBCMD_CHEM_ID   	0x0008
#define BQ27541_SUBCMD_BD_OFFSET   	0x0009
#define BQ27541_SUBCMD_INT_OFFSET  	0x000a
#define BQ27541_SUBCMD_CC_VER   	0x000b
#define BQ27541_SUBCMD_OCV  		0x000c
#define BQ27541_SUBCMD_BAT_INS   	0x000d
#define BQ27541_SUBCMD_BAT_REM   	0x000e
#define BQ27541_SUBCMD_SET_HIB   	0x0011
#define BQ27541_SUBCMD_CLR_HIB   	0x0012
#define BQ27541_SUBCMD_SET_SLP   	0x0013
#define BQ27541_SUBCMD_CLR_SLP  	0x0014
#define BQ27541_SUBCMD_FCT_RES  	0x0015
#define BQ27541_SUBCMD_ENABLE_DLOG  	0x0018
#define BQ27541_SUBCMD_DISABLE_DLOG 	0x0019
#define BQ27541_SUBCMD_SEALED   	0x0020
#define BQ27541_SUBCMD_ENABLE_IT  0x0021
#define BQ27541_SUBCMD_DISABLE_IT 0x0023
#define BQ27541_SUBCMD_CAL_MODE  	0x0040
#define BQ27541_SUBCMD_RESET   	0x0041
#define ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN   	(-2731)
#define BQ27541_INIT_DELAY   		((HZ)*1)
#define BQ27541_CHECK_BUS_STATES_DELAY   		((HZ)*60)

#endif
