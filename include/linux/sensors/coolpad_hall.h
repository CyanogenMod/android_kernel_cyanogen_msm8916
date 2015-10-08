/********************************************************************************/
/*																				*/
/* Copyright (c) 2000-2010  YULONG Company             　　　　　　　       	*/
/*         宇龙计算机通信科技（深圳）有限公司  版权所有 2000-2010               */
/*																				*/
/* PROPRIETARY RIGHTS of YULONG Company are involved in the           			*/
/* subject matter of this material.  All manufacturing, reproduction, use,      */
/* and sales rights pertaining to this subject matter are governed by the     	*/
/* license agreement.  The recipient of this sofPARAMSare implicitly accepts    */ 
/* the terms of the license.                                                    */
/* 本软件文档资料是宇龙公司的资产,任何人士阅读和使用本资料必须获得     			*/
/* 相应的书面授权,承担保密责任和接受相应的法律约束.                             */
/*																				*/
/********************************************************************************/

/**************************************************************************
**  Copyright (C), 2000-2010, Yulong Tech. Co., Ltd.
**  FileName:         Coolpad_hall.h
**  Author:            shuaixinzhong
**  Version :          1.00
**  Date:              2010-12-20
**  Description:       hall driver
**                      
**  History:         
**  <author>		<time>		<version >      <desc>
**  shuaixinzhong	2010-12-20	1.00		创建
**  changxuejian        2010-12-21	2.00		modify
**  yangjiayao		2014-02-13	3.00		modified for double hall sensor                     
**************************************************************************/

struct hall_node {
	const char *name;
	int gpio;
	int irq;
	bool wakeup;
};

struct hall_platform_data {
	const char *name;
	
	struct hall_node *hall;
	int hall_num;
};
