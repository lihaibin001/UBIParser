/* $Header:   ACC_Detect.h $*/
/*----------------------------------------------------------------------------/
 *  (C)Dedao, 2017
 *-----------------------------------------------------------------------------/
 *
 * Copyright (C) 2017, Dedao, all right reserved.
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this condition and the following disclaimer.
 *
 * This software is provided by the copyright holder and contributors "AS IS"
 * and any warranties related to this software are DISCLAIMED.
 * The copyright owner or contributors be NOT LIABLE for any damages caused
 * by use of this software.
 *----------------------------------------------------------------------------*/
#ifndef __ACC_DETECT_H__
#define __ACC_DETECT_H__

/*******************************************************************************
 * include
 ******************************************************************************/
#include "compiler.h"
//#include "rtc.h"
//#include "fsl_gpio.h"
//#include "fsl_port.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ACC_GPIO        GPIOC
//#define ACC_PORT_CLK    kCLOCK_PortC
#define ACC_GPIO_PIN    GPIO_Pin_2

/*******************************************************************************
 * Declaration
 ******************************************************************************/
//void ACC_InitializeMonitor(void);
void ACC_Signal_Monitor(void);
//void ACC_SetACCStatus(bool status);
//bool ACC_GetStatus(void);
bool ACC_Is_Signal_On_After_Delay(void);
bool ACC_Is_Signal_Off_After_Delay(void);
bool ACC_Signal_GetCurrentSignalLevel(void);

void ACC_Voltage_Change_Monitor(void);
void ACC_initialize(void);

bool ACC_Set_Source(void);
bool ACC_Is_Volt_Checking(void);

#endif //__ACC_DETECT_H__