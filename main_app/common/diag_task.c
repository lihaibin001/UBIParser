/*----------------------------------------------------------------------------/
 *  (C)Dedao, 2016
 *-----------------------------------------------------------------------------/
 *
 * Copyright (C) 2016, Dedao, all right reserved.
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this condition and the following disclaimer.
 *
 * This software is provided by the copyright holder and contributors "AS IS"
 * and any warranties related to this software are DISCLAIMED.
 * The copyright owner or contributors be NOT LIABLE for any damages caused
 * by use of this software.
 *----------------------------------------------------------------------------*/

/**********************************************************************
   Title                      : Diag_Task.C

   Module Description         : 

   Author                     : 

   Created                    : 2016-08-29

 **********************************************************************/

/*********************************************************************/
/* Include header files                                              */
/*********************************************************************/
/*********************************************************************/
#include "standard.h"
#include "gps.h"

#define USE_DEBUG
#include "Debug.h"

/*********************************************************************/
/* File level pragmas                                                */
/*********************************************************************/

/*********************************************************************/
/* Constant and Macro Definitions using #define                      */
/*********************************************************************/
/*USER DEFINITION*/

#define  diag_current_state()       (diag_current_state)

#define AD_EXT_BATTERY (13)
#define AD_INT_BATTERY (1)


#define EXT_LOW_VOLTAGE_ALARM_THRESHOLD (1150)
#define INT_LOW_VOLTAGE_ALARM_THRESHOLD (350)

/*********************************************************************/
/* Enumerations and Structures and Typedefs                          */
/*********************************************************************/


/*********************************************************************/
/* Function Prototypes for Private Functions with File Level Scope   */
/*********************************************************************/

/*event handler*/
static void diag_evt_nop(int16_t data);


/*********************************************************************/
/* Static Variables and Const Variables With File Level Scope        */
/*********************************************************************/

// Definition of the event handler function pointer array.
static const void_int16_fptr event_handler[DIAG_NUM_EVENTS] = 
{
	diag_evt_nop,			   		   
};

/*********************************************************************/
/* Global and Const Variable Defining Definitions / Initializations  */
/*********************************************************************/
static Self_Diag_T diag_result;

/*********************************************************************/
/* ROM Const Variables With File Level Scope                         */
/*********************************************************************/

/*********************************************************************/
/* Add User defined functions                                        */
/*********************************************************************/
// Test once
static void diag_test_flash(uint32_t address);

// Test repeatly
static void diag_self_test(void);
static void diag_test_voltage(void);

static void diag_test_gps(void);
static void diag_test_gprs(void);

/**********************************************************************
 *    Function: Diag_Task
 *  Parameters: None
 *     Returns: None
 * Description: Main routine called by the operating system
 *********************************************************************/
void Diag_Task(void *pvParameters)
{
    Data_Message_T msg;          // Holds message received from the system
    uint32_t csq_time = 0;
    csq_time = OS_Time();

#ifdef USE_DEBUG
    DEBUG_PRINT0( DEBUG_MEDIUM, "[DIAG]:DIAG TASK Started!\r\n");
#endif
    
#if 0
    diag_test_flash(0);
#endif
    while(PS_Running())
    {
        if(E_OK == OS_Wait_Message(OS_DIAG_TASK,&msg.all,100))
        {
            if((msg.parts.msg > 0) && ((msg.parts.msg) < DIAG_NUM_EVENTS))
            {
                (*event_handler[msg.parts.msg])(msg.parts.data);       // Run event handler routine
            }
        }
        if ((csq_time + MSec_To_Ticks(3000)) < OS_Time())
        {
            diag_self_test();
            csq_time = OS_Time();
        }
    }

    OS_Terminate_Task();
}


/**********************************************************************
 * Description: Do nothing event handler
 * Parameters: message data
 *     Returns: None
 *********************************************************************/
static void diag_evt_nop(int16_t data)
{
	
}

static void diag_test_flash(uint32_t address)
{
    uint8_t flash_test_result=1;
    uint32_t test_flash_address=FLASH_RESERVED_OFFSET;
    uint16_t test_flash_len=20;
    uint8_t test_flash_data[]={0x01,0x02,0x03,0x04,0x05,
                             0x06,0x07,0x08,0x09,0x0A,
                             0x51,0x52,0x53,0x54,0x55,
                             0x56,0x57,0x58,0x59,0x5A};
    uint8_t test_flash_read[20];
    uint8_t i=0;

    sFLASH_Init();
    if (sFLASH_GD25Q16C_ID!=sFLASH_ReadID())
    {
        DEBUG_PRINT0(DEBUG_MEDIUM,"[DIAG]Flash Wrong ID\n\r");
        DEBUG_PRINT0(DEBUG_MEDIUM,"[DIAG]Flash test fail\n\r");
    }
#if 0
    sFLASH_EraseSector(test_flash_address);
    sFLASH_WriteBuffer(test_flash_data,test_flash_address,test_flash_len);
    sFLASH_ReadBuffer(test_flash_read,test_flash_address,test_flash_len);
    for(i=0;i<test_flash_len;i++)
    {
        if (test_flash_read[i]!=test_flash_data[i])
        {
            DEBUG_PRINT2(DEBUG_MEDIUM,"[DIAG] Flash write fail! [%x,%x]\n\r",test_flash_data[i],test_flash_read[i]);
            flash_test_result=0;
        }
    }
#endif
    if (flash_test_result)
    {
        diag_result.flash_ok=1;
        DEBUG_PRINT0(DEBUG_MEDIUM,"[DIAG]Flash test OK\n\r");
    }
    else
    {
        diag_result.flash_ok=0;
        DEBUG_PRINT0(DEBUG_MEDIUM,"[DIAG]Flash test fail\n\r");
    }
}

static void diag_test_voltage(void)
{
    uint16_t vol=Pwr_Fail_Get_Voltage();
    diag_result.ext_voltage=vol;
    DEBUG_PRINT1(DEBUG_MEDIUM,"[DIAG]Ext Battery=%d\n\r",vol);
    if (vol < EXT_LOW_VOLTAGE_ALARM_THRESHOLD)
    {
        OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_BATTERY_FAULT, 1));
    }
#if 1
    vol=Pwr_Fail_Get_Int_Voltage();
    diag_result.int_voltage=vol;
    DEBUG_PRINT1(DEBUG_MEDIUM,"[DIAG]Int Battery=%d\n\r",vol);
    if (vol < INT_LOW_VOLTAGE_ALARM_THRESHOLD)
    {
      /* message type need to change for inner battery? */  
      OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_BATTERY_FAULT, 1));
    }
#endif
}

static void diag_test_temp(void)
{
}

static void diag_test_gps(void)
{
    if (vGps_Get_Gps_Status())
    {
        diag_result.gps_fixed=1;
        DEBUG_PRINT0(DEBUG_MEDIUM,"[DIAG]GPS fixed\n\r");
    }
    else
    {
        diag_result.gps_fixed=0;
        DEBUG_PRINT0(DEBUG_MEDIUM,"[DIAG]GPS NOT fixed\n\r");
    }
}

static void diag_test_gprs(void)
{
    if (0!=GPRS_server_connected())
    {
        diag_result.network_connected=1;
        DEBUG_PRINT0(DEBUG_MEDIUM,"[DIAG]Server connected\n\r");
    }
    else
    {
        diag_result.network_connected=0;
        DEBUG_PRINT0(DEBUG_MEDIUM,"[DIAG]Server NOT connected\n\r");
    }
}

static void diag_test_acc(void)
{
    if (0!=GPRS_server_connected())
    {
        diag_result.network_connected=1;
        DEBUG_PRINT0(DEBUG_MEDIUM,"[DIAG]Server connected\n\r");
    }
    else
    {
        diag_result.network_connected=0;
        DEBUG_PRINT0(DEBUG_MEDIUM,"[DIAG]Server NOT connected\n\r");
    }
}

static void diag_self_test(void)
{
    diag_test_voltage();
    diag_test_gps();
    diag_test_gprs();
}

void diag_get_result(Self_Diag_T *result)
{
    memcpy(result, &diag_result, sizeof(Self_Diag_T));
}

/********************************************************************** 
 *                                                             
 * REVISION RECORDS                                            
 *                                                             
*********************************************************************/
/* $HISTROY$

 *********************************************************************/
