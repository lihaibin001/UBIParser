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
   Title                    : PERIODIC.C

   Module Description       : This is the standard code file for PERIODIC.

   Author                   : 

   Created                  : 

   Configuration ID         : 

 *********************************************************************/

/**********************************************************************
 * Installation Instructions (periodic tasks, etc.)
 *
 *********************************************************************/


/**********************************************************************
 * Include header files
 *********************************************************************/
#include "stm32f10x_gpio.h"
#include "standard.h"
#include "periodic.h"
#include "gps.h"
#include "ATProtocol.h"
#include "TelmApp.h"

#define USE_DEBUG
#include "Debug.h"

/**********************************************************************
 * File level pragmas
 *********************************************************************/

/**********************************************************************
 * Constant and Macro Definitions using #define
 *********************************************************************/
#define  OS_PERIODIC_TICKS   5
#define  Periodic_Stay_Awake()   true
/**********************************************************************
 * Enumerations and Structures and Typedefs
 *********************************************************************/

/**********************************************************************
 * Global and Const Variable Defining Definitions / Initializations
 *********************************************************************/

/**********************************************************************
 * Static Variables and Const Variables With File Level Scope
 *********************************************************************/
static uint8_t pending_ticks;
static uint32_t eng_on_low_volt_cnt = 0;
static uint32_t gps_first_fixed_cnt=0;
static uint32_t gps_first_fixed_sec=0;
static uint8_t gps_first_fixed_flag=1;
/**********************************************************************
 * ROM Const Variables With File Level Scope
 *********************************************************************/



/**********************************************************************
 * Function Prototypes for Private Functions with File Level Scope
 *********************************************************************/

static void periodic_initialization(void);   // Initialize periodic tasks
static void periodic_1x(void);               /* Called every  4      /   5 ms        */
static void periodic_2x(void);               /* Called every  8      /  10 ms        */
static void periodic_4x(void);               /* Called every  16     /  20 ms        */
static void periodic_8x(void);               /* Called every  32     /  40 ms        */
static void periodic_16x(void);              /* Called every  64     /  80 ms        */
static void periodic_32x(void);              /* Called every  128    / 160 ms        */
static void periodic_64x(void);              /* Called every  256    / 320 ms        */
static void periodic_128x(void);             /* Called every  512    / 640 ms        */
static void periodic_256x(void);             /* Called every  1.024  /  1.28 Second  */
static void periodic_512x(void);             /* Called every  2.049  /  2.56 Seconds */
static void periodic_1024x(void);            /* Called every  4.096  /  5.12 Seconds */
static void periodic_2048x(void);            /* Called every  8.192  / 10.24 Seconds */
static void periodic_4096x(void);            /* Called every 16.384  / 20.48 Seconds */
static void periodic_8192x(void);            /* Called every 32.768  / 40.96 Seconds */

static void Periodic_Set_GPS_Fixed(uint32_t count);

/**********************************************************************
 * Function Definitions
 *********************************************************************/

/**********************************************************************
 *   Function: Task_Periodic
 *
 *  Parameters: None
 *
 *    Returns: None
 *
 * Description:  This tasks runs periodically (OS_Periodic_Ticks * RTI period).
 *               It then runs slower loops at 2^n multiples of the base rate. 
 *
 *********************************************************************/
void Periodic_Task(void *pvParameters)
{
    uint_fast16_t slice = 0;
    uint_fast16_t mask;
    #ifdef USE_DEBUG
    DEBUG_PRINT0( DEBUG_MEDIUM, "[Periodic]:Periodic TASK Started!\r\n");
    #endif
    periodic_initialization();                   // initialize all periodic tasks
 
    ACC_initialize();
    
    while (E_OK == OS_Get_Resource(RES_PERIODIC)) {};
                                                // eat all pending ticks at start up
    while (PS_Running() || Periodic_Stay_Awake())      // run the exec as long as system is 'awake'
    {
        OS_Wait_Resource(RES_PERIODIC, 50);
                                                // wait for next periodic tick
        periodic_1x();                            // call base rate task
        slice++;                                  // increment time slice counter
        if (0 != slice)                           // don't search for bit if slice == 0
        {
            mask = 1;
            while (0 == (slice & mask))            // lowest set bit in slice
            {                                      // bit number indicates current time period
                mask <<= 1;
            }

            switch (mask)
            {
                case   0x0001:                   
                    periodic_2x();
                    break;
                case   0x0002:                   
                    periodic_4x();
                    break;
                case   0x0004:                   
                    periodic_8x();
                    break;
                case   0x0008:                   
                    periodic_16x();
                    break;
                case   0x0010:                   
                    periodic_32x();
                    break;
                case   0x0020:                   
                    periodic_64x();
                    break;
                case   0x0040:                   
                    periodic_128x();
                    break;
                case   0x0080:                   
                    periodic_256x();
                    break;
                case   0x0100:                   
                    periodic_512x();
                    break;
                case   0x0200:                   
                    periodic_1024x();
                    break;
                case   0x0400:                   
                    periodic_2048x();
                    break;
                case   0x0800:                   
                    periodic_4096x();
                    break;
                case   0x1000:                   
                    periodic_8192x();
                    break;
                default : ;                         // unused time slice
            }
        }
    }
    OS_Terminate_Task();                         // End task
}

/**********************************************************************
 *   Function: Periodic_Tick
 *  Parameters: None
 *    Returns: None
 * Description: This routine is call from the periodic RTI interrupt.
 *              It schedules the periodic task to run every OS_PERIODIC_TICKS
 *********************************************************************/
void Periodic_Tick(void)
{
    pending_ticks++;
    if (pending_ticks >= OS_PERIODIC_TICKS)
    {  
        pending_ticks = 0;
        OS_Release_Resource_From_ISR(RES_PERIODIC, false);           // trigger periodic task
    }
}

/***********************************************************************
 * Description: Initializes periodic tasks
 *  Parameters: None
 *     Returns: None
 ***********************************************************************/
void periodic_initialization(void)   // Initialize periodic tasks
{
	LED_Set_Toggle_Period(0, LED_OFF);
	LED_Set_Toggle_Period(1, LED_OFF);
}

/*************************************************************************
 * Description: User configurable routine to execute every periodic cycle
 *              NOTE: MUST NOT BLOCK (i.e. do not execute any wait or
 *                   schedule a routine).
 *  Parameters: None
 *     Returns: None
 *************************************************************************/
void periodic_1x(void)               /* Called every  4      /   5 ms        */
{
    Pwr_Fail_Monitor();
    TMR_Check_Timers();
}

/***********************************************************************
 * Description: User configurable routine to execute every 7.8125 mS
 *               NOTE: Must not Block (i.e. execute any wait or schedule routine)
 *  Parameters: None
 *     Returns: None
 ***********************************************************************/
void periodic_2x(void)               /* Called every  8      /  10 ms        */
{

}

/***********************************************************************
 * Description: User configurable routine to execute every 7.8125 mS
 *               NOTE: Must not Block (i.e. execute any wait or schedule routine)
 *  Parameters: None
 *     Returns: None
 ***********************************************************************/
void periodic_4x(void)               /* Called every  16     /  20 ms        */
{
  ACC_Voltage_Change_Monitor();
  
//  
//  static uint16_t     batt_volt_max = 0;
//    static uint16_t     batt_volt_min = 0;
//    static uint32_t     period_slice = 0;
//    static uint8_t      event_debounce=2;
//    static uint16_t     batt_volt = 0;
//    static uint16_t     batt_volt_latch = 0;
//    static uint16_t     batt_volt_last = 0;
//    
//    batt_volt = Pwr_Fail_Get_Voltage();
//#if 0
//    DEBUG_PRINT1 (DEBUG_MEDIUM,"\r\n[SYSTEM]:Batt=%d V \r\n", batt_volt);
//#endif
//    
//    if (0 == batt_volt_last)
//    {
//      batt_volt_last = batt_volt ;
//    }
//    else if(((batt_volt_last - batt_volt) > 100) && (batt_volt_latch ==0))
//    {
//      batt_volt_latch = batt_volt_last ;
//
//    }
//    else
//    {
//      
//    }
// 
//          period_slice ++;
//    
//    batt_volt_last = batt_volt ;
//    
////    if(batt_volt > batt_volt_max)
////    {
////        batt_volt_max = batt_volt;
////    }
////    else if((batt_volt < batt_volt_min) ||(batt_volt_min == 0))
////    {
////        batt_volt_min = batt_volt;
////    }
//    // lower than 2.45V
////    if (batt_volt<245)
////    {
////        // switch to long time wakeup mode;
////    }
//
//#if 0   //
//    if(period_slice%150 == 0)//3s
//    {
////        /*check if battery voltage > 13.25V, indicate that engine is running*/
////        if(((batt_volt_max + batt_volt_min)>>1) >= 1325)
////        {
////            if(event_debounce > 0)
////                event_debounce --;
////
////            if(event_debounce == 0)
////            {
////                event_debounce = 2;
////            }
////        }
////        else
////        {
////            if(((batt_volt_max + batt_volt_min)>>1) < 10)
////            {
////                eng_on_low_volt_cnt ++;
////            }
////			else
////			{
////				eng_on_low_volt_cnt  = 0;
////			}
////			event_debounce = 2;
////		}
//
//		/*check if battery voltage delta>1V, indicate that engine is starting up*/
//		if((batt_volt_max - batt_volt_min) >=150)
//		{
//          rl_set_acc_status(ON);    //ywf
//		}
//        
//        batt_volt_max = batt_volt;
//        batt_volt_min = batt_volt;
//	}
//#endif
// 	period_slice ++;
}

/***********************************************************************
 * Description: User configurable routine to execute every 7.8125 mS
 *               NOTE: Must not Block (i.e. execute any wait or schedule routine)
 *  Parameters: None
 *     Returns: None
 ***********************************************************************/
void periodic_8x(void)               /* Called every  32     /  40 ms        */
{

}

/***********************************************************************
 * Description: User configurable routine to execute every 7.8125 mS
 *               NOTE: Must not Block (i.e. execute any wait or schedule routine)
 *  Parameters: None
 *     Returns: None
 ***********************************************************************/
void periodic_16x(void)              /* Called every  64     /  80 ms        */
{

}

/***********************************************************************
 * Description: User configurable routine to execute every 7.8125 mS
 *               NOTE: Must not Block (i.e. execute any wait or schedule routine)
 *  Parameters: None
 *     Returns: None
 ***********************************************************************/
void periodic_32x(void)              /* Called every  128    / 160 ms        */
{
	static uint32_t led_period_slice = 0;
	LED_Toggle_Period_T led_on_period1 = LED_OFF;
	LED_Toggle_Period_T led_on_period2 = LED_OFF;
	uint8_t led_off_period1 = 0x04;
	uint8_t led_off_period2 = 0x04;

    if(PS_Running())
    {
        uint8_t gps_gsm_status=0;
		if(false == vGps_Get_Gps_Status())
		{
			//GPS position is not available
			gps_gsm_status &= ~(0x01);
		}
        else
        {
            gps_gsm_status |= 0x01;
        }
        
        
		if(vATProt_Get_TCP_State() < NET_DATA_TRANS)
		{
			//2G is not connected
			gps_gsm_status &= ~(0x02);
		}
        else
        {
            gps_gsm_status |= 0x02;
        }
            
#if 0
        if (0==device_unmount_status())
        {
            gps_gsm_status = 0x3;
        }
        else
        {
            gps_gsm_status = 0x0;
        }
#endif
        switch(gps_gsm_status)
        {
            case 0:
                LED_Set_Toggle_Period(0,LED_OFF);
                LED_Set_Toggle_Period(1,LED_TOGGLE_FAST_BLINK_1);
                break;
            case 1:
                LED_Set_Toggle_Period(0,LED_TOGGLE_FAST_BLINK_1);
                LED_Set_Toggle_Period(1,LED_TOGGLE_FAST_BLINK_1);
                break;
            case 2:
                LED_Set_Toggle_Period(0,LED_OFF);
                LED_Set_Toggle_Period(1,LED_TOGGLE_SLOW_BLINK_1);
                break;
            case 3:
                LED_Set_Toggle_Period(0,LED_OFF);
                LED_Set_Toggle_Period(1,LED_ON);
                break;
            default:
                break;
        }
    }

//    if((Sys_Is_RTC_Wakeup())
//        ||(Sys_Is_RTC_Deep_Wakeup())
//        ||(!PS_Running()))
//    {
//        //system is not full function,entering sleep
//        LED_Set_Toggle_Period(0, LED_OFF);
//        LED_Set_Toggle_Period(1, LED_OFF);
//    }

	led_on_period1 = LED_Get_Toggle_Period(0);
	led_on_period2 = LED_Get_Toggle_Period(1);

    if ((LED_OFF==led_on_period1) && (LED_OFF==led_on_period2))
    {
        IO_MCU_LED1_CTL_OUT(Bit_SET);
        IO_MCU_LED2_CTL_OUT(Bit_SET);
    }
#if 0
    else if (LED_OFF==led_on_period2)
    {
        IO_MCU_LED1_CTL_OUT(Bit_RESET);
        IO_MCU_LED2_CTL_OUT(Bit_SET);
    }
#endif
    else if ((LED_OFF == led_on_period1) && (LED_ON == led_on_period2))
    {
        IO_MCU_LED1_CTL_OUT(Bit_SET);
        IO_MCU_LED2_CTL_OUT(Bit_RESET);
    }
    else
    {
        if(led_period_slice>(led_on_period1+led_on_period2+led_off_period1+led_off_period2))
        {
            led_period_slice=0;
        }
        else if ((led_period_slice>0)&& (led_period_slice<=led_on_period1))
        {
            IO_MCU_LED1_CTL_OUT(Bit_RESET);
            IO_MCU_LED2_CTL_OUT(Bit_SET);
        }
        else if ((led_period_slice>led_on_period1)&& (led_period_slice<=led_on_period1+led_off_period1))
        {
            IO_MCU_LED1_CTL_OUT(Bit_SET);
            IO_MCU_LED2_CTL_OUT(Bit_SET);
        }
        else if ((led_period_slice>led_on_period1+led_off_period1) && (led_period_slice<=led_on_period1+led_off_period1+led_on_period2))
        {
            IO_MCU_LED1_CTL_OUT(Bit_SET);
            if (led_on_period2>0)
            {
                IO_MCU_LED2_CTL_OUT(Bit_RESET);
            }
            else
            {
                IO_MCU_LED2_CTL_OUT(Bit_SET);
            }
        }
        else
        {
            IO_MCU_LED1_CTL_OUT(Bit_SET);
            IO_MCU_LED2_CTL_OUT(Bit_SET);
        }
    }

	led_period_slice ++;
}

/***********************************************************************
 * Description: User configurable routine to execute every 7.8125 mS
 *               NOTE: Must not Block (i.e. execute any wait or schedule routine)
 *  Parameters: None
 *     Returns: None
 ***********************************************************************/
void periodic_64x(void)              /* Called every  256    / 320 ms        */
{
    // First time gps fix message
    if (vGps_Get_Gps_Status())
    {
        gps_first_fixed_cnt++;
        Periodic_Set_GPS_Fixed(gps_first_fixed_cnt);
        if (gps_first_fixed_flag > 0)
        {
            OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_GPS_FIRST, 1));
            OS_Send_Message(OS_RECORD_TASK,Build_Message(RECORD_EVT_GPS_FIXED, 1));
        }
        gps_first_fixed_flag=0;
    }
    else
    {
        gps_first_fixed_cnt++;
    }
    
    Pwr_Charge_Monitor();
    ACC_Signal_Monitor();
    
}

/***********************************************************************
 * Description: User configurable routine to execute every 7.8125 mS
 *               NOTE: Must not Block (i.e. execute any wait or schedule routine)
 *  Parameters: None
 *     Returns: None
 ***********************************************************************/
void periodic_128x(void)             /* Called every  512    / 640 ms        */
{
}

/***********************************************************************
 * Description: User configurable routine to execute every 7.8125 mS
 *               NOTE: Must not Block (i.e. execute any wait or schedule routine)
 *  Parameters: None
 *     Returns: None
 ***********************************************************************/
void periodic_256x(void)             /* Called every  1.024  /  1.28 Second  */
{
}

/***********************************************************************
 * Description: User configurable routine to execute every 7.8125 mS
 *               NOTE: Must not Block (i.e. execute any wait or schedule routine)
 *  Parameters: None
 *     Returns: None
 ***********************************************************************/
void periodic_512x(void)             /* Called every  2.049  /  2.56 Seconds */
{
}

/***********************************************************************
 * Description: User configurable routine to execute every 7.8125 mS
 *               NOTE: Must not Block (i.e. execute any wait or schedule routine)
 *  Parameters: None
 *     Returns: None
 ***********************************************************************/
void periodic_1024x(void)            /* Called every  4.096  /  5.12 Seconds */
{

}

/***********************************************************************
 * Description: User configurable routine to execute every 7.8125 mS
 *               NOTE: Must not Block (i.e. execute any wait or schedule routine)
 *  Parameters: None
 *     Returns: None
 ***********************************************************************/
void periodic_2048x(void)            /* Called every  8.192  / 10.24 Seconds */
{

}

/***********************************************************************
 * Description: User configurable routine to execute every 7.8125 mS
 *               NOTE: Must not Block (i.e. execute any wait or schedule routine)
 *  Parameters: None
 *     Returns: None
 ***********************************************************************/
void periodic_4096x(void)            /* Called every 16.384  / 20.48 Seconds */
{
}

/***********************************************************************
 * Description: User configurable routine to execute every 7.8125 mS
 *               NOTE: Must not Block (i.e. execute any wait or schedule routine)
 *  Parameters: None
 *     Returns: None
 ***********************************************************************/
void periodic_8192x(void)            /* Called every 32.768  / 40.96 Seconds */
{
}

/***********************************************************************
 * Description: 
 *  Parameters: None
 *     Returns: None
 ***********************************************************************/
uint32_t Periodic_Get_Low_Volt_Cnt(void)           
{
	return eng_on_low_volt_cnt;
}
/***********************************************************************
 * Description: 
 *  Parameters: None
 *     Returns: None
 ***********************************************************************/
void Periodic_Clear_Low_Volt_Cnt(void)           
{
	eng_on_low_volt_cnt = 0;
}

static void Periodic_Set_GPS_Fixed(uint32_t count)
{
    gps_first_fixed_sec=(count*32)/100;
}

uint32_t Periodic_Get_GPS_Fixed(void)
{
    return gps_first_fixed_sec;
}
/**********************************************************************
 *
 * REVISION RECORDS
 *
 *********************************************************************/
/*********************************************************************/
/* 
 * 
 *********************************************************************/
