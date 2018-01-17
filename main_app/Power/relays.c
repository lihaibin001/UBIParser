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
/* $Header:   relays.c  $*/
/**********************************************************************
   Title                    : relays.c

   Module Description       : This is the standard code file for relays.

   Author                   : 

   Created                  : 

   Configuration ID         : 

 *********************************************************************/

/*********************************************************************/
/* Include header files                                              */
/*********************************************************************/
/* Dependent "compile.cmd"                                           */
/*********************************************************************/
#include        "relays.h"
#include        "lowpower.h"
#include        "system.h"
#include        "delay.h"
//#include	"Gsensor.h"
#include	"GPRS.h"
#include	"TelmProtocol.h"
//#include "crc_ccitt.h"

#define USE_DEBUG
#include "Debug.h"

/*********************************************************************/
/* File level pragmas                                                */
/*********************************************************************/

/*********************************************************************/
/* Constant and Macro Definitions using #define                      */
/*********************************************************************/
//Fixme
//#define Pwr_Fail_Is_Reset_Condition() false


#define rl_ignition_to_awake rl_play_to_awake

#define rl_write_timer(x, y)      (rl_timer[x] = y)
#define rl_read_timer(x)          (rl_timer[x])
#define rl_timer_running(x)       (rl_timer[x] > OS_Time())

#define rl_set_current_state(state)    (rl_data = state)
#define rl_get_current_state()         (rl_data)

#define rl_remain_idle(x)                 ((x > 0) ? (OS_Time() <= x) : 1)

#define rl_wait_finish_awake_sequence_hook()             
#define rl_regulator_outputs_ok()	     true
#define rl_awake_to_idle_hook()
#define rl_idle_to_awake_hook()
#define rl_awake_to_play_hook()                                       
#define rl_wait_to_go_idle_hook()                                      
#define rl_other_task_cleanup_complete() TRUE

#define partial_terminating_tasks         all_terminating_tasks
#define ALL_TERMINATING_TASKS             (Num_Elems(all_terminating_tasks))    

#define RL_GPRS_START_DELAY (2000)

#define MIN_TO_TICKS    (60000)
#define MIN_TO_SEC    (60)
#define SEC_TO_TICKS    (1000)

// time in seconds
#define ENV_TEST_SLEEP_TIME (1200)

// time in ms
#define ENV_TEST_WAKE_TIME (60000)

/*********************************************************************/
/* Enumerations and Structures and Typedefs                          */
/*********************************************************************/

typedef enum rl_current_state_tag
{
   RL_CS_IDLE,
   RL_CS_AWAKE,
   RL_CS_PLAY,
   MAX_RL_CURRENT_STATES
} rl_current_state_t;

enum
{
   AWAKE_DELAY_TIMER,
   REG_OFF_DELAY_TIMER,
   WAIT_TO_STOP_TIMER,
   PF_DELAY_TIMER,
   NUM_RL_TIMERS
};

/*********************************************************************/
/* Function Prototypes for Private Functions with File Level Scope   */
/*********************************************************************/
//user define
static bool rl_wake_up(void);
static void rl_finish_awake_sequence_hook (void);
static void rl_configure_idle_hook (void);
static void rl_check (void);
static void rl_emergency_shutdown(void);
//building block
static void rl_idle_to_awake(void);
static void rl_play_to_awake(void);
static void rl_cs_awake(void);
static void rl_cs_play(void);
static void rl_awake_to_idle(void);
static void rl_cs_idle(void);
static void rl_wait_for_tasks_to_suspend(void);
static void rl_check_wake_up(void);
static void rl_low_voltage (void);
static void rl_awake_to_play(void);
static void rl_go_sleep(void);

static void RL_Start_GPRS(void);
static void RL_Start_GPS(void);
static void rl_rtc_config(void);
//static void rl_exti_config(void);
static uint8_t rl_get_low_batt_sleep(void);

/*===========================================================================*\
 * Global and Const Variable Defining Definitions / Initializations
\*===========================================================================*/
static uint16_t acc_on_abs_volt=0;
static uint16_t acc_off_abs_volt=0;
static uint8_t acc_status=0;

/*********************************************************************/
/* Static Variables and Const Variables With File Level Scope        */
/*********************************************************************/
static rl_current_state_t rl_data ;
static bool rl_wake_up_requested;

static Tick_Type rl_timer[NUM_RL_TIMERS];

static bool rl_is_amp_muted;

static bool rl_gps_pwr_up = false;
static bool rl_first_go_sleep=true;

static const void_fptr rl_current_state[MAX_RL_CURRENT_STATES] = 
{
   rl_cs_idle,
   rl_cs_awake,
//   rl_cs_play
};

/* add/subtract tasks which terminate/don't terminate here - */
/*static const Task_Type all_terminating_tasks[] =         
{
    OS_IOT_TASK,
};*/

__no_init uint32_t rtc_timeout;
__no_init bool can_interrupt_disable;
__no_init uint8_t low_batt_sleep;
/*********************************************************************/
/* Add User defined functions                                        */
/*********************************************************************/
/*********************************************************************/
/* Global Function Definitions                                       */
/*********************************************************************/
/**********************************************************************
*    Function: rl_not_awake_IO
*
*  Parameters: none
*
*     Returns: none
*
* Description: This routine handles any specific action in 
*              preparation for I/O idle configuration.
*
**********************************************************************/
static void rl_not_awake_IO(void)
{
   /* 1. set all port type to I/O */
   /* 2. set all port mode to input/output */    
//3:  Push-pull output mode, output speed is 50 MHz 
//4:  floating Input mode
	/* PA0--15  */   						
	GPIOA->CRL=0X44444444;	/*4444 4444 , all input*/
	GPIOA->CRH=0X44444444;	/*4444 1441 , all input, except PA8, PA11 */
	GPIOA->ODR=0X00000000;  /* all output pin input, except PA8, PA11 */

//	/* PB0--15  */
	GPIOB->CRL=0X44444444;	/*4444 4141 ,all input, except PB0,PB2 */
	GPIOB->CRH=0X44444444;	/*4444 4444 ,all input */
	GPIOB->ODR=0X00000000;  /* all output pin input: except PB0, PB2 low*/

//	/* PC0--15  */
	GPIOC->CRL=0X44444444;	/*4444 4111 ,all input, except PC0,PC1*/
	GPIOC->CRH=0X44444444;	/*4444 4444, all input */
	GPIOC->ODR=0X00000000; /* all output pin input: except PC0,PC1 low */

//	/* PD2  */
	GPIOD->CRL=0X44444444;     /* PD2 input mode */
	GPIOD->CRH=0X44444444;
	GPIOD->ODR=0X00000000;    /* PD2 input */


}

/**********************************************************************
*
*    Function: rl_configure_idle
*
*  Parameters: none
*
*     Returns: none
*
* Description: executes all configurations needed for idle mode
*
**********************************************************************/
static void rl_configure_idle_hook (void)
{

}

/**********************************************************************
*    Function: rl_configure_micro_for_idle
*
*  Parameters: none
*
*     Returns: none
*
* Description: Intended to perform necessary steps to put micro into
*              idle mode.
*
**********************************************************************/

static void rl_configure_micro_for_idle(void)
{
   	ADC_DeInit(ADC1);
    ADC_Cmd(ADC1,DISABLE);

	Micro_Go_Sleep();
}

/**********************************************************************
*    Function: rl_begin_awake_sequence_hook
*  Parameters: none
*     Returns: none
* Description: hook function for RL_Begin_Awake_Sequence()
**********************************************************************/
static void rl_finish_awake_sequence_hook (void)
{
//	#ifdef USE_DEBUG
//	uint8_t wakeup_source;
//	wakeup_source = Sys_Get_Wakeup_Src_Flags();
//	if(!Cold_Start())
//    		DEBUG_PRINT1(DEBUG_HIGH,"[SYSTEM]:Wake Up SRC:%x\n\r",wakeup_source);
//	#endif
//	rl_gps_pwr_up = false;//init
//	if(!Sys_Is_RTC_Deep_Wakeup())
	{
   	    rl_gps_pwr_up = true;

       	RL_Start_GPS();
       	{
            DEBUG_PRINT0(DEBUG_MEDIUM,"[RELAYS]Start GPRS 2\n\r");
            Sys_Clear_2G_Wakeup_Flag();
            TMR_Start_Timer(GPRS_START_TIMER, RL_GPRS_START_DELAY, RL_Start_GPRS);	
       	}
    }
}

static void rl_get_config(void)
{
    Config_t config_data;
    Get_config_data(&config_data);
    if (config_data.structData.ignition_on_absolute_voltage != 0)
    {
        acc_on_abs_volt=config_data.structData.ignition_on_absolute_voltage;
    }
    if (config_data.structData.ignition_off_absolute_voltage != 0)
    {
        acc_off_abs_volt=config_data.structData.ignition_off_absolute_voltage;
    }
}

/**********************************************************************
 *    Function: Relays_Task
 *
 *  Parameters: none
 *
 *     Returns: none
 *
 * Description: Entry point for the relays task.
 *
 *********************************************************************/

void Relays_Task(void *pvParameters)
{
    rl_current_state_t old_current_state;
    uint32_t wake_time=0;
    #ifdef USE_DEBUG
    DEBUG_PRINT0(DEBUG_MEDIUM,"[Relays]Relays task started!\r\n");
    #endif
    rl_idle_to_awake();

//    sFLASH_Init();
//    Load_Param();
//    rl_get_config();

    FOREVER
    {
        OS_Wait_Resource(RES_RELAYS, MAX_RL_WAIT_TIME);  /* wait for semaphore or timeout */

        /* do all other checks first */
        rl_check();

        do
        {
            if (MAX_RL_CURRENT_STATES > rl_get_current_state())
            {
                old_current_state = rl_get_current_state();
                (*rl_current_state[rl_get_current_state()])();
            }
            else
            {
                /* current state index is currupted, fix it! */
                old_current_state = MAX_RL_CURRENT_STATES;/* force state machine to run again */
            }
        } while (old_current_state != rl_get_current_state());
    }
}
/**********************************************************************
*
*    Function: RL_Set_Pwr_Fail_Detected
*
*  Parameters: status true/false
*
*     Returns: none
*
* Description: informs Relays that powerfail  voltage range is
*              entered or exited
*
**********************************************************************/
void RL_Set_Pwr_Fail_Detected(bool status)
{
    UNUSED_PARAM(status);

    /* trigger RELAYS task when powerfail status is updated */
    OS_Release_Resource(RES_RELAYS);
}

/**********************************************************************
 *    Function: rl_wake_up
 *  Parameters: none
 *     Returns: none
 * Description: Intended to contain any obd specific information which
 *              would cause the obd to wake up (such as the SOS/CAN Bus
 *              waking up).
 **********************************************************************/

static bool rl_wake_up(void)
{
    bool    wake_up = false;
    uint8_t wake_up_flags=0;

    /* always check wake up sources !!! */
    /* remember any wake up request immediatelly */
    wake_up_flags = Sys_Get_Wakeup_Src_Flags();

    /* Check ACC signal status, if on, restart the system */
    if (ACC_Signal_GetCurrentSignalLevel())
    {
      rl_wake_up_requested = true;
    }
    
    #if 0
    /* only if battery voltage is okay execute possible wake up sources !!! */
    if (Pwr_Fail_Is_Reset_Condition())  
    {
        /* This input will be asserted low whenever the low battery voltage warning 
        condition (BATT<= 6.0V ||(BATT>= 28 V) is detected.*/

        /* when powerfail timer started check if expired then obd must stay off */
        if (!rl_timer_running(WAIT_TO_STOP_TIMER))
        {
            if (wake_up_flags)
            {
                rl_wake_up_requested = true;
            }
            else
            {
                rl_wake_up_requested = false;
                rl_not_awake_IO();
                /* do NOT start REG_OFF_DELAY_TIMER here as this would prevent
                   the uC from going to STOP mode !!! [HPO, 25.May2009] */
            }
        }
        else
        {
            /* save wake up reason as long as WAIT_TO_STOP_TIMER not expired */
            if (wake_up_flags)
            {
                rl_wake_up_requested = true;
            }
        }
    }
    else
    #endif
    {
        /* but wake up earliest when REG_OFF timer is expired */
        if (!rl_timer_running(REG_OFF_DELAY_TIMER))
        {
            /* process any wake up request only after REG_OFF timer expired ! */
            wake_up = rl_wake_up_requested || (wake_up_flags > 0x00);
            if(wake_up)
            {
                rl_wake_up_requested = 0;
                wake_up_flags = 0;
            }
        }
    }
    return( wake_up );
}

static void RL_Start_GPRS(void)
{
    /**********2G start sequence*************/
    /*Enable  PA.11(4V_2G-CTRL), It is power supply of 2G module*/
    IO_4V_CTRL_OUT(Bit_SET);
 
    /*delay 100ms for power stable*/
    rl_delay_without_schedule(100);
    {
        OS_Activate_Task(OS_IOT_TASK); 	
    }
}

static void RL_Start_GPS(void)
{
    /**********GPS start sequence*************/
    /*Enable PB.00 MCU_3V3-GPS_EN*/
    IO_3V3_GPS_EN_OUT(Bit_SET);
    /*delay 25ms for power stable*/
    rl_delay_without_schedule(25);
    {
        OS_Activate_Task(OS_GPS_TASK); 
    }
}

/**********************************************************************
 *    Function: RL_Begin_Awake_Sequence
 *  Parameters: none
 *     Returns: none
 * Description: Performs initial steps to move obd to awake state - 
 *              namely, turns the regulator on
 *********************************************************************/

void RL_Begin_Awake_Sequence(void)
{

}

/**********************************************************************
 *    Function: RL_Finish_Awake_Sequence
 *
 *  Parameters: none
 *
 *     Returns: none
 *
 * Description: Performs any further steps to move obd to awake state -
 *              namely, wait for the regulator to power up
 *
 *********************************************************************/

void RL_Finish_Awake_Sequence(void)
{
    /* the AWAKE_DELAY_TIMER was loaded in RL_Begin_Awake_Sequence; */
    /* wait until it expires before proceeding but do not give up   */
    rl_write_timer(AWAKE_DELAY_TIMER, OS_Time() + REG_ON_TIME);

    while (rl_timer_running(AWAKE_DELAY_TIMER))
    {    
        /* do powerfail monitoring */
        if (Pwr_Fail_Is_Reset_Condition())
        {
           /* shut voltage off and go to idle */
            rl_emergency_shutdown();
        }
        else if(Pwr_Fail_AD_get_Voltage() < 1150)
        {
            if(1 == rl_get_low_batt_sleep())//recover from low battery force sleep
            {
                if(false == Sys_Get_Standby_Req_Flag())
                {
                    RL_Force_Sleep();//Not yet, so force sleep again right now
                }
            }
        }
        else
        {
            rl_set_low_batt_sleep(0);//battery recovery, go ahead
        }

        Enable_Interrupts();                      // in case WTNCS is corrupt
        Feed_Dog();                               // make sure we have full watchdog timeout
    }
}

/*********************************************************************/
/* Local Function Definitions                                        */
/*********************************************************************/
/**********************************************************************
*    Function: rl_check
*  Parameters: none
*     Returns: void
* Description: this is a hook up function from the RL_Task,
*              call here all other functions that need to be executed every
*              task run, if not needed it can be removed by using a empty macro
**********************************************************************/
static void rl_check (void)
{
    /*            */
  if (Pwr_Fail_Is_Reset_Condition())
  {
    /* power voltage is too high, reset the system */
    DEBUG_PRINT0(DEBUG_HIGH,"[RELAY]:Battery over voltage, restarting!!!\n\r");
    OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_BATTERY_FAULT, 2));
    
    rl_reset();
  }
  else if (Pwr_Fail_Is_Shutdown())
  {
    /* shut voltage off and go standby
    * only extenal voltage up can wake up */
    
    DEBUG_PRINT0(DEBUG_HIGH,"[RELAY]:Battery low, Force Sleeping in 15s!!!\n\r");
    OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_BATTERY_FAULT, 1));				
    
    OS_Sleep(15000);//15 seconds for sending battery low warning
    
    rl_set_low_batt_sleep(1);
    RL_Force_Sleep();   // go to standby, and wait external voltage regain
    
  }
    else 
    {
      /* do nothing */
    }
   
  
  /* check powerfail-reset: voltage range that allows reset */
    /* check restart request flag that allows reset           */
//    if (Pwr_Fail_Is_Reset_Condition())
//    {
//        /* shut voltage off and go to idle */
//        rl_emergency_shutdown();        /* record the state before obd off when  power  became bad */
//    }
//    else if(Periodic_Get_Low_Volt_Cnt() > 20)//low volt for 20s in any state
//    {
//        DEBUG_PRINT0(DEBUG_HIGH,"[SYSTEM]:Battery low, Force Sleep!!!\n\r");
//        OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_BATTERY_FAULT, 1));				
//
//        OS_Sleep(15000);//15 seconds for sending battery low warning
//
//        rl_set_low_batt_sleep(1);
//        RL_Force_Sleep();
//    }
//    else if(PS_Play_On())
//    {
//        /* check for good battery voltage */
//        if (rl_is_amp_muted)
//        {
//            /* check if battery voltage is good */
//            if (Pwr_Fail_Is_Voltage_Good())
//            {
//                rl_is_amp_muted = false;
//            }
//        }
//        else
//        {
//            /* check if battery voltage is not good */
//            if (!Pwr_Fail_Is_Voltage_Good())
//            {
//                rl_is_amp_muted = true;
//               //should inform that voltage is not good,tbd
//            }
//        }
//    }
}

/**********************************************************************
 *    Function: rl_idle_to_awake
 *
 *  Parameters: none
 *
 *     Returns: none
 *
 * Description: Performs action to move obd from idle to awake
 *
 *********************************************************************/

static void rl_idle_to_awake(void)
{
   rl_finish_awake_sequence_hook();
   rl_set_current_state(RL_CS_AWAKE);
}

/**********************************************************************
 *    Function: rl_play_to_awake
 *  Parameters: none
 *     Returns: none
 * Description: Performs action to move obd from play to awake
 *********************************************************************/

static void rl_play_to_awake(void)
{
  rl_set_current_state(RL_CS_AWAKE);
}

/**********************************************************************
 *    Function: rl_cs_awake
 *  Parameters: none
 *     Returns: none
 * Description: Performs regulator and bridge support for awake state.
 *********************************************************************/
static void rl_cs_awake(void)
{
//    uint16_t vol_ext=Pwr_Fail_Get_Voltage();
//
//    uint16_t vol_int=Pwr_Fail_Get_Int_Voltage();
  
    if (!PS_Awake())
    {
        if(!Sys_Get_Standby_Req_Flag())
        {
            Sys_Clear_Wakeup_Src_Flags();
        }
        else
        {
            Sys_Clear_Standby_Req_Flag();
        }
        
        rl_awake_to_idle();        
    }
//    else if (PS_Play_On())
//    {
//        rl_awake_to_play();
//    }
    else /* refresh rl_cs_awake */
    {
    }
//    
}

/**********************************************************************
*    Function: rl_awake_to_play
*  Parameters: none
*     Returns: none
* Description: Performs action to move obd from awake to play
*********************************************************************/
//static void rl_awake_to_play (void)
//{ 
//    if(false == rl_gps_pwr_up)
//    {
//        rl_gps_pwr_up = true;
//        RL_Start_GPS();
//        if(Cold_Start())
//       	{
//            DEBUG_PRINT0(DEBUG_MEDIUM,"[RELAYS]Start GPRS\n\r");
//            Sys_Clear_2G_Wakeup_Flag();
//            TMR_Start_Timer(GPRS_START_TIMER, RL_GPRS_START_DELAY, RL_Start_GPRS);
//        }
//    }
//    rl_set_current_state(RL_CS_PLAY);
//}

/**********************************************************************
 *    Function: rl_cs_play
 *  Parameters: none
 *     Returns: none
 * Description: Performs regulator and bridge support for play state.
 **********************************************************************/

//static void rl_cs_play(void)
//{
//    if (!PS_Play_On())
//    {
//        rl_play_to_awake();
//    }
//    else /* refresh rl_cs_play */
//    {
//    }	   
//}

/**********************************************************************
 *    Function: rl_awake_to_idle
 *  Parameters: none
 *     Returns: none
 * Description: Performs action to move obd from wake to idle
 *********************************************************************/
static void rl_awake_to_idle(void)
{
    Config_t config_data;
    uint32_t sleep_time=0;
    uint8_t activate_status=Get_Activation_Status();

    Get_config_data(&config_data);

    #ifdef USE_DEBUG
        DEBUG_PRINT0(DEBUG_HIGH,"[SYSTEM]:Sleeping...\n\r");
    #endif

    {
        // set 3G module to sleep mode
        GPRS_Module_GoSleep();

        OS_Sleep(1000);
    }
    // Save data to flash
    Save_Param();
    OS_Sleep(STBY_OFF_WAIT_TICKS);

    Sys_Clear_Wakeup_Src_Flags();
    Sys_Clear_Standby_Req_Flag();

    IO_3V3_GPS_EN_OUT(Bit_RESET);
    IO_4V_CTRL_OUT(Bit_RESET);
    IO_MCU_LED1_CTL_OUT(Bit_SET);
    IO_MCU_LED2_CTL_OUT(Bit_SET);
    IO_CHARGE_CTL(Bit_RESET);

    rl_awake_to_idle_hook();

    rl_wait_for_tasks_to_suspend();

    OS_Sleep(STBY_OFF_WAIT_TICKS);

    Sensors_crash_reset();

    MDG_Clear_Mode();

    OS_Clr_Start_Flag();  //

    rl_not_awake_IO();

    if (Sys_Get_Int_battery_Flag())
    {
      // Set sleep wakeup time, 24 hours
      rl_set_rtc_timeout(PS_RTC_DEEP_STANDBY_TICK);
      /* Configure RTC clock source and prescaler */
      rl_rtc_config();   
    }
    
//    rl_exti_config();

    rl_set_current_state(RL_CS_IDLE);
    //Micro_Go_Sleep();
}

/**********************************************************************
 *    Function: rl_cs_idle
 *  Parameters: none
 *     Returns: none
 * Description: Performs regulator support for idle state.
 *********************************************************************/
static void rl_cs_idle(void)
{
    Tick_Type idle_time = 0;
   
    while(rl_remain_idle(idle_time) 
          && (!Sys_Get_Wakeup_Src_Flags()))
    {
        /*executes all configurations needed for idle mode*/
        rl_check_wake_up();
        rl_configure_idle_hook();
        Feed_Dog();
        Enable_Interrupts();
        rl_configure_micro_for_idle();
    }
     
    /* After wake up from sleep mode, go to here and warm restart */
     rl_check_wake_up();
}

/**********************************************************************
*
*    Function: rl_delay
*
*  Parameters: none
*
*     Returns: none
*
* Description: indicate that power supply for awake AP is ready
*
*********************************************************************/
void rl_delay_without_schedule(Tick_Type ms)
{
    rl_write_timer(AWAKE_DELAY_TIMER, OS_Time() + ms);

    while (rl_timer_running(AWAKE_DELAY_TIMER))
    {
        Enable_Interrupts();                      // in case WTNCS is corrupt
        Feed_Dog();                               // make sure we have full watchdog timeout
    }
}

/**********************************************************************
*    Function: RL_In_Idle
*  Parameters: none
*     Returns: whether or not relays is in the idle state
* Description: Accessor to determine if relays is in the idle state
*
*********************************************************************/
bool RL_In_Idle (void)
{
    return(RL_CS_IDLE == rl_get_current_state());
}

/**********************************************************************
*    Function: RL_In_Play
*  Parameters: none
*     Returns: whether or not relays is in the play state
* Description: Accessor to determine if relays is in the play state
*
*********************************************************************/
//bool RL_In_Play (void)
//{
//    return(RL_CS_PLAY == rl_get_current_state());
//}

/**********************************************************************
 *    Function: rl_wait_for_tasks_to_suspend
 *  Parameters: none
 *     Returns: none
 * Description: Sends out GO_IDLE message and waits for appropriate tasks
 *              to receive message and suspend.  Has a timeout value and
 *              will return E_OK or E_TIMEOUT depending on whether all
 *              tasks suspended in the appropriate time or not.
 *
 **********************************************************************/
static void rl_wait_for_tasks_to_suspend(void)
{
    int     awake_to_idle_increments = 0;
    Status_Type suspend_error;

    do
    {
        rl_check_wake_up();

        /* We need to make sure all tasks which are supposed to be suspended are indeed   */
        /* suspended before proceeding to the idle loop.  We also need to make sure we do */
        /* not hold up going to idle for tasks which do not suspend (e.g., OS_IDLE_TASK,  */
        /* OS_RELAYS).  Only those tasks with a while (PS_Running()) loop will suspend.   */

        suspend_error = E_OK;  /* assume we are cleaned up until proven otherwise in below test */
        if (!rl_other_task_cleanup_complete())
        {
            suspend_error = E_TIMEOUT;
        }
        if (E_TIMEOUT == suspend_error)
        {
            if (!(awake_to_idle_increments % NUM_INCREMENTS_TO_RESEND_IDLE_MSG))
            {

            }
            OS_Sleep(AWAKE_TO_IDLE_INCREMENT_TICKS);
        }//50ms * 10 = 500ms timer for all_terminating_tasks(current is 3g task) to finish current cycle(OTA packet decoding? dataflash writing? ).
    
    } while ((E_TIMEOUT == suspend_error) && (++awake_to_idle_increments < NUM_INCREMENTS_TO_FORCE_IDLE));
    if (!awake_to_idle_increments)
    {
        /* every task which should have suspended already was, send out one GO_IDLE     */
        /* for tasks which do not suspend so that they are aware that we are idle       */
        /* (otherwise would have been notified non-suspending tasks in above for loop). */
      
      
        OS_Send_Message(OS_PSYNC_TASK, PS_EVT_GO_IDLE);
    }
}

/**********************************************************************
 *    Function: rl_check_wake_up
 *
 *  Parameters: none
 *
 *     Returns: none
 *
 * Description: This function determines whether or not the obd 
 *              should wake up from an idle state.  If the obd needs
 *              to wake up, this function will call Restart() and NOT
 *              return to the caller.
 *
 *********************************************************************/

static void rl_check_wake_up(void)
{
    if (rl_wake_up())
    {
        SY_Warm_Start();
    }
}
 

/**********************************************************************
*
*    Function: rl_low_voltage
*
*  Parameters: none
*
*     Returns: none
*
* Description: Performs action for staying in low voltage mode.
*              As long as 3V3 Main is not stable, we stay in while loop.
*              After 1 sec, system is immediatelly shut down.
*              When leaving power-fail state, Cold Start will be
*              performed.
*
*********************************************************************/
static void rl_low_voltage (void)
{

}
/**********************************************************************
*
*    Function: rl_full_awake_on
*
*  Parameters: none
*
*     Returns: none
*
* Description: indicate that power supply for awake AP is ready
*
*********************************************************************/
bool rl_full_awake_on(void)
{
    return(rl_data > RL_CS_IDLE);
}
/**********************************************************************
*
*    Function: RL_Enter_Powerfail
*
*  Parameters: none
*
*     Returns: none
*
* Description: Set the relays to the powerfail state
*
*
*********************************************************************/
extern void RL_Enter_Powerfail(bool low_voltage)
{
    if(low_voltage)
    {
        rl_low_voltage();
    }
    else
    {

    }
}

/**********************************************************************
*
*    Function: rl_emergency_shutdown
*
*  Parameters: none
*
*     Returns: none
*
* Description: Performs action for staying in powerfail reset mode.
*             As long as the battery sense line is low (Vbatt < 6.0V)
*             we stay in a while loop. 
*             If the battery sense line goes up again (Vbatt > 6.5V)
*             a warm start is done.
*
*********************************************************************/
static void rl_emergency_shutdown(void)
{
    if (PS_IDLE==PS_Get_Current_State())
        return;
    PS_Set_Idle();
#if 0
    rl_awake_to_idle();
#endif
    rl_write_timer(PF_DELAY_TIMER, OS_Time() + (MSec_To_Ticks(180000)));
    /*wait 5mins for recovery, then give up and go idle*/
    while (rl_timer_running(PF_DELAY_TIMER))
    {
        Feed_Dog();                               // make sure we have full watchdog timeout
        Enable_Interrupts();                      // in case WTNCS is corrupt
        Halt();
    }
}
/**********************************************************************
*
*    Function: RL_Force_Sleep
*
*  Parameters: none
*
*     Returns: none
*
* Description:Force sleep in case CAN bus is always active,disable CAN wake up interrupt
*
*********************************************************************/
void RL_Force_Sleep(void)
{
    Sys_Clear_Wakeup_Src_Flags();
    Sys_Req_Enter_Deep_Standby();   	
}

/**
  * @brief  Configures RTC clock source and prescaler.
  * @param  None
  * @retval None
  */
static void rl_rtc_config(void)
{
    NVIC_InitTypeDef  NVIC_InitStructure;
    EXTI_InitTypeDef  EXTI_InitStructure;
    
    /* Configure EXTI Line17(RTC Alarm) to generate an interrupt on rising edge */
    EXTI_ClearITPendingBit(EXTI_Line17);
    EXTI_InitStructure.EXTI_Line = EXTI_Line17;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    
    /* 2 bits for Preemption Priority and 2 bits for Sub Priority */

    NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable PWR and BKP clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

    /* Allow access to BKP Domain */
    PWR_BackupAccessCmd(ENABLE);

    /* Enable the LSE OSC */
    RCC_LSICmd(ENABLE);
    /* Wait till LSE is ready */
    /* Select the RTC Clock Source */
    RCC_LSEConfig(RCC_LSE_ON);
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
    {}

    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
    /* Enable RTC Clock */
    RCC_RTCCLKCmd(ENABLE);
    /* Wait for RTC registers synchronization */
    RTC_WaitForSynchro();

    /* Set RTC prescaler: set RTC period to 1sec */
    RTC_SetPrescaler(32767);

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    /* Enable the RTC Second */

    /* Enable the RTC Alarm interrupt */
    RTC_ITConfig(RTC_IT_ALR, ENABLE);

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    /* Alarm time */
    RTC_SetAlarm(RTC_GetCounter()+ rl_get_rtc_timeout());

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
}

void rl_rtc_disable(void)
{
  NVIC_InitTypeDef  NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  /* Configure EXTI Line17(RTC Alarm) to generate an interrupt on rising edge */
  EXTI_ClearITPendingBit(EXTI_Line17);
  EXTI_InitStructure.EXTI_Line = EXTI_Line17;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  /* 2 bits for Preemption Priority and 2 bits for Sub Priority */

  NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure);
}
//
///**
//  * @brief  Configures EXTI Lines.
//  * @param  None
//  * @retval None
//  */
//static void rl_exti_config(void)
//{
//    EXTI_InitTypeDef    EXTI_InitStructure;
//    NVIC_InitTypeDef    NVIC_InitStructure; 
//    
//    /* Configure EXTI Line17(RTC Alarm) to generate an interrupt on rising edge */
//    EXTI_ClearITPendingBit(EXTI_Line17);
//    EXTI_InitStructure.EXTI_Line = EXTI_Line17;
//    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
//    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//    EXTI_Init(&EXTI_InitStructure);
//    
//    /* moved from no_io routine */
//       /* 3. configure interrupts */
//    /* PA0-Wakeup pin for EXTI0 */
//    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
//    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
//    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
//    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//    EXTI_Init(&EXTI_InitStructure);
//    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
//
//    /* ACC */
//    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource2);
//    EXTI_InitStructure.EXTI_Line = EXTI_Line2;
//    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
////    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
//    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;      /*  */
//    
//    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//    EXTI_Init(&EXTI_InitStructure);
//    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
//
//    /* RING */
//    EXTI_InitStructure.EXTI_Line = EXTI_Line6;
//    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
//    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//    EXTI_Init(&EXTI_InitStructure);
//   
//    /* Accel */
//    EXTI_InitStructure.EXTI_Line = EXTI_Line8;
//    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
//    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//    EXTI_Init(&EXTI_InitStructure);
//    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
//}

/**
  * @brief  Get rtc_timeout value.
  * @param  None
  * @retval None
  */
uint32_t rl_get_rtc_timeout(void)
{
    return rtc_timeout;
}

/**
  * @brief  Set rtc_timeout value.
  * @param  None
  * @retval None
  */
void rl_set_rtc_timeout(uint32_t value)
{
    rtc_timeout = value;
}

/**
  * @brief  Get low_batt_sleep value.
  * @param  None
  * @retval None
  */
static uint8_t rl_get_low_batt_sleep(void)
{
    return low_batt_sleep;
}

/**
  * @brief  Set low_batt_sleep value.
  * @param  None
  * @retval None
  */
void rl_set_low_batt_sleep(uint8_t value)
{
    low_batt_sleep = value;
}

void rl_reset()
{
    Save_Param();
    SY_Cold_Start();
}

static void rl_go_sleep(void)
{
    if ((rl_first_go_sleep) && (0==Get_Last_GPS_uploaded()))
    {
        rl_first_go_sleep=false;
        DEBUG_PRINT0(DEBUG_MEDIUM,"[RELAYS] wait more, start sleep timer\n\r");
    }
    else
    {
        DEBUG_PRINT0(DEBUG_MEDIUM,"[RELAYS] Go to Sleep\n\r");
        rl_emergency_shutdown();
    }
}

extern uint8_t rl_get_acc_status(void)
{
    return acc_status;
}

extern void rl_set_acc_status(uint8_t status)
{
    acc_status=status;
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
