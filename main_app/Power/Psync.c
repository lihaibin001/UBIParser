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
/* $Header:   psync.c  $*/
/**********************************************************************
 *       Title:   psync.c
 *
 *  Description:  This is the standard code file for psync.
 *
 *      Author:   
 *
 *********************************************************************/

/**********************************************************************
 * Installation Instructions (periodic tasks, etc.)
 *
 *********************************************************************/

/**********************************************************************
 * Include header files
 *********************************************************************/
/* Dependent "compile.cmd"                                           */
/*********************************************************************/
#include "standard.h"
#include "fsm.h"
#include "Psync.hee"

#include "TelmApp.h"
#include "ATProtocol.h"
#include "TelmProtocol.h"

#define USE_DEBUG
#include "Debug.h"

/*********************************************************************
 * File level pragmas
 *********************************************************************/

/**********************************************************************
 * Constant and Macro Definitions using #define
 *********************************************************************/
#define PS_IO_DELAY 32

#define IGNITION_ON_DBNC  (32/PS_IO_DELAY) 
#define IGNITION_OFF_DBNC (250/PS_IO_DELAY)


#define CAN_Get_Ignition_State()  PS_Logical_Ignition_On()

/**********************************************************************
 * Enumerations and Structures and Typedefs
 *********************************************************************/

/**********************************************************************
 * Global and Const Variable Defining Definitions / Initializations
 *********************************************************************/
static ps_data_type ps_data = {0,0,0,false,false};
static PS_Cal_T ps_cal_data = {3,3,2};
//static PS_Cal_T ps_cal_data = {0xffff,0xffff,0xffff};
/**********************************************************************
 * Static Variables and Const Variables With File Level Scope
 *********************************************************************/
static bool idle_is_requested = false;
static bool rtc_is_not_requested = false;

__no_init uint8_t can_wakeup_cnt ;

/**********************************************************************
 * ROM Const Variables With File Level Scope
 *********************************************************************/
/**********************************************************************
 * Function Prototypes for Private Functions with File Level Scope
 *********************************************************************/
static void PS_Set_Logical_IGN_State(bool state);
static void PS_Set_Logical_Eng_State(bool state);
/*** STT Functions ***/
static uint8_t no_action(void);
static uint8_t ps_start_action(void);

static uint8_t ps_entry_root(void);
static uint8_t ps_entry_idle(void);
static uint8_t ps_entry_awake(void);
//static uint8_t ps_entry_sys_on(void);
//static uint8_t ps_entry_user_on(void);
static uint8_t ps_entry_eng_on(void);
static uint8_t ps_exit_awake(void);
//static uint8_t ps_exit_user_on(void);
static uint8_t ps_exit_eng_on(void);

static uint8_t ps_cs_root(void);
static uint8_t ps_cs_idle(void);
static uint8_t ps_cs_awake(void);
//static uint8_t ps_cs_sys_on(void);
//static uint8_t ps_cs_user_on(void);
static uint8_t ps_cs_eng_on(void);

//static uint8_t ps_check_user_mode_condition (void);
//static uint8_t ps_check_obd_active_user_on (void);
//static uint8_t ps_check_obd_active (void);

//static uint8_t ps_check_can_sleep (void);

//static uint8_t ps_set_force_user_off(void);
//static uint8_t ps_start_stdby_tmr(void);
//static uint8_t ps_restart_stdby_tmr(void);
//static uint8_t ps_stop_obd_on_tmr(void);
//static uint8_t ps_set_user_off_start_stdby_tmr (void);
//static uint8_t ps_set_ign_on_to_awake (void);

/********************************/
/* STATE TRANSITION DESCRIPTION */
/********************************/
#include "fsm_tran.h"
#include "ps_stt.h"

/********************************/
/* STATE TREE DESCRIPTION       */
/********************************/
#include "fsm_tree.h"
#include "ps_stt.h"

/**********************************************************************
 * Add User defined functions
 *********************************************************************/

/*===========================================================================*
 *
 * Please refer to the detailed description in psync.h.
 *
 *===========================================================================*/
//bool PS_Is_Obd_Off_Requested(void)
//{
//    return false;
//}

/**********************************************************************
 * Function Definitions
 *********************************************************************/

/**********************************************************************
*
*    Function: PS_Initialize
*
*  Parameters: none
*
*     Returns: none
*
* Description: Initialization of the PSYNC module
*
**********************************************************************/
void PS_Initialize (void)
{
    PS_Current_State_T new_state = 0;
    Data_Message_T     msg;

    if(Cold_Start())
    {
        PS_Set_Last_State(0);//set last state as PS_ROOT if cold start
        
        Sys_Clear_Int_battery_Flag();
    }

    msg.parts.msg  = START;
    msg.parts.data = 0;
    new_state = FSM_Process_Evt(msg, PS_ROOT, tree_psync);
    PS_Set_Current_State(new_state);
}

/**********************************************************************
*
*    Function: PS_Task
*
*  Parameters: none
*
*     Returns: none
*
* Description: Entry point for the psync task.
*
**********************************************************************/
void Psync_Task ( void *pvParameters )
{
    Data_Message_T     msg;
    Status_Type        status;
    PS_Current_State_T new_state;
    bool               state_changed;
    #ifdef USE_DEBUG
    DEBUG_PRINT0( DEBUG_MEDIUM, "[Psync]:Psync TASK Started!\r\n");
    #endif


#if 0
    if ((Cold_Start())
    	||(Sys_Is_Gsensor_Wakeup())
    	||(Sys_Is_Low_Batt_Wakeup()))
    {
        PS_Set_Obd_On(true);/*cold start, enter power on 5min mode*/
    }
#endif
    
    FOREVER
    {

        state_changed = false;

        // wait until next event in mailbox OR time expired
        status = OS_Wait_Message(OS_PSYNC_TASK,&msg.all, PS_TASK_REQUEUE_TIME);

        // process ALL events in queue first !!!
        while (E_OK == status)
        {
            // process event
            new_state = FSM_Process_Evt(msg, PS_Get_Current_State(), tree_psync);

            if (new_state != PS_Get_Current_State())
            {
                PS_Set_Current_State(new_state);
	            #ifdef USE_DEBUG
                DEBUG_PRINT1(DEBUG_HIGH,"[PSYNC]:PS STATE IS %x\n\r",new_state);
                #endif
                state_changed = true;
            }

            // get next event from mailbox without waiting
            status = OS_Receive_Message(OS_PSYNC_TASK,&msg.all);
        }

        // process all cs routines for the active state tree branch
        FSM_Process_CS(PS_Get_Current_State(), tree_psync);

        if (state_changed)
        {
            OS_Release_Resource(RES_RELAYS);
        }
    }
}
/*********************************************************************/
/*no actions                                                     */
/*********************************************************************/
static uint8_t no_action(void)
{
   return(0);
}
/*********************************************************************/
/* start actions                                                     */
/*********************************************************************/
static uint8_t ps_start_action (void)
{
    ps_put_permanent_sleep( false );
    ps_put_awake_requested( false );
    return(0);
}

/*********************************************************************/
/* entry actions                                                     */
/*********************************************************************/
static uint8_t ps_entry_root (void)
{
   PS_Current_State_T last_state = PS_Get_Last_State();

   /* correction of last_state */
   switch (last_state)
   {
//      case PS_USER_ON:
//      case PS_ENG_ON:
//         last_state = PS_SYS_ON;
//         break;
         
      case PS_AWAKE:
         /* do nothing, last state is okay */
         break;

      default:
         last_state = PS_AWAKE;
         break;   
   }

//   if (last_state == PS_AWAKE)
//   {
//      /* start standby timer to stay at least 30s in PS_AWAKE */
//      TMR_Start_Timer(PS_STDBY_TIMER, PS_STDBY_TIME3, null_action);
//   }
//   
   return(last_state);
}

static uint8_t ps_entry_idle (void)
{
   TMR_Stop_Timer(PS_STDBY_TIMER);

   {
      ps_put_permanent_sleep( true );
   }

   return(PS_IDLE);
}

static uint8_t ps_entry_awake (void)
{
//   PS_Set_Last_State(PS_AWAKE);
  Tick_Type awake_time;
  PS_Current_State_T last_state = PS_Get_Last_State();

   //start timer for a period to stay at least in PS_Awake before going PS_Idle
   if (PS_MIN_AWAKE_TIME > TMR_Get_Pending_Time(PS_MIN_AWAKE_TIMER))
   {
      if(MDG_In_Mode())
      {
      	TMR_Start_Timer(PS_MIN_AWAKE_TIMER, 0, null_action);//enter sleep right now
      }
      else if(Sys_Is_RTC_Deep_Wakeup())
      {
      	TMR_Start_Timer(PS_MIN_AWAKE_TIMER, 1000, null_action);
      }
      else
      {
      	TMR_Start_Timer(PS_MIN_AWAKE_TIMER, PS_MIN_AWAKE_TIME, null_action);
      }
   }

   if (Sys_Get_Int_battery_Flag())
   {
     awake_time = PS_STDBY_INT_BATT_TIME;
   }
   else if (last_state == PS_ENG_ON)
   {
     /* start standby timer to stay 300s in PS_AWAKE */
     awake_time = PS_STDBY_TIME_FROM_ENG_ON;
   }
   else
   {
     /* start standby timer to stay 30s in PS_AWAKE */
     awake_time = PS_STDBY_TIME3;     
   }

   TMR_Start_Timer(PS_STDBY_TIMER, awake_time, null_action);
   PS_Set_Last_State(PS_AWAKE);
   return(PS_AWAKE);
}

/*
static uint8_t ps_entry_sys_on (void)
{
   PS_Current_State_T last_state = PS_Get_Last_State();

   // correction of last_state in error case
   if ((last_state != PS_USER_ON)
   &&  (last_state != PS_ENG_ON))
   {
      last_state = PS_USER_ON;
   }

   return(last_state);
}
*/

//
//static uint8_t ps_entry_user_on (void)
//{
//   PS_Set_Last_State(PS_USER_ON);
//   ps_put_user_on( true );
//   TMR_Stop_Timer(PS_STDBY_TIMER);
//
//   /* start timer only if ign off and manuf diag not active AND eng diag not active */
//   if ((PS_Logical_Ignition_On() == ON))
//   {
//      /* start OBD_ON timer to stay here before going back to PS_Awake */
//      if(PS_Get_Obd_On_Minutes() != 0xFFFF)
//      {
//      }
//   }
//   else
//   {
//      /* start OBD_ON timer to stay here before going back to PS_Awake */
//      if(PS_Get_Obd_On_Minutes() != 0xFFFF)
//      {
//              if((Sys_Is_Gsensor_Wakeup())
//                  ||(Sys_Is_Low_Batt_Wakeup()))
//              {
//
//              }
//              else
//              {
//
//              }
//      }
//      else
//      {
//
//      }
//   }      
//   return(PS_USER_ON);
//}
//

static uint8_t ps_entry_eng_on (void)
{
   PS_Set_Last_State(PS_ENG_ON);
   Periodic_Clear_Low_Volt_Cnt();
   
   //
   TMR_Start_Timer(PS_INT_BATT_CHRG_DELAY_TIMER, PS_CHRG_DELAY_TIME, null_action);
   
   return(PS_ENG_ON);
}

/*********************************************************************/
/* exit actions                                                      */
/*********************************************************************/
static uint8_t ps_exit_awake (void)
{
   TMR_Stop_Timer(PS_MIN_AWAKE_TIMER);
   TMR_Stop_Timer(PS_STDBY_TIMER);
   return(0);
}

//static uint8_t ps_exit_user_on (void)
//{
//   return(0);
//}

static uint8_t ps_exit_eng_on (void)
{
   write_flash_last_pos();
   
   //
   TMR_Stop_Timer(PS_INT_BATT_CHRG_DELAY_TIMER);
   TMR_Stop_Timer(PS_INT_BATT_CHRG_TIMER);
   IO_CHARGE_CTL(Bit_RESET);
   
#if 0
   OS_Send_Message(OS_IOT_TASK, Build_Message(TM_EVT_ENG_ON_OFF, OFF));
#endif
   return(0);
}

/*********************************************************************/
/* cs routines                                                       */
/*********************************************************************/
static uint8_t ps_cs_root (void)
{
   return(0);
}

static uint8_t ps_cs_idle (void)
{
    if (ps_awake_requested())
    {
        OS_Send_Message(OS_PSYNC_TASK, PS_EVT_AWAKE);
    }
    else
    {
        OS_Release_Resource(RES_RELAYS);
    }
    return(0);
}

static uint8_t ps_cs_awake (void)
{
    /* check timer first to block checking of the other*/
   /*...function for at least PS_MIN_AWAKE_TIME*/
   if (TMR_Check_Timer(PS_MIN_AWAKE_TIMER))
   {
//      if ((PS_Is_Idle_Requested())
//	  	|| (Sys_Is_RTC_Deep_Wakeup()))
//      {
//         /* force immediate shutdown */
////         OS_Send_Message(OS_PSYNC_TASK, PS_EVT_IDLE);
//      }
//      else
      {
         if (TMR_Check_Timer(PS_STDBY_TIMER))/*standby time is up,go to idle*/
         {
//              OS_Send_Message(OS_PSYNC_TASK, PS_EVT_IDLE);
            if (!ACC_Is_Volt_Checking())
            {
                OS_Send_Message(OS_PSYNC_TASK, PS_EVT_GO_IDLE);
            }
         }
      }         
   }

//   if (TMR_Check_Timer(PS_STDBY_TIMEOUT_TIMER))//standby timeout, force sleep
//   {
//      DEBUG_PRINT0(DEBUG_HIGH,"[SYSTEM]:Standby Timeout, Force Sleep!!!\n\r");
//      RL_Force_Sleep();
//   }

   if (!Sys_Get_Int_battery_Flag())      // not use internal battery case
   {     
 //    if(ACC_Is_On_After_Delay())     //ACC on
     if(rl_get_acc_status())     //ACC on
     {
       PS_Set_Ignition(ON);
     }
   }
   return(0);
}

//static uint8_t ps_cs_sys_on (void)
//{
//   return(0);
//}

//static uint8_t ps_cs_user_on (void)
//{
////   if ((TMR_Check_Timer(PS_OBD_ON_TIMER))
////   	&&(!MDG_In_Mode()))
//   {
////      OS_Send_Message(OS_PSYNC_TASK, PS_EVT_OBD_ON_TMR_EXP);
//   }
//   return(0);
//}

static uint8_t ps_cs_eng_on (void)
{
    uint16_t vol_ext=Pwr_Fail_Get_Voltage();
    uint16_t vol_int=Pwr_Fail_Get_Int_Voltage();
    
    if((!rl_get_acc_status()) || Sys_Get_Int_battery_Flag()) /* ACC OFF */
    {
        PS_Set_Ignition(OFF);
    }
    
    if (TMR_Check_Timer(PS_INT_BATT_CHRG_DELAY_TIMER))
    {
        if (Pwr_Is_Charge_Request() && (!TMR_Is_Timer_Active(PS_INT_BATT_CHRG_TIMER)))
        {
          TMR_Start_Timer(PS_INT_BATT_CHRG_TIMER, PS_CHRG_TIME, null_action);
          IO_CHARGE_CTL(Bit_SET);
          DEBUG_PRINT1(DEBUG_MEDIUM,"[RELAY]Int Battery start charing=%d\n\r",vol_int);
        }
        else if (TMR_Check_Timer(PS_INT_BATT_CHRG_TIMER))
        {
          IO_CHARGE_CTL(Bit_RESET); // time out, stop chagring
        }
        else
        {
          //In charging
        }
    }
    
    return(0);
}


/*********************************************************************/
/* condition actions                                                 */
/*********************************************************************/
//static uint8_t ps_check_user_mode_condition (void)
//{
//   uint8_t condition = 1;
//
//   if (PS_Logical_Ignition_On())
//      condition = 1;
//   else
//      condition = 2;
//
//   return(condition);
//}
//
//static uint8_t ps_check_obd_active_user_on (void)
//{
//   uint8_t condition = 2;
//   
//   if (false)
//   {
//      condition = 1;
//   }
//   else
//   {
//      condition = 2;
//   }
//   return(condition);
//}
//
//static uint8_t ps_check_obd_active (void)
//{
//   uint8_t condition = 2;
//
//   if (false)//(Pwr_Fail_Is_Reset_Condition())
//      condition = 1;
//   else
//      condition = 2;
//
//   return(condition);
//}
//
//#if 0
//static uint8_t ps_check_user_on (void)
//{
//   uint8_t condition = 2;
//
//   if (ps_user_on())
//      condition = 1;
//   else
//      condition = 2;
//
//   return(condition);
//}
//#endif
//static uint8_t ps_check_can_sleep (void)
//{
//   uint8_t condition = 2;
//
//  return(condition);
//}

/*********************************************************************/
/* transition actions                                                */
/*********************************************************************/
//static uint8_t ps_set_force_user_off (void)
//{
//   ps_put_user_on( false );
//   DEBUG_PRINT0(DEBUG_HIGH,"[SYSTEM]:ACC ON time out, Force Sleep!!!\n\r");
//   RL_Force_Sleep();//force sleep, disable can interrupt
//   return(0);
//}
//
//static uint8_t ps_set_user_off_start_stdby_tmr (void)
//{
//   ps_put_user_on( false );
//
//   TMR_Start_Timer(PS_STDBY_TIMER, PS_STDBY_TIME3, null_action);
//   return(0);
//}
//
//static uint8_t ps_start_stdby_tmr (void)
//{
//   TMR_Start_Timer(PS_STDBY_TIMER, PS_STDBY_TIME3, null_action);
//   return(0);
//}
//
//static uint8_t ps_restart_stdby_tmr (void)
//{
//   TMR_Start_Timer(PS_STDBY_TIMER, PS_STDBY_RETRY_TIME, null_action);
//   if (!TMR_Is_Timer_Active(PS_STDBY_TIMEOUT_TIMER))
//   {
//      TMR_Start_Timer(PS_STDBY_TIMEOUT_TIMER, PS_STDBY_TIMEOUT_TIME, null_action);
//   }
//   return(0);
//}
//
//static uint8_t ps_stop_obd_on_tmr (void)
//{
////   TMR_Stop_Timer(PS_OBD_ON_TIMER);
//   TMR_Stop_Timer(TELM_BATT_CHECK_DELAY_TIMER);
//   TMR_Stop_Timer(TELM_BATT_CHECK_TIMER);
//   return(0);
//}
//
//static uint8_t ps_set_ign_on_to_awake (void)
//{
//   ps_put_user_on( false );
//
////   TMR_Start_Timer(PS_STDBY_TIMER, PS_STDBY_TIME_FROM_ENG_ON, null_action);
//
//   TMR_Start_Timer(TELM_BATT_CHECK_DELAY_TIMER, PS_BATT_CHECK_DELAY_TIME, PS_Start_Batt_Check_Timer);
//   return(0);
//}

/*********************************************************************/
/* user public function                                     */
/*********************************************************************/
void PS_Start_Batt_Check_Timer (void)
{
    if (Pwr_Fail_Is_Mute_Condition())
    {
        TMR_Start_Timer(TELM_BATT_CHECK_TIMER, 30000, PS_Check_Batt_Before_Sleep);
    }
}

/**********************************************************************
*
*    Function: PS_Check_Batt_Before_Sleep
*
*  Parameters: None
*
*
* Description: 
*
**********************************************************************/
void PS_Check_Batt_Before_Sleep (void)
{
   if (Pwr_Fail_Is_Mute_Condition())//workaround, send low battery message if power failed
   {
//        OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_BATTERY_FAULT, 1));				
   }
}

/**********************************************************************
*
*    Function: PS_Send_Crash_Msg_On_Timer
*
*  Parameters: None
*
*
* Description: 
*
**********************************************************************/
void PS_Send_Crash_Msg_On_Timer (void)//workaround only
{
   //if (false == PS_Logical_Ignition_On())//if ACC ON, discard this message
   if (!PS_Eng_On())//if Eng ON, discard this message
   {
//        OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_CAR_CRASHED, 1) );				
   }
}

/**********************************************************************
*
*    Function: PS_Idle
*
*  Parameters: None
*
*     Returns: true  - if in RL is in idle state
*              false - if not
*
* Description: Checks if RL is in idle state. This is used only from watchtmr.
*
**********************************************************************/
bool PS_Idle(void)
{
   return((RL_In_Idle())&&(!PS_Awake()));
}

/**********************************************************************
*
*    Function: PS_Awake
*
*  Parameters: None
*
*     Returns: true  - if PS is at least awake
*              false - if not
*
* Description: Checks if system is awake.
*
**********************************************************************/
bool PS_Awake(void)
{
   return(!(PS_Get_Current_State() <= PS_IDLE) || ps_awake_requested());
}

/**********************************************************************
*
*    Function: PS_Play_On
*
*  Parameters: None
*
*     Returns: true  - if play is requested
*              false - if play is not requested
*
* Description: Checks if play mode is requested.
*
**********************************************************************/
//bool PS_Play_On(void)
//{
//   return((PS_Get_Current_State() == PS_USER_ON)
//       || (PS_Get_Current_State() == PS_ENG_ON));
//}

///**********************************************************************
//*
//*    Function: PS_User_On
//*
//*  Parameters: None
//*
//*     Returns: true  - if obd is switched on by user
//*              false - if obd is not switched on by user
//*
//* Description: Checks if obd is switched on by user.
//*
//**********************************************************************/
//bool PS_User_On(void)
//{
//   return(PS_Get_Current_State() == PS_USER_ON);
//}

/**********************************************************************
*
*    Function: PS_Eng_On
*
*  Parameters: None
*
*     Returns: None
*
* Description: 
*
*********************************************************************/
bool PS_Eng_On(void)
{
   return(PS_Get_Current_State() == PS_ENG_ON);
}

/**********************************************************************
*
*    Function: PS_Running
*
*  Parameters: None
*
*     Returns: true  - if in system is still running
*              false - if not
*
* Description: Checks if system is still running
*
*********************************************************************/
bool PS_Running(void)
{
   //return(!(PS_Get_Current_State() == PS_IDLE) || ps_awake_requested());
   return(!(PS_Get_Current_State() == PS_IDLE));
}

/**********************************************************************
*
*    Function: PS_Full_System
*
*  Parameters: None
*
*     Returns: true  - if in PS_Full_System
*              false - if not
*
* Description: Checks if PS_Full_System
*
**********************************************************************/
bool PS_Full_System(void)
{
   return(!(PS_Get_Current_State() == PS_IDLE)
       && !(PS_Get_Current_State() == PS_AWAKE));
}

/**********************************************************************
*
*    Function: PS_Permanent_Sleep
*
*  Parameters: None
*
*     Returns: true  - if in permanent sleep mode
*              false - if not
*
* Description: Checks if in permanent sleep mode
*
*********************************************************************/
bool PS_Permanent_Sleep(void)
{
   return(ps_permanent_sleep());
}

/**********************************************************************
*
*    Function: PS_Pending_Awake
*
*  Parameters: None
*
*     Returns: true  - if awake requested
*              false - if awake not requested
*
* Description: Checks if awake is requested
*
**********************************************************************/
bool PS_Pending_Awake(void)
{
   bool wake_up = false;
   /*Fixme:reserved*/
   return(wake_up);
}

/**********************************************************************
*
*    Function: PS_Get_Idle_Time
*
*  Parameters: None
*
*     Returns: How long the obd should remain idle, given in absolute
*              idle ticks (DO NOT ADD OS_Time() TO THIS RESULT);
*              0 indicates forever)
*
* Description: Calculates how long the obd should stay in idle.
*
*********************************************************************/
Tick_Type PS_Get_Idle_Time(void)
{
   // calculate here the time and condition to stay idle:
   // if not permanent sleep e.g. 2h TIM readiness OR 2x 2h Timeframe
   // then come out of IDLE every 1.5 sec to check for TA
   // else stay in idle forever (= 0)
   /*CJ:reserved for future application,should be permanent sleep by now*/
   return(ps_permanent_sleep() ? 0 : (OS_Time() + PS_IDLE_TIME));
}

/**********************************************************************
*
*    Function: PS_Is_Idle_Requested
*
*  Parameters: None
*
*     Returns: true  - if idle requested
*              false - if idle not requested
*
* Description: returns if idle is requested
*
**********************************************************************/
bool PS_Is_Idle_Requested(void)
{
   return(idle_is_requested);
}

/**********************************************************************
*
*    Function: PS_Is_RTC_Requested
*
*  Parameters: None
*
*     Returns: true  - if idle requested
*              false - if idle not requested
*
* Description: returns if idle is requested
*
**********************************************************************/
bool PS_Is_RTC_Requested(void)
{
   return(!rtc_is_not_requested);
}

/**********************************************************************
*
*    Function: PS_Go_Idle
*
*  Parameters: rtc_config:if RTC periodically wake up is required
*
*     Returns: None
*
* Description: Forces a qucik (hard) transition to idle mode
*
*********************************************************************/
void PS_Go_Idle(bool rtc_config)
{
    idle_is_requested = true;
    rtc_is_not_requested = rtc_config;
    OS_Send_Message(OS_PSYNC_TASK, PS_EVT_GO_IDLE);
}

/**********************************************************************
 *
 *    Function: PS_Go_Awake
 *
 *  Parameters: awake_time_in_ms - ms value for time the obd should
 *              remain awake
 *
 *     Returns: None
 *
 * Description: This function requests a wake up of the obd for
 *              a specified period of time.
 *
 *********************************************************************/
void PS_Go_Awake(Tick_Type awake_time_in_ms)
{
    if (awake_time_in_ms > TMR_Get_Pending_Time(PS_MIN_AWAKE_TIMER))
    {
        TMR_Start_Timer(PS_MIN_AWAKE_TIMER, awake_time_in_ms, null_action);
    }

    OS_Send_Message(OS_PSYNC_TASK, PS_EVT_AWAKE);
}

/**********************************************************************
*
*    Function: PS_Set_Ignition
*
*  Parameters: status 0=OFF, 1=ON
*
*     Returns: None
*
* Description: informs PSYNC about the ignition status, called by the
*              CAN module
*
**********************************************************************/
void PS_Set_Ignition(bool status)
{
    Message_Type msg;
    if(status != PS_Logical_Ignition_On())  
    {
        PS_Set_Logical_IGN_State(status);
        msg = PS_Logical_Ignition_On() ? PS_EVT_IGN_ON : PS_EVT_IGN_OFF;
        OS_Send_Message(OS_PSYNC_TASK, msg);
    }
}
/**********************************************************************
*
*    Function: PS_Set_Engine_On
*
*  Parameters: status 0=OFF, 1=ON
*
*     Returns: None
*
* Description: 
*
**********************************************************************/
void PS_Set_Engine_On(bool status)
{
    Message_Type msg;
    if(status != PS_Logical_Engine_On())  
    {
        PS_Set_Logical_Eng_State(status);
        msg = PS_Logical_Engine_On() ? PS_EVT_ENG_ON : PS_EVT_ENG_OFF;
        OS_Send_Message(OS_PSYNC_TASK, msg);
    }
}

/**********************************************************************
*
*    Function: PS_Set_Obd_On
*
*  Parameters: status t/f
*
*     Returns: None
*
* Description: requests PS to switch Obd ON (behaviour like onoff key)
*
*********************************************************************/
void PS_Set_Obd_On(bool status)
{
   OS_Send_Message(OS_PSYNC_TASK, status ? PS_EVT_OBD_ON : PS_EVT_OBD_OFF);
}

/**********************************************************************
*
*    Function: PS_Set_ON_Minutes
*
*  Parameters: None
*
*     Returns: None
*
* Description: sets the ps_on_minutes
*
*********************************************************************/
void PS_Set_ON_Minutes(uint16_t minutes)
{
   PS_Set_Obd_On_Minutes(minutes);

//   if (TMR_Is_Timer_Active(PS_OBD_ON_TIMER))
   {
      //restart timer with new value
//      TMR_Start_Timer(PS_OBD_ON_TIMER, PS_OBD_ON_TIME, null_action);
   }
}
/*===========================================================================*\
 *
 * Please refer to the detailed description in psync_ps.h.
 *
\*===========================================================================*/
void PS_Set_Obd_On_Minutes(uint16_t minutes)
{
   ps_cal_data.obd_on_minutes = minutes;
}

/*===========================================================================*\
 *
 * Please refer to the detailed description in psync_ps.h.
 *
\*===========================================================================*/
uint16_t PS_Get_Obd_On_Minutes(void)
{
   return(ps_cal_data.obd_on_minutes);
}
/*===========================================================================*\
 *
 * Please refer to the detailed description in psync_ps.h.
 *
\*===========================================================================*/
void PS_Set_ACC_On_Minutes(uint16_t minutes)
{
   ps_cal_data.acc_on_minutes = minutes;
}

/*===========================================================================*\
 *
 * Please refer to the detailed description in psync_ps.h.
 *
\*===========================================================================*/
uint16_t PS_Get_ACC_On_Minutes(void)
{
   return(ps_cal_data.acc_on_minutes);
}

/*===========================================================================*\
 *
 * Please refer to the detailed description in psync_ps.h.
 *
\*===========================================================================*/
void PS_Set_Event_On_Minutes(uint16_t minutes)
{
   ps_cal_data.event_on_minutes = minutes;
}

/*===========================================================================*\
 *
 * Please refer to the detailed description in psync_ps.h.
 *
\*===========================================================================*/
uint16_t PS_Get_Event_On_Minutes(void)
{
   return(ps_cal_data.event_on_minutes);
}

/**********************************************************************
*
*    Function: PS_Set_Restart_ON_Timer
*
*  Parameters: None
*
*     Returns: None
*
* Description: starts timer without storing new value
*
*********************************************************************/
void PS_Restart_ON_Timer(uint16_t minutes)
{
//   if (TMR_Is_Timer_Active(PS_OBD_ON_TIMER))
   {
      //restart timer with new value
//      TMR_Start_Timer(PS_OBD_ON_TIMER, minutes * 60000, null_action);
   }
}

/**********************************************************************
*
*    Function: PS_Get_ON_Minutes
*
*  Parameters: None
*
*     Returns: None
*
* Description: returns ps_on_minutes
*
*********************************************************************/
uint16_t PS_Get_ON_Minutes(void)
{
   return(PS_Get_Obd_On_Minutes());
}

/**********************************************************************
*
*    Function: PS_Set_Idle
*
*  Parameters: None
*
*     Returns: None
*
* Description: Forces a hard idle transition
*
*********************************************************************/
void PS_Set_Idle(void)
{
   PS_Set_Current_State(PS_IDLE);
}

/**********************************************************************
*
*    Function: PS_Update
*
*  Parameters: None
*
*     Returns: None
*
* Description: forces a immediate run of PSYNC task
*
*********************************************************************/
//static void PS_Update(void)
//{
//   OS_Send_Message(OS_PSYNC_TASK, PS_EVT_UPDATE);
//}

/*===========================================================================*\
 *
 * Please refer to the detailed description in psync.h.
 *
\*===========================================================================*/
void PS_Set_Current_State(PS_Current_State_T state)
{
   ps_data.current_state = state;
}

/*===========================================================================*\
 *
 * Please refer to the detailed description in psync.h.
 *
\*===========================================================================*/
PS_Current_State_T PS_Get_Current_State(void)
{
   return(ps_data.current_state);
}

/*===========================================================================*\
 *
 * Please refer to the detailed description in psync.h.
 *
\*===========================================================================*/
void PS_Set_Last_State(PS_Current_State_T state)
{
   ps_data.last_state = state;
}

/*===========================================================================*\
 *
 * Please refer to the detailed description in psync.h.
 *
\*===========================================================================*/
PS_Current_State_T PS_Get_Last_State(void)
{
   return(ps_data.last_state);
}

/*===========================================================================*\
 *
 * Please refer to the detailed description in psync.h.
 *
\*===========================================================================*/
void PS_Set_Flag(PS_Flags_Enum_T flag, bool status)
{
   if(true == status)
   {
       Set_Bit(&ps_data.flags, flag);
   }
   else
   {
       Clear_Bit(&ps_data.flags, flag);
   }
}

/*===========================================================================*\
 *
 * Please refer to the detailed description in psync_ps.h.
 *
\*===========================================================================*/
bool PS_Get_Flag(PS_Flags_Enum_T flag)
{
   return(Read_Bit(&ps_data.flags, flag));
}

/**********************************************************************
 * Description: 
 *  Parameters: None
 *     Returns: 
 *********************************************************************/
bool PS_Logical_Engine_On(void)
{
   return(ps_data.logical_engine);
}

/**********************************************************************
 * Description: Set logical engine state
 *  Parameters: new state value
 *     Returns: void
 *********************************************************************/
static void PS_Set_Logical_Eng_State(bool state)
{
   ps_data.logical_engine = state;
}

/**********************************************************************
 * Description: Accessor to determine whether logical ignition is on or not.
 *  Parameters: None
 *     Returns: whether or not the logical ignition is on
 *********************************************************************/
bool PS_Logical_Ignition_On(void)
{
   return(ps_data.logical_ignition);
}
/**********************************************************************
 * Description: Set logical ignition state
 *  Parameters: new state value
 *     Returns: void
 *********************************************************************/
static void PS_Set_Logical_IGN_State(bool state)
{
   ps_data.logical_ignition = state;
}

/**********************************************************************
 *
 * Revision History
 *
 *********************************************************************
 *********************************************************************/
