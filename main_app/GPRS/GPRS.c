/**********************************************************************
   Title                      : GPRS.c         
                                                                         
   Module Description         : Telematics module. This file is the communication task
                                        with telm modlue.

   Author                     : 
   
 *********************************************************************/

/**********************************************************************
 * Include files                                                       
 *********************************************************************/
#include "standard.h"
#include "ATProtocol.h"
#include "ATApp.h"
#include "TelmProtocol.h"
#include "stm32f10x_rtc.h"
#include <string.h>

#include "uart.h"
#include "gps.h"
//#include "spi_flash.h"
#define USE_DEBUG
#include "Debug.h"

/**********************************************************************
 * Constant and Macro Definitions using #define
 *********************************************************************/
#define TIMER_QUERY_INTERVAL	(MSec_To_Ticks(2000))
#define TIMER_UPGRADE_GUARD   (10 * ONE_MINUTE_IN_TICKS)   //3G firmware upgrade guard timer is 

//#define TEST_2G_SIGNAL
//#define TEST_BATT
//#define TEMPERATURE_READ_ENABLE
//#define USE_DEV_TRACE

#define OTA_DATA_LEN_MAX 0x3FF00
#ifdef USE_DEV_TRACE
#define DEV_TRACE_TIME (1400000)
#endif
/*********************************************************************/
/* Enumerations and Structures and Typedefs                          */
/*********************************************************************/
static bool	telematics_module_pwr_on;
static bool telematics_module_upgrade_state; // 0: No upgrade state, 1: upgrade in progress

/* Addr 0x00 - 0x03 */
typedef struct init_flag_tag
{
    uint8_t flags[4];
}init_flag_t;

typedef struct gps_back_header_tag
{
    uint8_t total_num;
    uint8_t sent_ptr;
    uint8_t store_ptr;
}gps_back_header_t;

typedef struct ota_header_tag
{
    uint8_t ota_start_flag;
    uint8_t ota_data_len[4];
    uint8_t sw_ver[4];
    uint8_t checksum[4];
}ota_header_t;

static uint8_t dev_activated_state = 1; // 0: Not activated, 1: activated

//static uint8_t harsh_data = 0; // 0: No harsh data to send, 1: data to send
static uint8_t accept_call_flag = 0; // 0: no call, 1: call accepted
extern  uint8_t prvATProt_Uart_Transmit(const char* tx_buf, uint16_t bytes);
/**********************************************************************
 * Function Prototypes for Private Functions with File Level Scope
 *********************************************************************/
static void prvGSM_Module_Init(void);

static void prvGSM_evt_nop(int16_t data);
static void prvGSM_evt_car_crashed(int16_t data);
static void prvGSM_evt_reset_module(int16_t data);
static void prvGSM_evt_sys_power_lost(int16_t data);
static void prvGSM_evt_car_theft(int16_t data);
static void prvGSM_evt_gps_upload(int16_t data);
static void prvGSM_evt_dev_upload(int16_t data);
static void prvGSM_evt_batt_upload(int16_t data);
static void prvGSM_evt_env_test(int16_t data);
static void prvGSM_evt_backup_gps_upload(int16_t data);
static void prvGSM_evt_config_upload(int16_t data);
static void prvGSM_evt_ota(int16_t data);
static void prvGSM_evt_gps_first_fixed(int16_t data);
static void prvGSM_evt_trip(int16_t data);
static void prvGSM_evt_dbe(int16_t data);

//static void prvVehicleCrashed_Scan_init(void);

uint32_t get_OTA_checksum(void);

/*********************************************************************/
/* Static Variables and Const Variables With File Level Scope        */
/*********************************************************************/
static void_int16_fptr tcom_event_handler[]=
{
    prvGSM_evt_nop,					//TM_EVT_NOP
    prvGSM_evt_reset_module,			//TM_EVT_RESET_MODULE
    prvGSM_evt_car_crashed,			//TM_EVT_CAR_CRASHED
    prvGSM_evt_sys_power_lost,		//TM_EVT_BATTERY_FAULT 
    prvGSM_evt_car_theft,    //TM_EVT_CAR_THEFT	 //!!NOT USED
    prvGSM_evt_gps_upload,
    prvGSM_evt_dev_upload,
    prvGSM_evt_batt_upload,
    prvGSM_evt_env_test,
    prvGSM_evt_backup_gps_upload,
    prvGSM_evt_config_upload,
    prvGSM_evt_ota,
    prvGSM_evt_gps_first_fixed,
    prvGSM_evt_trip,
    prvGSM_evt_dbe,
};

/*********************************************************************/
/* User file include                                                 */
/*********************************************************************/

/**********************************************************************
 * Function Definitions
 *********************************************************************/

/*******************************************************************************
*    Function:  GPRS_Set_Telematics_Activation_State
*
*  Parameters:  
*     Returns:  None
* Description:  GPRS_Set_Telematics_Activation_State.
*******************************************************************************/
void GPRS_Set_Telematics_Activation_State(uint8_t act_flag)
{
//    dev_activated_state = act_flag;
//    if (act_flag == 1)
//        set_activated();
}

/*******************************************************************************
*    Function:  GPRS_Get_Telematics_Activation_State
*
*  Parameters:  
*     Returns:  None
* Description:  GPRS_Get_Telematics_Activation_State.
*******************************************************************************/
uint8_t GPRS_Get_Telematics_Activation_State(void)
{
   return dev_activated_state;
}
/*******************************************************************************
*    Function:  GPRS_Set_Telematics_Module_Pwr_State
*
*  Parameters:  
*     Returns:  None
* Description:  GPRS_Set_Telematics_Module_Pwr_State.
*******************************************************************************/
void GPRS_Set_Telematics_Module_Pwr_State(bool pwr_flag)
{
	telematics_module_pwr_on = pwr_flag;
}

/*******************************************************************************
*    Function:  GPRS_Get_Telematics_Module_Pwr_State
*
*  Parameters:  
*     Returns:  None
* Description:  GPRS_Get_Telematics_Module_Pwr_State.
*******************************************************************************/
bool GPRS_Get_Telematics_Module_Pwr_State(void)
{
	return telematics_module_pwr_on;
}
/*******************************************************************************
*    Function:  GPRS_Get_FMUpgrade_state
*
*  Parameters:  void
*     Returns:  void
* Description:  return the firmware upgrade state: 1 is in upgrade mode, 0 is not in upgrade mode
*******************************************************************************/
bool GPRS_Get_FMUpgrade_state(void)
{
   return telematics_module_upgrade_state;
}
/*******************************************************************************
*    Function:  GPRS_Get_FMUpgrade_state
*
*  Parameters:  void
*     Returns:  void
* Description:  return the firmware upgrade state: 1 is in upgrade mode, 0 is not in upgrade mode
*******************************************************************************/
void GPRS_Set_FMUpgrade_state(uint8_t fmupgrade_state)
{
   telematics_module_upgrade_state = fmupgrade_state;
}

/*******************************************************************************
*    Function:  IOT_Task
*
*  Parameters:  None
*     Returns:  None
* Description:  TASK to handle communication between MCU and GPRS module.
*******************************************************************************/
extern void IOT_Task(void* pvParameters)
{
    Data_Message_T msg;
    uint32_t csq_time = 0;

    #ifdef USE_DEBUG
    DEBUG_PRINT0( DEBUG_MEDIUM, "[2G]:2G TASK Started!\r\n");
    #endif

    prvGSM_Module_Init();
    csq_time = OS_Time();

#ifdef USE_DEV_TRACE
    if (u3G_Get_FMUpgrade_state() != 1)
        TMR_Start_Timer(DEV_TRACE_TIMER, DEV_TRACE_TIME, vSend_Dev_Trace);	
#endif


    while(PS_Running()&&(Mdg_SW_Upgrage_mode==false))
    {
        /* wait event from other tasks */
        if(E_OK == OS_Wait_Message(OS_IOT_TASK, &msg.all, MSec_To_Ticks(20)))// 20ms testing
      	{
            if(msg.parts.msg < TM_NUM_EVENTS)
            {
                if(NULL != tcom_event_handler[msg.parts.msg])
                {
                    (*tcom_event_handler[msg.parts.msg])(msg.parts.data);
                }
            }
        }

        if (GPRS_Get_Telematics_Module_Pwr_State())
        {
        }

        if ((csq_time + MSec_To_Ticks(300)) < OS_Time())
        {
            vATApp_Loop_Check();
#ifdef TELM_TEST_PHONE
            if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8) == 0)
            {
                DEBUG_PRINT0(DEBUG_HIGH,"RING!\r\n");
                if (accept_call_flag==0)
                {
                    accept_call();
                    accept_call_flag=1;
                }
            }
            else
            {
                accept_call_flag=0;
            }
#endif
            csq_time = OS_Time();
        }

        if (GPRS_Get_Telematics_Module_Pwr_State())
        {
            vATProt_Check_Receive();
            vTelmApp_main_loop();
            vATProt_Check_Transmit();
        }
    }
    /* execute necessary process for going to sleep */

    OS_Terminate_Task();
}

/*******************************************************************************
*    Function:  prvGSM_Module_Init
*
*  Parameters:  None
*     Returns:  None
* Description:  Initialize variables TCOM task use. UART Initilization should be put here.
*******************************************************************************/
static void prvGSM_Module_Init(void)
{
    GPRS_Set_Telematics_Module_Pwr_State(false);
    vATProt_Init();
    vATApp_Init();
    GPRS_Set_Telematics_Module_Pwr_State(true);
}

/*******************************************************************************
*    Function:  v3G_Module_GoSleep
*
*  Parameters:  None
*     Returns:  None
* Description:  Initialize variables TCOM task use. UART Initilization should be put here.
*******************************************************************************/
void GPRS_Module_GoSleep(void)
{
    if ((1 == GPRS_Get_FMUpgrade_state()) &&
          (1 == GPRS_Get_Telematics_Activation_State()))
    {
        GPRS_Set_FMUpgrade_state(0);
    }
    vATProt_sendAT_Command(AT_CMD_IPCLOSE_EXCUTE,NULL,NULL);
    OS_Sleep(600);
    vATApp_GoSleep();
    vATProt_GoSleep();
}

/*******************************************************************************
*    Function:  prvGSM_evt_nop
*
*  Parameters:  None
*     Returns:  None
* Description:  Send series of command to 3G module for initilization.
*******************************************************************************/
static void prvGSM_evt_nop(int16_t data)
{
}

/*******************************************************************************
* Function:  prvGSM_evt_reset_module
*
* Parameters:  None
* Returns:  None
* Description:  
*******************************************************************************/
static void prvGSM_evt_reset_module(int16_t data)
{
   vATProt_Com_Reset();
   vATApp_Init();
}

/*******************************************************************************
*    Function:  prvGSM_evt_car_crashed
*
*  Parameters:  None
*     Returns:  None
* Description:  
*******************************************************************************/
static void prvGSM_evt_car_crashed(int16_t data)
{
    if (NET_DATA_TRANS == vATProt_Get_TCP_State())
    {
	    vTelmApp_uploadData(TELM_SEC_EVT_CRASH, NULL);
    }
    else
    {
        vTelmApp_Backup_Msg(TELM_SEC_EVT_CRASH);
    }
}

/*******************************************************************************
* Function:  prvGSM_evt_sys_power_lost
*
* Parameters:  None
* Returns:  None
* Description:  automotive battery power lost
*******************************************************************************/
static void prvGSM_evt_sys_power_lost(int16_t data)
{
    if (1 != GPRS_Get_Telematics_Activation_State())
        return;
    DEBUG_PRINT0(DEBUG_HIGH,"[2G] low battery voltage!\n\r");
    /* Set car battery abnormal status */
//    set_battery_status(1);
    if (NET_DATA_TRANS == vATProt_Get_TCP_State())
    {
	    vTelmApp_uploadData(TELM_SEC_EVT_BATT, NULL);
    }
    else
    {
        vTelmApp_Backup_Msg(TELM_SEC_EVT_BATT);
//        save_current_except();
    }
}

// move after ignition off
static void prvGSM_evt_car_theft(int16_t data)
{
    if (1 != GPRS_Get_Telematics_Activation_State())
        return;
    DEBUG_PRINT0(DEBUG_MEDIUM,"[2G] abnormal movement!\n\r");
    if (NET_DATA_TRANS == vATProt_Get_TCP_State())
    {
        vTelmApp_uploadData(TELM_SEC_EVT_TOWING, NULL);
    }
    else
    {
        vTelmApp_Backup_Msg(TELM_SEC_EVT_TOWING);
    }
}

static void prvGSM_evt_gps_upload(int16_t data)
{
    if (1 != GPRS_Get_Telematics_Activation_State())
        return;
//    DEBUG_PRINT0(DEBUG_HIGH,"[2G] upload GPS!\n\r");
    Set_Last_GPS_uploader(0);
    if (NET_DATA_TRANS == vATProt_Get_TCP_State())
    {
	    vTelmApp_uploadData(TELM_SEC_EVT_GPS, NULL);
    }
    else
    {
        //save GPS point
    }
}

static void prvGSM_evt_dev_upload(int16_t data)
{
    vTelmApp_uploadData(TELM_SEC_EVT_DEV, NULL);
}

static void prvGSM_evt_batt_upload(int16_t data)
{
    vTelmApp_uploadData(TELM_SEC_EVT_BATT, NULL);
}

static void prvGSM_evt_env_test(int16_t data)
{
    if (1 != GPRS_Get_Telematics_Activation_State())
        return;
    if (NET_DATA_TRANS == vATProt_Get_TCP_State())
    {
	    vTelmApp_uploadData(TELM_SEC_EVT_BATT, NULL);
    }
    else
    {
    }
}

static void prvGSM_evt_backup_gps_upload(int16_t data)
{
    if (1 != GPRS_Get_Telematics_Activation_State())
        return;
    if (NET_DATA_TRANS == vATProt_Get_TCP_State())
    {
	    vTelmApp_uploadData(TELM_SEC_EVT_BACKUP, NULL);
    }
}

static void prvGSM_evt_config_upload(int16_t data)
{
    if (1 != GPRS_Get_Telematics_Activation_State())
        return;
    if (NET_DATA_TRANS == vATProt_Get_TCP_State())
    {
	    vTelmApp_uploadData(TELM_SEC_EVT_CONFIG, NULL);
    }
}

static void prvGSM_evt_ota(int16_t data)
{
    if (1 != GPRS_Get_Telematics_Activation_State())
        return;
    if (NET_DATA_TRANS == vATProt_Get_TCP_State())
    {
	    vTelmApp_uploadData(TELM_SEC_EVT_OTA, NULL);
    }
}

static void prvGSM_evt_gps_first_fixed(int16_t data)
{
//    Periodic_Get_GPS_Fixed();
    if (1 != GPRS_Get_Telematics_Activation_State())
        return;
    if (NET_DATA_TRANS == vATProt_Get_TCP_State())
    {
	    vTelmApp_uploadData(TELM_SEC_EVT_GPS_FIX, NULL);
    }
    else
    {
        vTelmApp_Backup_Msg(TELM_SEC_EVT_GPS_FIX);
    }
}

static void prvGSM_evt_trip(int16_t data)
{
    if (1 != GPRS_Get_Telematics_Activation_State())
        return;
    // If data==2, trip start; If data==1, trip end
    if (NET_DATA_TRANS == vATProt_Get_TCP_State())
    {
	    vTelmApp_uploadData(TELM_SEC_EVT_TRIP, NULL);
    }
    else
    {
        vTelmApp_Backup_Msg(TELM_SEC_EVT_TRIP);
    }
}

static void prvGSM_evt_dbe(int16_t data)
{
    if (1 != GPRS_Get_Telematics_Activation_State())
        return;
    if (NET_DATA_TRANS == vATProt_Get_TCP_State())
    {
	    vTelmApp_uploadData(TELM_SEC_EVT_DRIVE_BHV, NULL);
    }
    else
    {
        vTelmApp_Backup_Msg(TELM_SEC_EVT_DRIVE_BHV);
    }
}

extern uint8_t GPRS_server_connected(void)
{
    if (NET_TCP_CONNECTED<=vATProt_Get_TCP_State())
        return 1;
    else
        return 0;
}

/*=======================================================================================*\
 * File Revision History
 *=======================================================================================
 * ----------  ------   ---------------------------------------------
 *
\*=======================================================================================*/
