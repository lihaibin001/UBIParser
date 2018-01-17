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
 * Include files                                                       
 *********************************************************************/
#include <standard.h>
#include "ATProtocol.h"
#include "ATApp.h"
#include "TelmProtocol.h"
#include <stdio.h>
#include "uart.h"
#include "GPRS.h"

#ifdef USE_AGPS
#include "gps.h"    //Must put this after 2G.h, beacuse USE_AGPS is defined in 2G.h.
#endif

#include "Debug.h"

/******************************************************************************
* Constant and Macro Definitions using #define
*****************************************************************************/

/*********************************************************************/
/* Enumerations and Structures and Typedefs                          */
/*********************************************************************/

/**********************************************************************
 * Function Prototypes for Private Functions with File Level Scope
 *********************************************************************/

/*********************************************************************/
/* Static Variables and Const Variables With File Level Scope        */
/*********************************************************************/
static callBack				IP_Send_CallBack_Backup;

static Tick_Type Conn_Timeout_timer = MSec_To_Ticks(100000);
static Tick_Type Conn_Timeout;
static uint16_t send_retries = 0;

#ifdef USE_UNICOM_APN_1 //Server, OTA check start after configuration download from server.
//static bool telematics_ota_check_request = false; //OTA check request flag, true: need OTA check.
static bool telematics_ota_check_request = true; //Meituan 2B is the same protocol as 2C, this should be true.
#endif
#ifdef USE_UNICOM_APN_2 //Meituan server, configuration command is not always received ,so initialize the OTA check flag to start OTA check.
static bool telematics_ota_check_request = true; //OTA check request flag, true: need OTA check.
#endif
#ifdef TEST_ON_OWN_SERVER //Test server, configuration command is not always received ,so initialize the OTA check flag to start OTA check.
static bool telematics_ota_check_request = true; //OTA check request flag, true: need OTA check.
#endif
static void vATApp_Poll_OTA_Check(void);
static void prvATApp_Timeout_Reset(void);
static void prvATApp_Check_Timeout(void);
static uint8_t vATApp_Retry_Inc(uint16_t retry_perid);
static void vATApp_Retry_Clear(void);
static void vATApp_Conn_Wait_Timeout(void);

/*******************************************************************************
*    Function:  vATApp_Init
*
*  Parameters:  
*     Returns:  None
* Description:  necessary process when start up.
*******************************************************************************/
extern void vATApp_Init(void)
{
    if (!ATProt_Power_Status())
    {
        UART_Reset_Buf(UART_GSM_CHANNEL);
        vATProt_Power_On();
    }
    prvATApp_Timeout_Reset();
    vATApp_Init_Active_Check();
    vATProt_sendAT_Command(AT_CMD_SET_ECHO, NULL, NULL);
    TMR_Start_Timer(TELM_TCP_TIMER,14000,vATApp_Conn_Wait_Timeout);
    Load_OTAHeader();
}

uint8_t vATApp_Retry_Inc(uint16_t retry_perid)
{
    uint8_t retry_result = 0;

    if (send_retries > 5*retry_perid)
    {
        send_retries = 0;
        vATProt_Com_Reset();
         vATProt_Power_Off();
        vATProt_Set_TCP_State(NET_INIT);
        vATProt_Power_On();
    }
    else if(0 == send_retries %retry_perid)
    {
        retry_result = 1;
    }

    send_retries++;
    return retry_result;
}

void vATApp_Retry_Clear(void)
{
    send_retries = 0;
}

//0: No times out, 1: time out
static uint8_t vATApp_Retry_Check(uint16_t retry_perid)
{
    return (0 == (send_retries++ %retry_perid));
}

void vATApp_Loop_Check(void)
{
    uint8_t net_state = vATProt_Get_TCP_State();

    // ATE0
    // AT+CMGD=1,2
    // AT+CMGF=1
    // AT+CIPHEAD=1
    // Init, wait for call ready
    // AT+CPIN?
    // AT+CGATT?
    // AT+CREG?
    // AT+CSQ
    // Connect
    // AT+CSTT
    // AT+CIICR
    // AT+CIFSR
    // AT+CIPSTART

    switch(net_state)
    {
        case NET_INIT:
            prvATApp_Check_Timeout();
            break;

        case NET_POWER_ON:
            prvATApp_Check_Timeout();
            vATProt_sendAT_Command(AT_CMD_NO_ECHO_SETTING, NULL, NULL);
            vATProt_sendAT_Command(AT_CMD_SMS_FORMAT, NULL, NULL);
            vATProt_sendAT_Command(AT_CMD_CIPHEAD, NULL, NULL);
            vATProt_Set_TCP_State(NET_ECHO_CLOSED);
            break;
        case NET_ECHO_CLOSED:
            prvATApp_Check_Timeout();
            vATProt_sendAT_Command(AT_CMD_GET_IMEI_NO, NULL, NULL);
            vATProt_sendAT_Command(AT_CMD_GET_IMSI_NO, NULL, NULL);
            vATProt_sendAT_Command(AT_CMD_CSQ_QUERY, NULL, NULL);
            vATProt_sendAT_Command(AT_CMD_SET_CREG, NULL, NULL);
            vATProt_Set_TCP_State(NET_SIM_CONNECTED);
            break;
        case NET_SIM_CONNECTED:
            prvATApp_Check_Timeout();
            vATProt_sendAT_Command(AT_CMD_CREG_QUERY, NULL, NULL);
            break;
        case NET_REGISTERED:
            prvATApp_Check_Timeout();
            vATApp_Retry_Clear();
            vATProt_sendAT_Command(AT_CMD_DELETE_SMS, NULL, NULL);
            vATProt_sendAT_Command(AT_CMD_CHECK_GPRS, NULL, NULL);
            vATProt_Set_TCP_State(NET_GPRS_ATTACHED);
            break;
        case NET_GPRS_ATTACHED:
            //Register APN
            vATProt_sendAT_Command(AT_CMD_CGATT, NULL, NULL);
            vATProt_sendAT_Command(AT_CMD_CHECK_GPRS, NULL, NULL);
            prvATApp_Check_Timeout();
#if 0
            vATProt_Set_TCP_State(NET_APN_REGISTERED);
#endif
            break;
        case NET_APN_REGISTERED:
            prvATApp_Check_Timeout();
            vATProt_sendAT_Command(AT_CMD_IP_INIT_EXCUTE, NULL, NULL);
            vATProt_sendAT_Command(AT_CMD_IP_REG_EXCUTE, NULL, NULL);
            vATProt_Set_TCP_State(NET_GPRS_ACTIVATED);
            break;
        case NET_GPRS_ACTIVATED:
            vATProt_sendAT_Command(AT_CMD_IP_QUERY, NULL, NULL);
            break;
        case NET_IP_GOT:
            vATProt_sendAT_Command(AT_CMD_IP_OPEN_EXCUTE, NULL, NULL);
            break;
        case NET_TCP_CONNECTED:
            /* Check if message need resend */
            Conn_Timeout = 0;
            vTelmApp_uploadData(TELM_SEC_EVT_DEV, NULL);
            vATProt_Set_TCP_State(NET_DATA_TRANS);
            break;
        case NET_DATA_TRANS:
            break;
    }
}

static void prvATApp_Timeout_Reset(void)
{
    Conn_Timeout = OS_Time()+Conn_Timeout_timer;
}

static void prvATApp_Check_Timeout(void)
{
    if ((Conn_Timeout < OS_Time()) && (Conn_Timeout != 0))
    {
        Conn_Timeout = 0;
        vATProt_Com_Reset();
        vATProt_Power_Off();
        rl_delay_without_schedule(1000);
        vATProt_Set_TCP_State(NET_INIT);
        vATProt_Power_On();
        prvATApp_Timeout_Reset();
        vATProt_sendAT_Command(AT_CMD_SET_ECHO, NULL, NULL);
    }
}

static void vATApp_Conn_Wait_Timeout(void)
{
    if (NET_INIT == vATProt_Get_TCP_State())
    {
        vATProt_Set_TCP_State(NET_POWER_ON);
    }
}

/*******************************************************************************
*    Function:  vATApp_GoSleep
*
*  Parameters:  
*     Returns:  None
* Description:  necessary process when going to sleep.
*******************************************************************************/
extern void vATApp_GoSleep(void)
{
}

/*******************************************************************************
*    Function:  vATApp_WakeUp
*
*  Parameters:  
*     Returns:  None
* Description:  necessary process when wake up.
*******************************************************************************/
extern void vATApp_WakeUp(void)
{
	vATApp_Init();
}

/*********************************************************
*Function Name: 	vATApp_IPSEND_Excute
*Prototype: 		vATApp_IPSEND_Excute(void)
*Called by: 		app
*Parameters:		char const* type: "TCP" or "UDP"
*					char const *ip	:destination ip address
*					uint16_t port	:destination port number
*					uint8_t const* ipData:data to be sent
*					uint8_t len		:length of data to be sent
*					callBack cb		:call back when data has been sent
*Returns:			void
*Description:		vATApp_IPOPEN_Query
**********************************************************/
void vATApp_IPSEND_Excute(uint8_t const* ipData,uint16_t len,callBack cb)
{
    /* data is not sent in this function because we need to confirm tcp link status first 
    so store the data and parameters  to be sent to static variable here */
    if (len > TELM_MODULE_MAX_BUFF_LEN)
        len = TELM_MODULE_MAX_BUFF_LEN;
    memcpy(TCP_backup_data.TCP_data, ipData, len);
    TCP_backup_data.TCP_Len = len;

    IP_Send_CallBack_Backup = cb;

    /* get network and tcp link status first */
    /* since the priority of commands below is from high to low,
    ATProtocol.c will automatically execute one by one from the first
    command to last */
    vATProt_sendAT_Command(AT_CMD_IP_SEND_DATA_EXCUTE, NULL, NULL);
}

void vATApp_IPSEND_Ack(uint8_t const* ipData,uint16_t len,callBack cb)
{
    if (len > TELM_MODULE_MAX_BUFF_LEN)
        len = TELM_MODULE_MAX_BUFF_LEN;
    memcpy(TCP_backup_data.Ack_data, ipData, len);
    TCP_backup_data.ACK_Len = len;
    IP_Send_CallBack_Backup = cb;

    /* get network and tcp link status first */
    /* since the priority of commands below is from high to low,
    ATProtocol.c will automatically execute one by one from the first
    command to last */
    vATProt_sendAT_Command(AT_CMD_IP_SEND_ACK_EXCUTE, NULL, NULL);
}

void vATApp_SMS_SEND_Excute(uint8_t const* sms_data, uint16_t len, callBack cb)
{

}

void vATApp_Init_Active_Check(void)
{
    // Default activated
}

void vATApp_Init_Deactive_Check(void)
{

}

void vATApp_Sent_Callback(void)
{
    if (IP_Send_CallBack_Backup != NULL)
        IP_Send_CallBack_Backup(AT_CMD_RES_OK);
}

static void vATApp_Poll_OTA_Check(void)
{
    //workaround by Chuanji:trigger only once OTA check message after config command from server
    //OTA check condition: Activated && PS STATE > PS_AWAKE && NO_Upgrading && OTA_Check_request
    if((1 == GPRS_Get_Telematics_Activation_State())&& (PS_Full_System())
        && (0 == GPRS_Get_FMUpgrade_state())&&(true == telematics_ota_check_request))
    {
        vTelmApp_uploadData(TELM_SEC_EVT_OTACHECK, NULL);
        /* Check if voltage buffer full */
        batt_volt_nv_check_tx();
        telematics_ota_check_request = false;
    }
}

void vATApp_Restart_OTA_Check(void)
{
    //Set OTA checked flag to restart OTA check command.
    telematics_ota_check_request = true;
}


/*=======================================================================================*\
 * File Revision History
 *=======================================================================================
 * ----------  ------   ---------------------------------------------
 *
\*=======================================================================================*/
