#ifndef _GPRS_H_
#define _GPRS_H_
/**********************************************************************
   Title                                   : GPRS.h         
                                                                         
   Module Description         : Telematics module header file. used for Telematic internal 
                                                function and external module.

   Author                               : 
   
 *********************************************************************/

/**********************************************************************
 * Include header files
 *********************************************************************/
/**********************************************************************
 * Constant and Macro Definitions using #define
 *********************************************************************/

/**************** GSM functional macro definition start**********************/

//#define TCOM_DATA_DES_ENCDEC   //Define it if use DES encryption-decryption
#define TCOM_SUPPORT_SMS
//#define TCOM_ANSWER_CALL

//#define USE_AGPS    //Define it if use AGPS.
#define USE_FIX_SERVER_IP    //Define it if the server's IP address is fixed. Else the address can be configured and saved in SPI flash.
/**************** 3G functional macro definition end**********************/

//#define CONFIG_TEST_SERVER

//#define USE_DOMAIN_NAME     //Default setting
#define USE_UNICOM_APN_1    //Default setting
//#define USE_SIMCARD_IMSI

//#define DEVICE_ACTIVATED    // Device always activated

#define MAX_RESPONSE_CHAR_LENGTH		(100)

//#define IP_DATA_LEN						(240)

#define TELM_MODULE_MAX_BUFF_LEN (1400)

//#define DEVICE_DEFAULT_SN "P006000300000003"

/**********************************************************************
 * Global Enumerations and Structures and Typedefs
 *********************************************************************/

typedef enum tm_event_tag
{
   TM_EVT_NOP,
   TM_EVT_RESET_MODULE,
   TM_EVT_CAR_CRASHED,
   TM_EVT_BATTERY_FAULT,
   TM_EVT_CAR_THEFT,
   TM_EVT_GPS_UPLOAD,
   TM_EVT_DEV_UPLOAD,
   TM_EVT_BATT_UPLOAD,
   TM_EVT_ENV_TEST,
   TM_EVT_BACKUP_GPS_UP,
   TM_EVT_CONFIG_UPLOAD,
   TM_EVT_OTA,
   TM_EVT_GPS_FIRST,
   TM_EVT_TRIP,
   TM_EVT_DBE,
   TM_NUM_EVENTS
}tm_event_t;

typedef enum AT_Cmd_Result_tag
{
	AT_CMD_RES_NONE,
	AT_CMD_RES_OK,
	AT_CMD_RES_ERR,
	AT_CMD_RES_CME_ERR,
	AT_CMD_RES_NUM
}AT_Cmd_Res;

typedef void (*callBack)(AT_Cmd_Res res);			// call back when finished an AT command

/**********************************************************************
 * Global Function Prototypes
 *********************************************************************/
extern void GPRS_Set_FMUpgrade_state(uint8_t fmupgrade_state);
extern bool GPRS_Get_FMUpgrade_state(void);

extern void IOT_Task(void* pvParameters);
extern void GPRS_Module_GoSleep(void);
void GPRS_Set_Telematics_Activation_State(uint8_t act_flag);
uint8_t GPRS_Get_Telematics_Activation_State(void);
void GPRS_Set_Telematics_Module_Pwr_State(bool pwr_flag);
bool GPRS_Get_Telematics_Module_Pwr_State(void);

extern void get_config(char *data);
extern uint8_t module_alive(void);
extern uint8_t GPRS_server_connected(void);

/*=======================================================================================*\
 * File Revision History
 *=======================================================================================
 * ----------  ------   ---------------------------------------------
 *
\*=======================================================================================*/
#endif
