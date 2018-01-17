#ifndef _TELM_APP_H_
#define _TELM_APP_H_
/**********************************************************************
   Title                                   : TelmApp.h         
                                                                         
   Module Description         : Telematics module header file. used for Telematic internal 
                                                function and external module.

   Author                               : 
   
 *********************************************************************/

/**********************************************************************
 * Include header files
 *********************************************************************/

#define TELM_APP_LEN_LON			(16)

typedef enum Telm_Info_Tag
{
    TELM_INFO_NONE,
	TELM_INFO_DEV,	/* Device information 					*/
	TELM_INFO_DRIVE_BHV,	/* Car drive behavior 					*/
	TELM_INFO_OTA,	/* OTA request 					*/
	TELM_INFO_OTA_CHECK,	/* OTA trigger 					*/
	TELM_INFO_GPS,	/* Position and speed related information	*/
    TELM_INFO_BACKUP_GPS, /* Backup GPS data in Flash */
	TELM_INFO_TRIP,	/* Trip data data 					*/
	TELM_INFO_CELL_LOC,	/* Cell location data 					*/
	TELM_INFO_BATT,	/* Battery sleep voltage 					*/
	TELM_INFO_TOWING,	/* Towing data 					*/
    TELM_INFO_ACTIVATION, /* Activation status */
    TELM_INFO_SIM, /* SIM IMSI */
    TELM_INFO_REMOVAL, /* Remove alarm */
    TELM_INFO_CONFIG, /* Config */
    TELM_INFO_GPS_FIX, /* GPS first fix time */
	TELM_INFO_CRASH,	/* Crash information 					*/
	TELM_INFO_SENSOR,	/* Sensor data		*/
	TELM_INFO_DASHBOARD,	/* Acc status		*/

    TELM_INFO_NUM
}Telm_info;

/* define the situation in which MINIX should upload infomation to server */
typedef enum Telm_Security_Event_Tag
{
	TELM_SEC_EVT_NONE,
	TELM_SEC_EVT_DEV,		/* Device ID					*/
	TELM_SEC_EVT_OTACHECK,		/* Check OTA condition					*/
	TELM_SEC_EVT_OTA,		/* OTA request					*/
	TELM_SEC_EVT_DRIVE_BHV,		/* car drive behavior					*/
	TELM_SEC_EVT_BACKUP,		/* Backup GPS data upload					*/
	TELM_SEC_EVT_GPS,		/* normal drive					*/
	TELM_SEC_EVT_TOWING,		/* Towing event messages					*/
	TELM_SEC_EVT_BATT,		/* Battery sleep voltage 					*/
	TELM_SEC_EVT_TRAVEL_SUMMARY,		/* Upload travel summary info 					*/
	TELM_SEC_EVT_ACTIVATE,		/* Activation status upload					*/
	TELM_SEC_EVT_REMOVAL,		/* Remove alarm				*/
	TELM_SEC_EVT_CONFIG,		/* Config */
	TELM_SEC_EVT_GPS_FIX,		/* GPS first fix time */
	TELM_SEC_EVT_TRIP,		/* Trip event, start or stopped */
	TELM_SEC_EVT_CRASH,		/* Send Gsensor data					*/
	TELM_SEC_EVT_SENSOR,		/* Send Sensor data	package		*/

	TELM_SEC_EVT_NUM
}Telm_Security_Event;

/* call back function type to define the process after data is uploaded to server */
typedef void (*notifyResult)(bool result);

typedef enum Telm_Comm_Tag
{
    TELM_COMM_NONE,
    TELM_COMM_THEFT,				/* vehicle theft detection command						*/
    TELM_COMM_FMC,					/* vehicle fmc command									*/
    TELM_COMM_GPS,					/* vehicle get gps command								*/
    TELM_COMM_HVAC,					/* vehicle hvac command									*/
    TELM_COMM_START,				/* vehicle start command								*/
    TELM_COMM_DRWN,					/* vehicle door&window command							*/
    TELM_COMM_SPEED,				/* vehicle speed control command						*/
    TELM_COMM_LAMP,					/* vehicle lamp control command							*/
    TELM_COMM_GET_STATUS,         /* vehicle get all status command                    */
    TELM_COMM_OTA,					/* device OTA command							*/
    TELM_COMM_CONFIG, /* device config command */
    TELM_COMM_SVRCMD, /* server command */
    TELM_COMM_DEV, /* device id command */
    TELM_COMM_PRV, /* private CAN data request */
    TELM_COMM_NUM
}Telm_Comm;

typedef enum Telm_Upload_Result_Tag
{
	TELM_UPLOAD_RES_OK,
	TELM_UPLOAD_RES_NG,
	TELM_UPLOAD_RES_TIMEOUT
}Telm_Upload_Result;

typedef enum Telm_Upload_Err_Code_Tag
{
	TELM_ERROR_STATUS=1,
	TELM_INVALID_DATA,
	TELM_INVALID_FORMAT,
    TELM_WRONG_POWER_MODE,
    TELM_WRONG_CONDITION,
    TELM_OPERATE_FAIL,
    TELM_ACCESS_DENIED,
    TELM_EXPIRED
}Telm_Err_Code;

/* struct to store the command recieved from server */
typedef struct Telm_Rcvd_Command_tag
{
	Telm_Comm 			commType;
	notifyResult		cmdExecuted;
	bool				result;
//	uint16_t			speed;
}Telm_Rcvd_Command;

/* app status */
typedef enum Telm_State_Tag
{
	TELM_STATE_IDLE,			/* IDLE 							*/
	TELM_STATE_BUSY,			/* BUSY 							*/
	TELM_STATE_NUM
}Telm_State;
typedef enum Telm_Queue_Handler_tag
{
	TELM_QUEUE_UPLOAD,
	TELM_QUEUE_COMMAND,
	TELM_QUEUE_NUM
}Telm_Queue_Handler;

typedef struct Gps_Store_Type_tag
{
    uint8_t longitude[TELM_APP_LEN_LON];
    uint8_t latitude[TELM_APP_LEN_LON];
    uint8_t utc_time[14];
    uint8_t speed;
    uint8_t engine_rpm[2];
    uint8_t valid;
    uint8_t temp;
    uint8_t intake_map;
    uint8_t intake_temp;
    uint8_t throttle;
}Gps_Store_Type_t;

typedef struct Engine_Backup_tag
{
    uint8_t engine_on;
    uint8_t batt_val;
    uint8_t odo[4];
    uint8_t fuel_used[4];
    uint8_t utc[4];
}Engine_Backup_t;

extern Telm_State eTelmApp_upload_state(void);
extern void vTelmApp_AT_Var_Reset(void);
extern uint8_t vTelmApp_uploadData(Telm_Security_Event evt, notifyResult uploaded);
extern void vTelmApp_Inform_Upload_Result(Telm_Upload_Result res);
extern void vTelmApp_Inform_Received_Command(Telm_Rcvd_Command* rcvdCmd);
extern uint8_t vTelmApp_Backup_Msg(Telm_Security_Event evt);
extern uint8_t vTelmApp_Upload_Backup(void);
extern uint8_t prvTelmApp_Queue_Query(Telm_Security_Event event);
extern void TelmApp_Set_Server_Ack_State(bool ack_or_nak);
extern void vTelmApp_main_loop(void);

extern void eng_on_off_nv_read_utc(uint8_t *data);
extern uint8_t eng_on_off_nv_read(uint8_t *data);
extern void eng_on_off_nv_load_info(void);
extern void eng_on_off_nv_write_info(void);
extern void eng_on_off_backup_msg_cb(void);

extern void eng_on_off_ram_read_utc(uint8_t *data);
extern uint8_t eng_on_off_ram_read(uint8_t *data);

extern void batt_volt_nv_read(uint8_t *data);
/*=======================================================================================*\
 * File Revision History
 *=======================================================================================
 * ----------  ------   ---------------------------------------------
 *
\*=======================================================================================*/
#endif	/* _TELM_APP_H_ */

