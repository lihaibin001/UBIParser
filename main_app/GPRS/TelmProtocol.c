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
#include "standard.h"
#include "TelmProtocol.h"
#include "GPRS.h"
#include "ATProtocol.h"
#include "ATApp.h"
#include <stdio.h>

#include "gps.h"
//#include "Gsensor.h"
#include "crc_ccitt.h"

#define USE_DEBUG
#include "Debug.h"

/******************************************************************************
* Constant and Macro Definitions using #define
*****************************************************************************/
/* length definition */
#define	TELM_INFO_LEN_ACK			(3)

//#define	TELM_INFO_LEN_DATALK			(2)

#define	TELM_INFO_LEN_DEV		(34)
#define	TELM_INFO_LEN_DRIVE		(26)
#define	TELM_INFO_LEN_OTA		(13)
#define TELM_INFO_LEN_BATT (2)
#define TELM_INFO_LEN_SIM (8)
#define TELM_INFO_LEN_REMOVAL (20)
#define TELM_INFO_LEN_GPS_FIX (2)
#define TELM_INFO_LEN_TRIP (25)
#define TELM_INFO_LEN_CRASH (6)
#define TELM_INFO_LEN_DRIVING_BHV (6)
#define TELM_INFO_LEN_SENSOR (35)
#define TELM_INFO_LEN_DASHBOARD (3)
#define TELM_INFO_LEN_TOWING (24)

#define TELM_COMM_LEN_DATA_MAX		(7)
//#define	TELM_COMM_LEN_HEAD			(TELM_INFO_LEN_VIN + TELM_INFO_LEN_UTC + 2) //

/*DES padding value*/
#define TELM_INFO_DES_PACK_LEN	    (8)
#define TELM_INFO_CS_LEN	        (2)
#define TELM_INFO_PADDING			(0x55)

#define	TELM_ACK_RCV_OK				(0x30)
#define	TELM_ACK_RCV_NG				(0x31)
#define	TELM_ACK_EXE_OK				(0x32)
#define	TELM_ACK_EXE_NG				(0x33)

#define	TELM_INFO_ID_DEV			(0x03)
#define	TELM_INFO_ID_CONFIG			(0x04)
#define	TELM_INFO_ID_PWR			(0x05)
#define	TELM_INFO_ID_ACTIVATE		(0x06)
#define	TELM_INFO_ID_GPS			(0x07)
#define	TELM_INFO_ID_RESET			(0x08)
#define	TELM_INFO_ID_CELLPOS		(0x09)
#define	TELM_INFO_ID_TOWING		    (0x2C)
#define	TELM_INFO_ID_REMOVAL		(0x2D)
#define	TELM_INFO_ID_GPS_FIX		(0x2E)
#define	TELM_INFO_ID_DASHBOARD		(0x33)
#define	TELM_INFO_ID_TRIPSUM		(0x3D)
#define	TELM_INFO_ID_SIM    		(0x40)
#define	TELM_INFO_ID_OTA    		(0x43)
#define	TELM_INFO_ID_CRASH    		(0x47)
#define	TELM_INFO_ID_DRVBHV    		(0x48)
#define	TELM_INFO_ID_SENSOR    		(0x49)
#define	TELM_INFO_ID_ERROR    		(0x63)

#define	TELM_COMM_ID_OTA			(0x43)
#define	TELM_COMM_ID_DEV			(0x03)
#define	TELM_COMM_ID_CONFIG			(0x04)
#define	TELM_COMM_ID_BATT			(0x05)
#define	TELM_COMM_ID_ACTIVATE		(0x06)
#define	TELM_COMM_ID_GPS			(0x07)
#define	TELM_COMM_ID_RESET			(0x08)

#define	TELM_INFO_CMD_REQ           (0x01)
#define	TELM_INFO_CMD_RESP          (0x02)
#define	TELM_INFO_CMD_EXEC          (0x04)

#define CONFIG_ID_DEV           		(0x01)
#define CONFIG_ID_ACC           		(0x02)
#define CONFIG_ID_WAKE          		(0x03)
#define CONFIG_ID_CRASH         		(0x04)
#define CONFIG_ID_DRIVE         		(0x05)
#define CONFIG_ID_GPS           		(0x06)
#define CONFIG_ID_LOG           		(0x07)
#define CONFIG_ID_SLEEP         		(0x08)
#define CONFIG_ID_BATT          		(0x09)
#define CONFIG_ID_THEFT   			    (0x0A)
#define CONFIG_ID_TRACKING    		 	(0x0B)
#define CONFIG_ID_NETWORK      			(0x0C)
#define CONFIG_IP_WAKEUP_REGULARLY		(0x0D)  /*2917/2/14 LiHaibin add*/

#define OTA_MAX_LEN (258048)

#define MAX_SUPPORTED_PID_PER_PACK (26)

#define SPEED_DETECT_INTERVAL (6) //3S is 6 times.
#define GPS_UPLOAD_MIN_INTERVAL (3)

#define GPS_UPLOAD_BUFFER_MAX (7)
#define MAX_BACKUP_GPS_ONE_PACK (7)

#define MAX_GSENSOR_DATA_LEN (192)
#define MAX_HIGH_G_LEN (96)

#define MAX_EXCEPT_RECORD_BACKUP (70)

#define THEFT_DETECT_VOLT_LOW (115)
#define TELM_GTIMES_UPLOAD_LIMIT (5)

#define HEADER_STX (0x55)
#define HEADER_IMEI_FLAG    (1<<7)
#define HEADER_TS_FLAG      (1<<6)
#define HEADER_MSGTTL_FLAG  (1<<5)
#define HEADER_VER          (0x7<<2)
#define HEADER_CHK_FLAG     (1<<1)

#define MAX_BACKUP_GPS_UPLOAD_NUM (5)

#define RESET_BIT_CONFIG    (1<<4)
#define RESET_BIT_SENSOR    (1<<3)
#define RESET_BIT_GSM       (1<<2)
#define RESET_BIT_GPS       (1<<1)
#define RESET_BIT_MCU       (0x1)


/*********************************************************************/
/* Enumerations and Structures and Typedefs                          */
/*********************************************************************/
/* protocol define */


/* Device info */
typedef union Telm_data_dev_tag
{
	uint8_t byte[TELM_INFO_LEN_DEV];
	struct {
		uint8_t imei[8];
		uint8_t dev_type;
		uint8_t hw_ver[4];
		uint8_t prtocol_ver;
		uint8_t sw_ver[4];
		uint8_t lac[2];
		uint8_t cell_id[2];
		uint8_t csq;
		uint8_t netreg;
		uint8_t phone_type[10];
	} structData;
} Telm_Data_Dev;

/* Drive info */
typedef union Telm_data_gps_tag
{
	uint8_t byte[TELM_INFO_LEN_DRIVE];
	struct {
        uint8_t timestamp[4];
		uint8_t latitude[4];
		uint8_t longitude[4];
		uint8_t altitude[4];
        uint8_t valid_flag;
        uint8_t cog[2];
        uint8_t speed;
        uint8_t used_sat_num;
        uint8_t pdop;
        uint8_t lac[2];
        uint8_t cell_id[2];
	} structData;
} Telm_Data_GPS;

/* Exception */

/* OTA command */
typedef union Telm_data_ota_tag
{
	uint8_t byte[TELM_INFO_LEN_OTA];
	struct {
        uint8_t ota_status;
        uint8_t target_ver[4];
        uint8_t download_index[4];
        uint8_t download_bytes[4];
	} structData;
} Telm_Data_Ota;

/* OTA check */


typedef union Telm_data_batt_tag
{
	uint8_t byte[TELM_INFO_LEN_BATT];
	struct {
            uint8_t power_status;
            uint8_t voltage;
	} structData;
}Telm_Data_Batt;

typedef union Telm_data_trip_tag
{
	uint8_t byte[TELM_INFO_LEN_TRIP];
	struct {
        uint8_t trip_status;
        uint8_t trip_start_time[4];
        uint8_t trip_start_odo[4];
        uint8_t trip_mileage[4];
        uint8_t avg_speed;
        uint8_t trip_end_time[4];
        uint8_t trip_end_odo[4];
        uint8_t accel_num;
        uint8_t brake_num;
        uint8_t fturn_num;
	} structData;
}Telm_Data_Trip;

typedef union Telm_data_crash_tag
{
	uint8_t byte[TELM_INFO_LEN_CRASH];
	struct {
        uint8_t crash_id;
        uint8_t crash_time[4];
        uint8_t crash_gps_num;
	} structData;
}Telm_Data_Crash;

typedef union Telm_data_driving_bhv_tag
{
	uint8_t byte[TELM_INFO_LEN_DRIVING_BHV];
	struct {
        uint8_t bhv_time[4];
        uint8_t bhv_gps_num;
	} structData;
}Telm_Data_Driving_Bhv;

typedef union Telm_data_sensor_tag
{
	uint8_t byte[TELM_INFO_LEN_SENSOR];
	struct {
        uint8_t sensor_type;
        uint8_t timestamp[4];
		uint8_t latitude[4];
		uint8_t longitude[4];
		uint8_t altitude[4];
        uint8_t valid_flag;
        uint8_t cog[2];
        uint8_t speed;
        uint8_t used_sat_num;
        uint8_t pdop;
        uint8_t lac[2];
        uint8_t cell_id[2];
        uint8_t capture_time[4];
        uint8_t capture_duration;
        uint8_t package_num;
        uint8_t sensor_setting;
	} structData;
}Telm_Data_Sensor;

typedef union Telm_data_dashboard_tag
{
	uint8_t byte[TELM_INFO_LEN_DASHBOARD];
	struct {
        uint8_t acc_status;
        uint8_t window;
        uint8_t door;
	} structData;
}Telm_Data_Dashboard;

typedef union Telm_data_removal_tag
{
	uint8_t byte[TELM_INFO_LEN_REMOVAL];
	struct {
        uint8_t timestamp[4];
        uint8_t latitude[4];
        uint8_t longitude[4];
        uint8_t altitude[4];
        uint8_t valid;
        uint8_t external_voltage[2];
        uint8_t internal_voltage;
	} structData;
}Telm_Data_Removal;

typedef union Telm_data_gps_fix_tag
{
	uint8_t byte[TELM_INFO_LEN_GPS_FIX];
	struct {
        uint8_t fix_time[2];
	} structData;
}Telm_Data_GPS_Fix;

typedef union Telm_data_towing_tag
{
	uint8_t byte[TELM_INFO_LEN_TOWING];
	struct {
        uint8_t timestamp[4];
        uint8_t latitude[4];
        uint8_t longitude[4];
        uint8_t altitude[4];
        uint8_t valid;
        uint8_t heading[2];
        uint8_t speed;
        uint8_t lac[2];
        uint8_t cell_id[2];
	} structData;
}Telm_Data_Towing;


/* Server Command Head */
typedef struct Telm_command_normal_struct_tag
{
   uint8_t devId[TELM_INFO_LEN_DEVID];
   uint8_t server_utc_time[TELM_INFO_LEN_UTC];
   uint8_t cmdID;
   uint8_t cmdLen;
   uint8_t byte[TELM_COMM_LEN_DATA_MAX];
} Telm_Command_Normal_Struct;

/* Server Command */
typedef union Telm_command_normal_tag
{
	Telm_Command_Normal_Struct	normal;
} Telm_Command;

/* telmatics info encode function type */
typedef uint16_t (*Telm_Info_Encode_Fun)(Telm_Upload_Info const *uploadInfo, uint8_t multiple, uint8_t *const encoded, uint16_t bufSize);

typedef struct InitFlag_Sct_tag
{
    uint8_t init_flag[4];
}InitFlag_Sct_t;

typedef struct Gps_Backup_Ctrl_tag
{
    uint8_t store_pos;
    uint8_t read_pos;
}Gps_Backup_Ctrl_t;


typedef struct Car_Status_Except_tag
{
    uint8_t upload;
    uint8_t status;
}Car_Status_Except_t;


/**********************************************************************
 * Function Prototypes for Private Functions with File Level Scope
 *********************************************************************/
/* preprocess for recieved data */
static uint16_t prvTelmProt_Decode_Remap(uint8_t * const buf,uint16_t* len);
static uint16_t	prvTelmProt_getPackageLen(uint8_t *data,uint16_t len);
/* sequence control of protocol level */
static void prvTelmProt_Return_Result(bool result);
/* function of encode */
static uint16_t prvTelmProt_Encode(Telm_Upload_Info_Request const* uploadReq, Telm_Upload_Info const * uploadInfo, uint8_t * const data, uint16_t bufSize);
static uint8_t prvTelmProt_Encode_Header(Telm_Upload_Info const * uploadInfo, uint8_t* const encoded, uint16_t len);
static uint8_t prvTelmProt_Encode_Chk(uint8_t *checksum, uint8_t *const encoded, uint16_t size);

static uint16_t prvTelmProt_Encode_Dev(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize);
static uint16_t prvTelmProt_Encode_Drive_Behavior(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize);
static uint16_t prvTelmProt_Encode_GPS(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize);
static uint16_t prvTelmProt_Encode_Get_OTA(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize);
static uint16_t prvTelmProt_Encode_Backup_GPS(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize);
static uint16_t prvTelmProt_Encode_Ota_Check(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize);
static uint16_t prvTelmProt_Encode_Cell_Loc(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize);
static uint16_t prvTelmProt_Encode_Batt(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize);
static uint16_t prvTelmProt_Encode_Trip(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize);
static uint16_t prvTelmProt_Encode_Towing(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize);
static uint16_t prvTelmProt_Encode_Activation(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize);
static uint16_t prvTelmProt_Encode_Sim(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize);
static uint16_t prvTelmProt_Encode_Removal(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize);
static uint16_t prvTelmProt_Encode_Config(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize);
static uint16_t prvTelmProt_Encode_GPS_Fix(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize);
static uint16_t prvTelmProt_Encode_Crash(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize);
static uint16_t prvTelmProt_Encode_Sensor(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize);
static uint16_t prvTelmProt_Encode_Dashboard(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize);

static uint16_t prvTelmProt_Encode_Remap(uint8_t * const buff,uint16_t len, uint16_t bufSize);
#ifdef TCOM_DATA_DES_ENCDEC
static uint8_t prvTelmProt_DES_EncPadding(uint8_t * const buf,const uint8_t len);
static void prvTelmProt_DES_Dec(uint8_t * const buf,const uint16_t len);
#endif
/* function of decode */
static bool prvTelmProt_parse_header(uint8_t const * const data, uint8_t *service_data, uint16_t *data_len);
static void prvTelmProt_DecodeAndHandleCmd(uint8_t *data, uint16_t len);
static void prvTelmProt_DecodeAndHandleAck(uint8_t *data);
static bool prvTelmProt_Decode_Command(uint8_t const * const data, uint16_t len, Telm_Rcvd_Command * const decoded);
static void prvTelmProt_commandAckSend(AT_Cmd_Res res);
static void Update_OTAHeader(void);

static void TelmProt_Get_Travel_Summary(Telm_Data_Trip *data);

/**********************************************************************
 * Variables with File Level Scope
 *********************************************************************/
/* variable to store hdr of last command which is needed 
	to send back the ack to server  */
static uint8_t  telm_last_hdr;
static uint8_t	ipData[TCP_TX_MAX_LEN];
static uint16_t	dataLen;
static Telm_Rcvd_Command telm_last_command;
static uint8_t vehicle_speed = 0;

static uint8_t speed_buf[10];
uint8_t speed_buf_start = 0;
uint8_t speed_buf_tail = 0;
static uint8_t speed_abnormal[3];

#ifdef TCOM_DATA_DES_ENCDEC
uint8_t enc_data[8];
uint8_t enc_cipher_data[8];
uint8_t enc_dencrypt_data[8];
uint8_t enc_key_made; // If the byte is 1: it means the enc key has made ok.
uint8_t enc_key_56bits[56];
uint8_t enc_sub_keys[16][48];
#endif


static OTAHdr_Sct_t ota_header_info;

__no_init uint8_t car_battery_status ; /* 0=normal, 1=car battery low,2=warning message is sent*/

static uint16_t config_upload_id=1;
static uint8_t ota_packet_flag = 0;

/**********************************************************************
 * Private variable with File Level Scope
 *********************************************************************/
Telm_Upload_Info			lastInfo;

static const Telm_Info_Encode_Fun telm_info_encode_table[] =
{
    prvTelmProt_Encode_Dev,       /* Device information 					*/
    prvTelmProt_Encode_Drive_Behavior,       /* Drive Behavior information 					*/
    prvTelmProt_Encode_Get_OTA, /* OTA request */
    prvTelmProt_Encode_Ota_Check, /* OTA check trigger */
    prvTelmProt_Encode_GPS,       /* GPS information 					*/
    prvTelmProt_Encode_Backup_GPS,       /* Backup GPS information 					*/
    prvTelmProt_Encode_Trip, /* Travel summary message */
    prvTelmProt_Encode_Cell_Loc, /* Cell location data message */
    prvTelmProt_Encode_Batt, /* Sleep battery data message */
	prvTelmProt_Encode_Towing,	/* Towing data 					*/
    prvTelmProt_Encode_Activation, /* Activation status */
    prvTelmProt_Encode_Sim, /* SIM imsi */
    prvTelmProt_Encode_Removal, /* Remove alarm */
    prvTelmProt_Encode_Config, /* Config data */
    prvTelmProt_Encode_GPS_Fix, /* GPS first fix time */
    prvTelmProt_Encode_Crash, /* Crash information 		*/
    prvTelmProt_Encode_Sensor, /* Sensor data		*/
    prvTelmProt_Encode_Dashboard, /* Acc status		*/
};

/*******************************************************************************
*    Function: vTelmProt_UploadInfo
*
*  Parameters: Telm_Upload_Info_Request : data upload request
*  Parameters: Telm_Upload_Info : data to upload
*     Returns: true:start send process, false: decode error, do not upload data.
* Description: interface for telematics app to send the upload command
*******************************************************************************/
extern bool vTelmProt_UploadInfo(Telm_Upload_Info_Request const * const req,Telm_Upload_Info const * const info)
{
    /* clear last sent data */
    memset(ipData, 0x00, sizeof(ipData));
    dataLen = 0;
    /* encode information to byte array according to protocol */
    dataLen = prvTelmProt_Encode(req,info,ipData,TCP_TX_MAX_LEN);
    /* send data by tcp/ip to server */

    /*Encoded data check*/
    if (dataLen > TCP_TX_MAX_LEN)
    {
        DEBUG_PRINT1(DEBUG_HIGH, "[2G]: Exceed max send length [%d]!\n\r", dataLen);
        return false;
    }
    else if (dataLen == 0)
    {
        DEBUG_PRINT0(DEBUG_HIGH,"[2G]: empty message sent!\n\r");
        return false;
    }
    else
    {
        vATApp_IPSEND_Excute(ipData,dataLen, NULL);
    }
    return true;
}

/*******************************************************************************
 *    Function: vTelmProt_informData
 *
 *  Parameters: uint8_t : recieved data
 *  Parameters: len : length of recieved data
 *     Returns: None
 * Description: interface for network tranciever modual to inform the arrival of
 *				new data
 *******************************************************************************/
extern void	vTelmProt_informData(uint8_t const *data, uint16_t len)
{
    uint16_t dataLen;
    uint8_t srv_data[1100];
    if (false == prvTelmProt_parse_header(data, srv_data, &len))
        return;

    while (len > 0)
    {
        /*Get the current packet from the received buffer*/
        dataLen = prvTelmProt_getPackageLen(srv_data, len);
            /* handle command package SERVER_REQ_CMD*/
//            dataLen = prvTelmProt_Decode_Remap((uint8_t *)data, &len);
        prvTelmProt_DecodeAndHandleCmd(srv_data,dataLen-1);
        /* Buffer change to the start of the next packet in recieved data */
        data += dataLen;
        len -= dataLen;
    }
}

/*******************************************************************************
 *    Function:  prvTelmProt_Decode_Remap
 *
 *  Parameters:  uint8_t * const:buffer to remap
 *  Parameters:  uint8_t :length to remap
 *  Parameters:  uint8_t :max length of buffer
 *     Returns:  None
 * Description:  
 *******************************************************************************/
static uint16_t prvTelmProt_Decode_Remap(uint8_t * const buf,uint16_t *len)
{
    uint16_t i = 0;
    return i;
}

/*******************************************************************************
 *    Function: prvTelmProt_getPackageLen
 *
 *  Parameters: uint8_t : recieved data
 *  Parameters: len : length of recieved data
 *     Returns: None
 * Description: calculate the length of recieved package by searching 0x1C
 *******************************************************************************/
static uint16_t	prvTelmProt_getPackageLen(uint8_t *data, uint16_t len)
{
    uint16_t ret = 0;
    uint8_t ext_bit=*(data+2)&0x1;
    if (ext_bit)
    {
        ret = (*(data+2)>>1) + (*(data+3)<<7);
        ret+=4;
    }
    else
    {
        ret = (*(data+2)>>1);
        ret+=3;
    }

    return ret;
}

void Clear_OTASector(void)
{
    sFLASH_EraseSector(OTA_LIVE_HEADER_ADDR);
}

void Load_OTAHeader(void)
{
    sFLASH_ReadBuffer(ota_header_info.byte, OTA_LIVE_HEADER_ADDR, sizeof(ota_header_info));
}

static void Update_OTAHeader(void)
{
    Clear_OTASector();
    sFLASH_WriteBuffer(ota_header_info.byte, OTA_LIVE_HEADER_ADDR, sizeof(ota_header_info));
}

static void prvTelmProt_ParseConfig(uint8_t *data, uint16_t len)
{
    uint8_t config_ver[2];
    uint16_t config_id=(*(data+2)<<8)+(*(data+3));
    uint8_t new_config_flag=0;
    static Config_t cfg;
    Get_config_data(&cfg);
    config_ver[0]=*data;
    config_ver[1]=(*(data+1));
    DEBUG_PRINT0(DEBUG_MEDIUM,"[IOT]: Get Configure info!\n\r");
    // check config id
    if ((cfg.structData.config_ver[0]!= config_ver[0]) || (cfg.structData.config_ver[1]!= config_ver[1]))
    {
        DEBUG_PRINT0(DEBUG_MEDIUM,"[IOT]: New config version!\n\r");
        new_config_flag=1;
        cfg.structData.config_ver[0]=config_ver[0];
        cfg.structData.config_ver[1]=config_ver[1];
    }
    switch(config_id)
    {
        case CONFIG_ID_DEV:
            break;
        case CONFIG_ID_ACC:
            break;
        case CONFIG_ID_WAKE:
            if (len<7)
                return;
            cfg.structData.wakeup_lasting_time[0]=*(data+4);
            cfg.structData.wakeup_lasting_time[1]=*(data+5);
            cfg.structData.data_resend_times=*(data+6);
            break;
        case CONFIG_ID_CRASH:
            break;
        case CONFIG_ID_DRIVE:
            break;
        case CONFIG_ID_GPS:
            if (len<9)
                return;
            cfg.structData.GPS_upload_interval=*(data+4);
            cfg.structData.GPS_record_mileage[0]=*(data+5);
            cfg.structData.GPS_record_mileage[1]=*(data+6);
            cfg.structData.GPS_record_interval=*(data+7);
            cfg.structData.GPS_record_turning=*(data+8);
            break;
        case CONFIG_ID_LOG:
            break;
        case CONFIG_ID_SLEEP:
            if (len<7)
                return;
            cfg.structData.sleep_time[0]=*(data+4);
            cfg.structData.sleep_time[1]=*(data+5);
            cfg.structData.network_fail_sleep_time=*(data+6);
            break;
        case CONFIG_ID_BATT:
            if (len<6)
                return;
            cfg.structData.external_low_voltage_threshold=*(data+4);
            cfg.structData.internal_low_voltage_threshold=*(data+5);
            break;
        case CONFIG_ID_THEFT:
            break;
        case CONFIG_ID_TRACKING:
            if (len<8)
                return;
            cfg.structData.tracking_mode_on_time[0]=*(data+4);
            cfg.structData.tracking_mode_on_time[1]=*(data+5);
            cfg.structData.tracking_mode_sleep_time[0]=*(data+6);
            cfg.structData.tracking_mode_sleep_time[1]=*(data+7);
            break;
        case CONFIG_ID_NETWORK:
            if (len<42)
                return;
            cfg.structData.network_connection_retain_time=*(data+4);
            memcpy(cfg.structData.network_connection_port,data+5,2);
            memcpy(cfg.structData.network_connection_IP,data+7,16);
            memcpy(cfg.structData.network_connection_APN,data+23,20);
            break;
		/******************
		 * 2017/2/14
		 * LiHaibin modify
		 * Add wake up regularly mode*/
        case CONFIG_IP_WAKEUP_REGULARLY:
        	if(len < 3){
        		return;
        	}
			if(((uint32_t)(*(data + 5) & 0x000000FF ) <= 23) && \
					((uint32_t)(*(data + 6) & 0x000000FF) <= 59))
            { //verify time format
				//time legitimacy
	        	cfg.structData.wakeup_type = *(data + 4); //extract wake up mode
	        	cfg.structData.wakeup_timeHour = *(data + 5); //extract wake up time hour value.
	        	cfg.structData.wakeup_timeMinute = *(data + 6);
			}

        	break;
        default:
            break;
    }
    // set config
    if (new_config_flag)
    {
        Set_Config(cfg);
    }
}

static void prvTelmProt_ParseConfigReq(uint8_t *data, uint16_t len)
{
    if (len >=2)
    {
        uint16_t config_id=(*(data)<<8)+(*(data+1));
        config_upload_id=config_id;
        OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_CONFIG_UPLOAD, 1));
    }
}

static void prvTelmProt_ParseOTA(uint8_t *data, uint16_t length)
{
    uint16_t total_num = 0;
    uint16_t cur_num = 0;
    uint32_t data_len = 0;
    uint32_t i=0;
    uint32_t checksum = 0;
    uint32_t tmp_checksum = 0;
    uint8_t  ota_ctl=0;
    uint8_t target_ver[4];
    uint32_t total_bytes = 0;

    uint16_t tmp_num;
    //uint16_t tmp_total;

    ota_ctl=*(data);
    memcpy(target_ver, data+1, 4);
    total_bytes = (*(data+5)<<24)+(*(data+6)<<16)+(*(data+7)<<8)+(*(data+8));
    total_num = (*(data+9)<<8) + (*(data+10));
    cur_num = (*(data+11)<<8) + (*(data+12));
    data_len = (*(data+13)<<8) + (*(data+14));
    DEBUG_PRINT1(DEBUG_MEDIUM,"[2G]: OTA package num [%d]!\n\r", cur_num);
    TelmProt_Set_New_OTA(0);
    tmp_num = (ota_header_info.structData.ota_current_package_index[0]<<8) + (ota_header_info.structData.ota_current_package_index[1]);
    if ((tmp_num + 1 != cur_num) && (0 != cur_num))
    {//the packet is must the (current +1), or 0.
        DEBUG_PRINT1(DEBUG_HIGH, "[2G]: OTA package num is not the expected num [%d] , discard the packet!\n\r",tmp_num + 1);
        return;
    }
    else if(cur_num > total_num)
    {//the packet num is must less than total.
        DEBUG_PRINT1(DEBUG_HIGH, "[2G]: OTA package num is exceed total num [%d] , discard the packet!\n\r",total_num);
        return;
    }
    //Get the ota data length.
    uint32_t len = (ota_header_info.structData.ota_downloaded_bytes[0] << 24) +
                   (ota_header_info.structData.ota_downloaded_bytes[1] << 16) +
                   (ota_header_info.structData.ota_downloaded_bytes[2] << 8) +
                   (ota_header_info.structData.ota_downloaded_bytes[3] << 0);

    if (total_num > 0)
    {
//        if ((ota_header_info.structData.ota_start[0] == (START_OTA_FLAG & 0xff)) && 
//            (ota_header_info.structData.ota_start[1] == ((START_OTA_FLAG>>8) & 0xff)))
        if ((ota_ctl != OTA_FLAG_START) || (cur_num > 0))
        {
            if (cur_num == 0)
            {//Packet 0: start to download( ota_start is not set), or resume download( ota_start is set)
                ota_header_info.structData.ota_ctrl=OTA_FLAG_START;
                ota_header_info.structData.ota_total_package[0]=(total_num>>8) & 0xff;
                ota_header_info.structData.ota_total_package[1]=(total_num) & 0xff;
                if ((ota_header_info.structData.ota_target_ver[0] != target_ver[0]) ||
                    ota_header_info.structData.ota_target_ver[1] != target_ver[1])
                {
//                    ota_header_info.structData.ota_start[0] = 0;
//                    ota_header_info.structData.ota_start[1] = 0;
                    ota_header_info.structData.ota_ctrl=OTA_FLAG_END;
                    Clear_OTASector();
                    return;
//                    goto __OTA_RESTART;
                }
                GPRS_Set_FMUpgrade_state(1);
                return;
            }
            else if (cur_num == total_num)
            {//Packet sum: Only checksum, no flash data.Compare the checksum to check the entire download data.
                uint8_t flash_data;
                uint32_t flash_checksum = 0;
                ota_header_info.structData.ota_ctrl = ota_ctl;

                if (data_len<2)
                    return;
                if (data_len > 2)
                {
                    tmp_checksum = 0;
                    tmp_checksum = crc_ccitt(tmp_checksum,(data+15),data_len);
                    flash_checksum = (*(data+15+data_len)<<8) + *(data+15+data_len+1);
                    len += (data_len);
                    sFLASH_WriteBuffer(data+15,OTA_DATA_ADDR+len-(data_len),data_len);
                    OS_Sleep(MSec_To_Ticks(100));
                    sFLASH_ReadBuffer(data+15,OTA_DATA_ADDR+len-(data_len),data_len);
                    checksum = 0;
                    checksum = crc_ccitt(checksum,(data+15),data_len);
                    if (checksum != tmp_checksum)
                    {
                        DEBUG_PRINT2(DEBUG_HIGH,"[Upgrade]:Wrong checksum, restart [%x,%x]!\n\r",checksum,tmp_checksum);
                        ota_header_info.structData.ota_ctrl=OTA_FLAG_END;
                        Clear_OTASector();
                        return;
//                        goto __OTA_RESTART;
                    }
                    else
                    {                  
                        ota_header_info.structData.ota_downloaded_bytes[3] = len & 0xff;
                        ota_header_info.structData.ota_downloaded_bytes[2] = (len>>8) & 0xff;
                        ota_header_info.structData.ota_downloaded_bytes[1] = (len>>16) & 0xff;
                        ota_header_info.structData.ota_downloaded_bytes[0] = (len>>24) & 0xff;                    
                    }
                    DEBUG_PRINT1(DEBUG_MEDIUM,"[2G]: OTA datalen [%x]\n\r", len);
                }
                Update_OTAHeader();

//                flash_checksum = (*(data+15+data_len-2)<<8) + *(data+15+data_len-1);
                tmp_checksum = 0x0;
                for (i=0;i<total_bytes;i++)
                {
                    sFLASH_ReadBuffer(&flash_data, OTA_DATA_ADDR+i, 1);
                    tmp_checksum = crc_xmodem(tmp_checksum,&flash_data,1);
                    if( 0 == i%1000)
                        Feed_Dog();
                }
                if (flash_checksum != tmp_checksum)
                {
                    DEBUG_PRINT2(DEBUG_HIGH, "[OTA] Flash checksum error! [server:%x,flash:%x]\n\r",flash_checksum,tmp_checksum);
//                    ota_header_info.structData.ota_start[0] = 0;
//                    ota_header_info.structData.ota_start[1] = 0;
                    ota_header_info.structData.ota_ctrl=OTA_FLAG_END;
                    Clear_OTASector();
                    return;
//                    goto __OTA_RESTART;
                }

//                sFLASH_WriteBuffer(ota_header_info.byte, OTA_HEADER_ADDR, sizeof(ota_header_info));
//                OS_Sleep(MSec_To_Ticks(1000));
//                sFLASH_ReadBuffer(ota_header_info.byte, OTA_HEADER_ADDR, sizeof(ota_header_info));
//                Clear_OTASector();
//                sFLASH_EraseSector(GPS_BACKUP_ADDR);
                DEBUG_PRINT1(DEBUG_HIGH,"[Upgrade]:Finished ! CS = [%x]\n\r", flash_checksum);
                OS_Sleep(MSec_To_Ticks(1000));
                SY_Cold_Start();
                return; //just in case,the cold start has not been executed!
            }
            else
            {//Packet xx: flash data packet.Save the data in spi flash and verify flash data .
                ota_header_info.structData.ota_ctrl = ota_ctl;
                ota_header_info.structData.ota_total_package[1] = total_num & 0xff;
                ota_header_info.structData.ota_total_package[0] = (total_num>>8) & 0xff;
                ota_header_info.structData.ota_current_package_index[1] = cur_num & 0xff;
                ota_header_info.structData.ota_current_package_index[0] = (cur_num>>8) & 0xff;
                memcpy(ota_header_info.structData.ota_target_ver, data+1, 4);

                tmp_checksum = 0;
                tmp_checksum = crc_ccitt(tmp_checksum,(data+15),data_len);

                len += data_len;
                if (len > OTA_MAX_LEN)
                {
                    DEBUG_PRINT1(DEBUG_HIGH, "[2G]: OTA exceed max length [%x]\n\r",len);
                    GPRS_Set_FMUpgrade_state(0);
                    return;
                }

                if (data_len > 0)
                {
                    sFLASH_WriteBuffer(data+15,OTA_DATA_ADDR+len-data_len,data_len);
                    OS_Sleep(MSec_To_Ticks(100));
                    sFLASH_ReadBuffer(data+15,OTA_DATA_ADDR+len-data_len,data_len);
                    checksum = 0;
                    checksum = crc_ccitt(checksum,(data+15),data_len);
                    if (checksum != tmp_checksum)
                    {
                        DEBUG_PRINT2(DEBUG_HIGH,"[Upgrade]:Wrong checksum, restart [%x,%x]!\n\r",checksum,tmp_checksum);
                        ota_header_info.structData.ota_ctrl=OTA_FLAG_END;
                        Clear_OTASector();
                        return;
//                        goto __OTA_RESTART;
                    }
                    else
                    {
                        ota_header_info.structData.ota_downloaded_bytes[3] = len & 0xff;
                        ota_header_info.structData.ota_downloaded_bytes[2] = (len>>8) & 0xff;
                        ota_header_info.structData.ota_downloaded_bytes[1] = (len>>16) & 0xff;
                        ota_header_info.structData.ota_downloaded_bytes[0] = (len>>24) & 0xff;
                    }
                    DEBUG_PRINT1(DEBUG_MEDIUM,"[2G]: OTA datalen [%x]\n\r", len);
                }
                Update_OTAHeader();
                GPRS_Set_FMUpgrade_state(1);
                OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_OTA, 1));
            }
        }
        else
        {
//            __OTA_RESTART:
            if (cur_num != 0)
            {
                GPRS_Set_FMUpgrade_state(0);
                vATApp_Restart_OTA_Check();
                //vTelmApp_uploadData(TELM_SEC_EVT_OTACHECK, NULL);
                return;
            }
            else
            {
                if (memcmp(target_ver,ota_header_info.structData.ota_target_ver,4)==0)
                {
                    GPRS_Set_FMUpgrade_state(1);
                    OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_OTA, 1));
                    return;
                }
            }

            ota_header_info.structData.ota_ctrl = OTA_FLAG_START;
            ota_header_info.structData.ota_total_package[0] = (total_num>>8) & 0xff;
            ota_header_info.structData.ota_total_package[1] = (total_num) & 0xff;
            ota_header_info.structData.ota_current_package_index[0] = (cur_num>>8) & 0xff;
            ota_header_info.structData.ota_current_package_index[1] = (cur_num) & 0xff;
            memcpy(ota_header_info.structData.ota_target_ver, target_ver, 4);

            ota_header_info.structData.ota_total_bytes[0] = (total_bytes>>24) & 0xff;
            ota_header_info.structData.ota_total_bytes[1] = (total_bytes>>16) & 0xff;
            ota_header_info.structData.ota_total_bytes[2] = (total_bytes>>8) & 0xff;
            ota_header_info.structData.ota_total_bytes[3] = total_bytes & 0xff;

            ota_header_info.structData.ota_downloaded_bytes[3] = data_len & 0xff;
            ota_header_info.structData.ota_downloaded_bytes[2] = (data_len>>8) & 0xff;
            ota_header_info.structData.ota_downloaded_bytes[1] = (data_len>>16) & 0xff;
            ota_header_info.structData.ota_downloaded_bytes[0] = (data_len>>24) & 0xff;

            for(i=0;i<0x40;i++)
            {
                sFLASH_EraseSector(OTA_DATA_ADDR+i*0x1000);
            }
            if (data_len > 0)
            {
                sFLASH_WriteBuffer(data+15, OTA_DATA_ADDR, data_len);
            }
            Update_OTAHeader();
            GPRS_Set_FMUpgrade_state(1);
            OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_OTA, 1));
        }
    }
}

static void prvTelmProt_ParseActivate(uint8_t *data, uint16_t len)
{
    uint8_t old_stat=Get_Activation_Status();
    if (len < 1)
        return;
    Set_Activation_Status(*data);
    if (*data != old_stat)
    {
        rl_reset();
    }
}

static void prvTelmProt_ParseReset(uint8_t *data, uint16_t len)
{
    if (len<1)
        return;
    if (*data & RESET_BIT_SENSOR)
    {
        //nothing to do
    }
    if (*data & RESET_BIT_GSM)
    {
        //nothing to do
    }
    if (*data & RESET_BIT_GPS)
    {
        //nothing to do
    }
    if (*data & RESET_BIT_MCU)
    {
        rl_reset();
    }
    if (*data & RESET_BIT_CONFIG)
    {
        Reset_default_config();
    }
}

static void prvTelmProt_ParseData(uint8_t *data, uint16_t len)
{
    uint16_t service_id = ((*data)<<2)+((*(data+1)>>6) & 0x3);
    uint8_t cmd_type = (*(data+1)>>1) & 0x1f;
    uint8_t len_ext = *(data+2) & 0x1;
    uint16_t data_ptr = 0;
    uint16_t msg_len = 0;

    if (len_ext==0)
    {
        data_ptr=3;
        msg_len=*(data+2)>>1;
    }
    else
    {
        data_ptr=4;
        msg_len=(*(data+2)>>1)+(*(data+3)<<7);
    }
    switch(service_id)
    {
        case TELM_COMM_ID_DEV:
            if (cmd_type == TELM_INFO_CMD_REQ)
            {
                OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_DEV_UPLOAD, 1));
            }
            break;
        case TELM_COMM_ID_BATT:
            if (cmd_type == TELM_INFO_CMD_REQ)
            {
                OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_BATT_UPLOAD, 1));
            }
            break;
        case TELM_COMM_ID_OTA:
            if (cmd_type == TELM_INFO_CMD_EXEC)
            {
                prvTelmProt_ParseOTA(data+data_ptr, msg_len);
            }
            break;
        case TELM_COMM_ID_CONFIG:
            if (cmd_type == TELM_INFO_CMD_EXEC)
            {
                prvTelmProt_ParseConfig(data+data_ptr, msg_len);
            }
            else if (cmd_type == TELM_INFO_CMD_REQ)
            {
                // send event to upload config data
                prvTelmProt_ParseConfigReq(data+data_ptr, msg_len);
            }
            break;
        case TELM_COMM_ID_ACTIVATE:
            if (cmd_type == TELM_INFO_CMD_EXEC)
            {
                prvTelmProt_ParseActivate(data+data_ptr, msg_len);
            }
            break;
        case TELM_COMM_ID_GPS:
            if (cmd_type == TELM_INFO_CMD_REQ)
            {
                // send event to upload GPS
                OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_GPS_UPLOAD, 1));
            }
            break;
        case TELM_COMM_ID_RESET:
            prvTelmProt_ParseReset(data+data_ptr, msg_len);
            break;
        default:
            break;
    }
}

/*******************************************************************************
*    Function: prvTelmProt_Return_Result
*
*  Parameters: bool : result of executing command
*     Returns: None
* Description: send result of command result to server
*******************************************************************************/
static void prvTelmProt_Return_Result(bool result)
{
}

/*******************************************************************************
*    Function: prvTelmProt_Encode
*
*  Parameters: Telm_Upload_Info_Request const*:uploading request
*  Parameters: Telm_Upload_Info const*:information to upload
*     Returns: None
* Description: check if there is upload request in the queue and send it
*******************************************************************************/
static uint16_t prvTelmProt_Encode(Telm_Upload_Info_Request const* uploadReq, Telm_Upload_Info const * uploadInfo, uint8_t * const data, uint16_t bufSize)
{
    uint16_t ptn = 0;
    uint16_t offset = 0;
    uint8_t infoIndex = 0;//init index is 0
    uint16_t tmp_ptn=0;
    static uint8_t tmp_data[TCP_TX_MAX_LEN];
    uint8_t multiple=0;
    uint8_t header_len=0;

//    DEBUG_PRINT1(DEBUG_MEDIUM,"EVT %d\n\r",uploadReq->evt);
    /* encode each type of infomation according to protocol */
    for (infoIndex = 0; infoIndex < TELM_INFO_IN_PACK_MAX_NUM; infoIndex++)
    {
        uint8_t encode_index = uploadReq->reqTable[uploadReq->evt][infoIndex];
        multiple=1;
//        DEBUG_PRINT1(DEBUG_MEDIUM,"encode %d\n\r",encode_index);
        if (encode_index == TELM_INFO_NONE)
            break;
        if ((encode_index != TELM_INFO_NONE) && (tmp_ptn < bufSize))
        {
            // Set last encode frame to multiple 0
            if ((uploadReq->reqTable[uploadReq->evt][infoIndex+1] == TELM_INFO_NONE) || ((infoIndex+1)>= TELM_INFO_IN_PACK_MAX_NUM))
            {
                multiple=0;
            }
            offset = telm_info_encode_table[encode_index-1](uploadInfo,multiple,&tmp_data[tmp_ptn],bufSize - tmp_ptn);
            if (offset == 0)
            {
                DEBUG_PRINT0(DEBUG_HIGH,"[2G]:Encode empty message!\n\r");
                return 0;
            }
            switch(encode_index)
            {
                case TELM_INFO_DEV:
                    DEBUG_PRINT0(DEBUG_MEDIUM,"[2G]:Device info sent[mid 3]\n\r");
                break;
                case TELM_INFO_DRIVE_BHV:
                    DEBUG_PRINT0(DEBUG_MEDIUM,"[2G]:Exception msg sent[mid 72]\n\r");
                break;
                case TELM_INFO_OTA:
                    DEBUG_PRINT0(DEBUG_MEDIUM,"[2G]:OTA request sent[mid 67]\n\r");
                break;
                case TELM_INFO_OTA_CHECK:
                    DEBUG_PRINT0(DEBUG_MEDIUM,"[2G]:[mid 67]\n\r");
                    break;
                default:
                break;
            }
            /* add whole CRC */
            tmp_ptn += offset;
        }
    }
    header_len=prvTelmProt_Encode_Header(uploadInfo,data,tmp_ptn);
    memcpy(data+header_len,tmp_data,tmp_ptn);

    ptn=tmp_ptn+header_len;
    prvTelmProt_Encode_Chk(data+ptn,data,ptn);
    ptn+=1;
    return ptn;
}

static uint8_t prvTelmProt_Encode_Chk(uint8_t *checksum, uint8_t *const encoded, uint16_t size)
{
    uint16_t i=0;
    uint8_t checksum_tmp=0;
    for (i=0;i<size;i++)
    {
        checksum_tmp=checksum_tmp ^ *(encoded+i);
    }
    *checksum=checksum_tmp;
    return 1;
}

static uint16_t prvTelmProt_Encode_Dev(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize)
{
    Telm_Data_Dev* pDev = (Telm_Data_Dev *)(encoded+3);
//    uint8_t	i = 0;
    /* checksum is not included in TELM_INFO_LEN_BODY,so plus 2 */
    uint16_t ret = TELM_INFO_LEN_DEV + 3;
    int8_t *hw_ver;
    int8_t *sw_ver;
    hw_ver=SY_Hwid();
    sw_ver=SY_Swid();

    *encoded = (TELM_INFO_ID_DEV>>2)&0xff;
    *(encoded+1) = ((TELM_INFO_ID_DEV<<6)&0xff) | (TELM_INFO_CMD_RESP<<1) | (multiple & 0x1);
    *(encoded+2) = (TELM_INFO_LEN_DEV<<1)&0xff;
    if (ret <= bufSize)
    {
//        uint32_t hw_ver_tmp=(hw_ver[3]-'0')*10+(hw_ver[1]-'0');
//        uint32_t sw_ver_tmp=(sw_ver[15]-'0')*10+(sw_ver[14]-'0');
        pDev->structData.hw_ver[0] = '0';
        pDev->structData.hw_ver[1] = '0';
        pDev->structData.hw_ver[2] = hw_ver[1];
        pDev->structData.hw_ver[3] = hw_ver[3];
        pDev->structData.sw_ver[0] = sw_ver[7];
        pDev->structData.sw_ver[1] = sw_ver[8];
        pDev->structData.sw_ver[2] = sw_ver[14];
        pDev->structData.sw_ver[3] = sw_ver[15];
        pDev->structData.dev_type=1;

        memset(pDev->structData.phone_type,0,10);
        ATProt_Get_Loc(pDev->structData.lac,pDev->structData.cell_id);
        pDev->structData.csq=pcATProt_getRSSI();
        pDev->structData.netreg=ATProt_Get_Netreg();
        ATProt_Get_Imei(pDev->structData.imei);
#ifdef EV_TEST_DATA
        {
            Self_Diag_T result;
            diag_get_result(&result);
            pDev->structData.cell_id[0]=0;
            pDev->structData.cell_id[1]=result.temp;
//            memcpy(pDev->structData.cell_id, result.temp, 2);
        }
#endif
    }
    else
    {
        ret = bufSize;
    }
    return ret;
}

static uint16_t prvTelmProt_Encode_GPS(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize)
{
	Telm_Data_GPS* pDrive = (Telm_Data_GPS*)(encoded+5);
    static gps_data_t gpsInfo;
    vGps_Get_Gps_Info(&gpsInfo);

	/* ID and length, so plus 3 */
	uint16_t ret = TELM_INFO_LEN_DRIVE + 5;
    *encoded = (TELM_INFO_ID_GPS>>2)&0xff;
    *(encoded+1) = ((TELM_INFO_ID_GPS<<6)&0xff) | (TELM_INFO_CMD_RESP<<1) | (multiple & 0x1);
    *(encoded+2) = ((TELM_INFO_LEN_DRIVE+2)<<1)&0xff;
    *(encoded+3) = 0x00;
    *(encoded+4) = 0x01;
	if (ret <= bufSize)
	{
        uint32_t timestamp=sys_get_cur_sec_offset();
        uint16_t speed = (gpsInfo.speed[0]<<8)+gpsInfo.speed[1];
        if (!gpsInfo.valid)
        {
//            timestamp=RTC_GetCounter();
            uint8_t clk_tmp[14];
            if (0!=ATProt_Get_Clock(clk_tmp))
            {
                timestamp=sys_get_sec_offset(clk_tmp);
            }
        }
//        DEBUG_PRINT2(DEBUG_MEDIUM,"[GPS Encode][%d.%d]\n\r",timestamp,gpsInfo.valid);
        pDrive->structData.timestamp[0]=(timestamp>>24) & 0xff;
        pDrive->structData.timestamp[1]=(timestamp>>16) & 0xff;
        pDrive->structData.timestamp[2]=(timestamp>>8) & 0xff;
        pDrive->structData.timestamp[3]=(timestamp) & 0xff;
        memcpy(pDrive->structData.longitude,gpsInfo.longitude,4);
        memcpy(pDrive->structData.latitude,gpsInfo.latitude,4);
        memcpy(pDrive->structData.altitude,gpsInfo.altitude,4);
        pDrive->structData.used_sat_num=gpsInfo.gnss_sat_info.used_sat_num + gpsInfo.bd_sat_info.used_sat_num;
        if (false==gpsInfo.valid)
            pDrive->structData.valid_flag=0;
        else
            pDrive->structData.valid_flag=1;
        memcpy(pDrive->structData.cog,gpsInfo.cog,2);
        if (speed<=255)
            pDrive->structData.speed=speed;
        else
            pDrive->structData.speed=255;
        pDrive->structData.pdop=gpsInfo.pdop[1];
        ATProt_Get_Loc(pDrive->structData.lac,pDrive->structData.cell_id);
        Set_Last_GPS_uploader(1);
    }
    else
    {
        ret = bufSize;
    }

    return ret;
}

static uint16_t prvTelmProt_Encode_Drive_Behavior(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize)
{
    uint16_t ret = 4;
    uint16_t encode_len=0;;
    *encoded = (TELM_INFO_ID_DRVBHV>>2)&0xff;
    *(encoded+1) = ((TELM_INFO_ID_DRVBHV<<6)&0xff) | (TELM_INFO_CMD_RESP<<1) | (multiple & 0x1);
    encode_len=get_dbe_data(encoded+4,1000);
    ret += encode_len;
    *(encoded+2) = ((encode_len<<1)&0xff);
    *(encoded+3) = (encode_len>>7)&0xff;
    
    return ret;
}

static uint16_t prvTelmProt_Encode_Get_OTA(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize)
{
    Telm_Data_Ota *pOta = (Telm_Data_Ota*)(encoded+3);
	uint16_t ret = TELM_INFO_LEN_OTA + 3;
//    OTAHdr_Sct_t ota_header;
//    Get_OTA_Info(&ota_header);

	if (ret <= bufSize)
	{
        uint32_t package_number = (ota_header_info.structData.ota_current_package_index[0] << 8)+
                (ota_header_info.structData.ota_current_package_index[1]);
        *encoded = (TELM_INFO_ID_OTA>>2)&0xff;
        *(encoded+1) = ((TELM_INFO_ID_OTA<<6)&0xff) | (TELM_INFO_CMD_RESP<<1) | (multiple & 0x1);
        *(encoded+2) = (TELM_INFO_LEN_OTA<<1)&0xff;
        package_number+=1;
        pOta->structData.ota_status=ota_header_info.structData.ota_ctrl;
        memcpy(pOta->structData.target_ver,ota_header_info.structData.ota_target_ver,4);
        pOta->structData.download_index[0]=package_number>>24;
        pOta->structData.download_index[1]=package_number>>16;
        pOta->structData.download_index[2]=package_number>>8;
        pOta->structData.download_index[3]=package_number & 0xff;
        memcpy(pOta->structData.download_bytes,ota_header_info.structData.ota_downloaded_bytes,4);
	}
	else
	{
		ret = bufSize;
	}
	return ret;
}

static uint16_t prvTelmProt_Encode_Backup_GPS(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize)
{
    static gps_data_t gpsInfo;
    static uint8_t upload_num=0;
    static uint8_t upload_size=0;
    static uint8_t gps_data_tmp[MAX_BACKUP_GPS_UPLOAD_NUM*sizeof(Telm_Data_GPS)];

	/* ID and length, so plus 2 */
	uint16_t ret = TELM_INFO_LEN_DRIVE + 5;
    vGps_Get_Gps_Info(&gpsInfo);
    *encoded = (TELM_INFO_ID_GPS>>2)&0xff;
    *(encoded+1) = ((TELM_INFO_ID_GPS<<6)&0xff) | (TELM_INFO_CMD_RESP<<1) | (multiple & 0x1);
    if (MAX_BACKUP_GPS_UPLOAD_NUM < Get_GPS_Data_Total_Number())
    {
        upload_num=MAX_BACKUP_GPS_UPLOAD_NUM;
    }
    else
    {
        upload_num=Get_GPS_Data_Total_Number();
    }
	if (ret <= bufSize)
	{
        upload_num=Read_GPS_Data(gps_data_tmp,upload_num);
        if (upload_num==0)
            return 0;
        else if (upload_num>MAX_BACKUP_GPS_UPLOAD_NUM)
            upload_num=MAX_BACKUP_GPS_UPLOAD_NUM;
        Set_GPS_Data_Next_Read_Pointer(upload_num);
        upload_size=(upload_num*sizeof(Telm_Data_GPS));
        // upload number not greater than 5
        if (upload_size>127)
        {
            *(encoded+2) = (((upload_size+2)<<1)&0xff) | 0x01;
            *(encoded+3) = ((upload_size+2)>>7)&0xff;
            *(encoded+4) = 0x00;
            *(encoded+5) = upload_num;
            ret=upload_size+6;
            memcpy(encoded+6,gps_data_tmp,upload_size);
        }
        else
        {
            *(encoded+2) = ((upload_size+2)<<1)&0xff;
            *(encoded+3) = 0x00;
            *(encoded+4) = upload_num;
            ret=upload_size+5;
            memcpy(encoded+5,gps_data_tmp,upload_size);
        }
    }
    else
    {
        ret = bufSize;
    }

    return ret;
}

static uint16_t prvTelmProt_Encode_Ota_Check(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize)
{
	uint16_t ret = 0;

	return ret;
}

static uint16_t prvTelmProt_Encode_Cell_Loc(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize)
{
    uint16_t ret=0;
    return ret;
}

static uint16_t prvTelmProt_Encode_Batt(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize)
{
	Telm_Data_Batt* pBatt = (Telm_Data_Batt *)(encoded+3);

	/* checksum is not included in TELM_INFO_LEN_BODY,so plus 2 */
	uint16_t ret = TELM_INFO_LEN_BATT + 3;
    *encoded = (TELM_INFO_ID_PWR>>2)&0xff;
    *(encoded+1) = ((TELM_INFO_ID_PWR<<6)&0xff) | (TELM_INFO_CMD_RESP<<1) | (multiple & 0x1);
    *(encoded+2) = (TELM_INFO_LEN_BATT<<1)&0xff;

	if (ret <= bufSize)
	{
        Self_Diag_T result;
        diag_get_result(&result);
        pBatt->structData.power_status = 2;
        pBatt->structData.voltage = result.int_voltage/10;
	}
	else
	{
		ret = bufSize;
	}

	return ret;
}

static uint16_t prvTelmProt_Encode_Trip(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize)
{
	/* checksum is not included in TELM_INFO_LEN,so plus 2 */
	uint16_t ret = TELM_INFO_LEN_TRIP + 3;
    Telm_Data_Trip *pTrip=(Telm_Data_Trip *)(encoded+3);
    *encoded = (TELM_INFO_ID_TRIPSUM>>2)&0xff;
    *(encoded+1) = ((TELM_INFO_ID_TRIPSUM<<6)&0xff) | (TELM_INFO_CMD_RESP<<1) | (multiple & 0x1);
    *(encoded+2) = (TELM_INFO_LEN_TRIP<<1)&0xff;
    // calculate trip info
    get_trip_data(pTrip->byte);

	return ret;
}

static uint16_t prvTelmProt_Encode_Towing(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize)
{
	uint16_t ret = TELM_INFO_LEN_TOWING + 3;
    Telm_Data_Towing *pData=(Telm_Data_Towing *)(encoded+3);
    static gps_data_t gpsInfo;
    vGps_Get_Gps_Info(&gpsInfo);

    *encoded = (TELM_INFO_ID_TOWING>>2)&0xff;
    *(encoded+1) = ((TELM_INFO_ID_TOWING<<6)&0xff) | (TELM_INFO_CMD_RESP<<1) | (multiple & 0x1);
    *(encoded+2) = (TELM_INFO_LEN_TOWING<<1)&0xff;

	if (ret <= bufSize)
	{
        uint32_t timestamp=sys_get_cur_sec_offset();
        uint16_t speed = (gpsInfo.speed[0]<<8)+gpsInfo.speed[1];
        if (!gpsInfo.valid)
        {
            uint8_t clk_tmp[14];
            if (0!=ATProt_Get_Clock(clk_tmp))
            {
                timestamp=sys_get_sec_offset(clk_tmp);
            }
        }
        pData->structData.timestamp[0]=(timestamp>>24) & 0xff;
        pData->structData.timestamp[1]=(timestamp>>16) & 0xff;
        pData->structData.timestamp[2]=(timestamp>>8) & 0xff;
        pData->structData.timestamp[3]=(timestamp) & 0xff;
        memcpy(pData->structData.longitude,gpsInfo.longitude,4);
        memcpy(pData->structData.latitude,gpsInfo.latitude,4);
        memcpy(pData->structData.altitude,gpsInfo.altitude,4);
        if (false==gpsInfo.valid)
            pData->structData.valid=0;
        else
            pData->structData.valid=1;
        memcpy(pData->structData.heading,gpsInfo.cog,2);
        if (speed<=255)
            pData->structData.speed=speed;
        else
            pData->structData.speed=255;
        ATProt_Get_Loc(pData->structData.lac,pData->structData.cell_id);
    }

    return ret;
}

static uint16_t prvTelmProt_Encode_Activation(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize)
{
	uint16_t ret = 4;
    *encoded = (TELM_INFO_ID_ACTIVATE>>2)&0xff;
    *(encoded+1) = ((TELM_INFO_ID_ACTIVATE<<6)&0xff) | (TELM_INFO_CMD_RESP<<1) | (multiple & 0x1);
    *(encoded+2) = (1<<1)&0xff;
    *(encoded+3) = Get_Activation_Status();
    return ret;
}

static uint16_t prvTelmProt_Encode_Sim(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize)
{
	uint16_t ret = 11;
    *encoded = (TELM_INFO_ID_SIM>>2)&0xff;
    *(encoded+1) = ((TELM_INFO_ID_SIM<<6)&0xff) | (TELM_INFO_CMD_RESP<<1) | (multiple & 0x1);
    *(encoded+2) = (TELM_INFO_LEN_SIM<<1)&0xff;

    ATProt_Get_Imsi(encoded+3);
    return ret;
}

static uint16_t prvTelmProt_Encode_Removal(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize)
{
    uint16_t ret = TELM_INFO_LEN_REMOVAL+3;
    Telm_Data_Removal *pData=(Telm_Data_Removal *)(encoded+3);
    static gps_data_t gpsInfo;
    vGps_Get_Gps_Info(&gpsInfo);

    *encoded = (TELM_INFO_ID_REMOVAL>>2)&0xff;
    *(encoded+1) = ((TELM_INFO_ID_REMOVAL<<6)&0xff) | (TELM_INFO_CMD_RESP<<1) | (multiple & 0x1);
    *(encoded+2) = (TELM_INFO_LEN_REMOVAL<<1)&0xff;

//    ATProt_Get_Imsi(encoded+3);
	if (ret <= bufSize)
	{
        uint32_t timestamp=sys_get_cur_sec_offset();
        Self_Diag_T result;
        diag_get_result(&result);
        if (!gpsInfo.valid)
            timestamp=RTC_GetCounter();
        pData->structData.timestamp[0]=(timestamp>>24) & 0xff;
        pData->structData.timestamp[1]=(timestamp>>16) & 0xff;
        pData->structData.timestamp[2]=(timestamp>>8) & 0xff;
        pData->structData.timestamp[3]=(timestamp) & 0xff;
        memcpy(pData->structData.longitude,gpsInfo.longitude,4);
        memcpy(pData->structData.latitude,gpsInfo.latitude,4);
        memcpy(pData->structData.altitude,gpsInfo.altitude,4);
        if (!gpsInfo.valid)
            pData->structData.valid=0;
        else
            pData->structData.valid=1;
        pData->structData.external_voltage[0]=0;
        pData->structData.external_voltage[1]=0;
        pData->structData.internal_voltage=result.int_voltage/10;
    }
    else
    {
        ret = bufSize;
    }
    return ret;
}

static uint16_t prvTelmProt_Encode_Config(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize)
{
    uint16_t ret=0;
    Config_t config_data;
    Get_config_data(&config_data);

    *encoded = (TELM_INFO_ID_CONFIG>>2)&0xff;
    *(encoded+1) = ((TELM_INFO_ID_CONFIG<<6)&0xff) | (TELM_INFO_CMD_RESP<<1) | (multiple & 0x1);

    switch(config_upload_id)
    {
        case CONFIG_ID_DEV:
            ret=8;
            *(encoded+2) = (5<<1)&0xff;
            *(encoded+3) = config_data.structData.config_ver[0];
            *(encoded+4) = config_data.structData.config_ver[1];
            *(encoded+5) = (CONFIG_ID_DEV>>8) & 0xff;
            *(encoded+6) = (CONFIG_ID_DEV) & 0xff;
            *(encoded+7) = Get_Activation_Status();
            break;
        case CONFIG_ID_ACC:
            ret=14;
            *(encoded+2) = (11<<1)&0xff;
            *(encoded+3) = config_data.structData.config_ver[0];
            *(encoded+4) = config_data.structData.config_ver[1];
            *(encoded+5) = (CONFIG_ID_ACC>>8) & 0xff;
            *(encoded+6) = (CONFIG_ID_ACC) & 0xff;
            *(encoded+7) = config_data.structData.ignition_on_threshold_low;
            *(encoded+8) = config_data.structData.ignition_on_threshold_high;
            *(encoded+9) = config_data.structData.ignition_on_down_max;
            *(encoded+10) = config_data.structData.ignition_on_rise_max;
            *(encoded+11) = config_data.structData.ignition_off_down_max;
            *(encoded+12) = config_data.structData.ignition_on_absolute_voltage;
            *(encoded+13) = config_data.structData.ignition_off_absolute_voltage;
            break;
        case CONFIG_ID_WAKE:
            ret=10;
            *(encoded+2) = (7<<1)&0xff;
            *(encoded+3) = config_data.structData.config_ver[0];
            *(encoded+4) = config_data.structData.config_ver[1];
            *(encoded+5) = (CONFIG_ID_WAKE>>8) & 0xff;
            *(encoded+6) = (CONFIG_ID_WAKE) & 0xff;
            *(encoded+7) = config_data.structData.wakeup_lasting_time[0];
            *(encoded+8) = config_data.structData.wakeup_lasting_time[1];
            *(encoded+9) = config_data.structData.data_resend_times;
            break;
        case CONFIG_ID_CRASH:
            ret=17;
            *(encoded+2) = (14<<1)&0xff;
            *(encoded+3) = config_data.structData.config_ver[0];
            *(encoded+4) = config_data.structData.config_ver[1];
            *(encoded+5) = (CONFIG_ID_CRASH>>8) & 0xff;
            *(encoded+6) = (CONFIG_ID_CRASH) & 0xff;
            *(encoded+7) = 0;
            *(encoded+8) = 0;
            *(encoded+9) = 0;
            *(encoded+10) = 0;
            *(encoded+11) = 0;
            *(encoded+12) = 0;
            *(encoded+13) = 0;
            *(encoded+14) = 0;
            *(encoded+15) = 0;
            *(encoded+16) = 0;
            break;
        case CONFIG_ID_DRIVE:
            ret=21;
            *(encoded+2) = (18<<1)&0xff;
            *(encoded+3) = config_data.structData.config_ver[0];
            *(encoded+4) = config_data.structData.config_ver[1];
            *(encoded+5) = (CONFIG_ID_DRIVE>>8) & 0xff;
            *(encoded+6) = (CONFIG_ID_DRIVE) & 0xff;
            memset(encoded+7,0,14);
            break;
        case CONFIG_ID_GPS:
            ret=12;
            *(encoded+2) = (9<<1)&0xff;
            *(encoded+3) = config_data.structData.config_ver[0];
            *(encoded+4) = config_data.structData.config_ver[1];
            *(encoded+5) = (CONFIG_ID_GPS>>8) & 0xff;
            *(encoded+6) = (CONFIG_ID_GPS) & 0xff;
            *(encoded+7) = config_data.structData.GPS_upload_interval;
            *(encoded+8) = config_data.structData.GPS_record_mileage[0];
            *(encoded+9) = config_data.structData.GPS_record_mileage[1];
            *(encoded+10) = config_data.structData.GPS_record_interval;
            *(encoded+11) = config_data.structData.GPS_record_turning;
            break;
        case CONFIG_ID_LOG:
            ret=9;
            *(encoded+2) = (11<<1)&0xff;
            *(encoded+3) = config_data.structData.config_ver[0];
            *(encoded+4) = config_data.structData.config_ver[1];
            *(encoded+5) = (CONFIG_ID_LOG>>8) & 0xff;
            *(encoded+6) = (CONFIG_ID_LOG) & 0xff;
            *(encoded+7) = config_data.structData.log_max_upload_bytes[0];
            *(encoded+8) = config_data.structData.log_max_upload_bytes[1];
            break;
        case CONFIG_ID_SLEEP:
            ret=10;
            *(encoded+2) = (11<<1)&0xff;
            *(encoded+3) = config_data.structData.config_ver[0];
            *(encoded+4) = config_data.structData.config_ver[1];
            *(encoded+5) = (CONFIG_ID_SLEEP>>8) & 0xff;
            *(encoded+6) = (CONFIG_ID_SLEEP) & 0xff;
            *(encoded+7) = config_data.structData.sleep_time[0];
            *(encoded+8) = config_data.structData.sleep_time[1];
            *(encoded+9) = config_data.structData.network_fail_sleep_time;
            break;
        case CONFIG_ID_BATT:
            ret=9;
            *(encoded+2) = (6<<1)&0xff;
            *(encoded+3) = config_data.structData.config_ver[0];
            *(encoded+4) = config_data.structData.config_ver[1];
            *(encoded+5) = (CONFIG_ID_BATT>>8) & 0xff;
            *(encoded+6) = (CONFIG_ID_BATT) & 0xff;
            *(encoded+7) = config_data.structData.external_low_voltage_threshold;
            *(encoded+7) = config_data.structData.internal_low_voltage_threshold;
            break;
        case CONFIG_ID_THEFT:
            ret=9;
            *(encoded+2) = (6<<1)&0xff;
            *(encoded+3) = config_data.structData.config_ver[0];
            *(encoded+4) = config_data.structData.config_ver[1];
            *(encoded+5) = (CONFIG_ID_THEFT>>8) & 0xff;
            *(encoded+6) = (CONFIG_ID_THEFT) & 0xff;
            *(encoded+7) = 0;
            *(encoded+8) = 0;
            break;
        case CONFIG_ID_TRACKING:
            ret=11;
            *(encoded+2) = (8<<1)&0xff;
            *(encoded+3) = config_data.structData.config_ver[0];
            *(encoded+4) = config_data.structData.config_ver[1];
            *(encoded+5) = (CONFIG_ID_TRACKING>>8) & 0xff;
            *(encoded+6) = (CONFIG_ID_TRACKING) & 0xff;
            *(encoded+7) = config_data.structData.tracking_mode_on_time[0];
            *(encoded+8) = config_data.structData.tracking_mode_on_time[1];
            *(encoded+9) = config_data.structData.tracking_mode_sleep_time[0];
            *(encoded+10) = config_data.structData.tracking_mode_sleep_time[1];
            break;
        case CONFIG_ID_NETWORK:
            ret=46;
            *(encoded+2) = (11<<1)&0xff;
            *(encoded+3) = config_data.structData.config_ver[0];
            *(encoded+4) = config_data.structData.config_ver[1];
            *(encoded+5) = (CONFIG_ID_NETWORK>>8) & 0xff;
            *(encoded+6) = (CONFIG_ID_NETWORK) & 0xff;
            *(encoded+7) = config_data.structData.network_connection_retain_time;
            memcpy(encoded+8,config_data.structData.network_connection_port,2);
            memcpy(encoded+10,config_data.structData.network_connection_IP,16);
            memcpy(encoded+26,config_data.structData.network_connection_APN,20);
            break;
        default:
            ret=0;
            break;
    }

    return ret;
}

static uint16_t prvTelmProt_Encode_GPS_Fix(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize)
{
	uint16_t ret = TELM_INFO_LEN_GPS_FIX+3;
    Telm_Data_GPS_Fix *pData=(Telm_Data_GPS_Fix *)(encoded+3);

    *encoded = (TELM_INFO_ID_GPS_FIX>>2)&0xff;
    *(encoded+1) = ((TELM_INFO_ID_GPS_FIX<<6)&0xff) | (TELM_INFO_CMD_RESP<<1) | (multiple & 0x1);
    *(encoded+2) = (TELM_INFO_LEN_GPS_FIX<<1)&0xff;

	if (ret <= bufSize)
	{
        uint32_t first_fix_time=Periodic_Get_GPS_Fixed();
        pData->structData.fix_time[0]=(first_fix_time>>8) & 0xff;
        pData->structData.fix_time[1]=(first_fix_time) & 0xff;
    }
    else
    {
        ret = bufSize;
    }
    return ret;
}

static uint16_t prvTelmProt_Encode_Crash(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize)
{
	uint16_t ret = 0;
    uint16_t data_len=0;
    *encoded = (TELM_INFO_ID_CRASH>>2)&0xff;
    *(encoded+1) = ((TELM_INFO_ID_CRASH<<6)&0xff) | (TELM_INFO_CMD_RESP<<1) | (multiple & 0x1);

    // Crash data, sensor data list
    data_len=get_crash_data(encoded+3,800);

    *(encoded+2) = ((data_len<<1)&0xff);
//    *(encoded+3) = (data_len>>7)&0xff;
    ret=4+data_len;

    return ret;
}

static uint16_t prvTelmProt_Encode_Sensor(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize)
{
	uint16_t ret = TELM_INFO_LEN_SENSOR;
    uint16_t sensor_size=0;
    Telm_Data_Sensor *pSensor=(Telm_Data_Sensor *)(encoded+4);
    *encoded = (TELM_INFO_ID_SENSOR>>2)&0xff;
    *(encoded+1) = ((TELM_INFO_ID_SENSOR<<6)&0xff) | (TELM_INFO_CMD_RESP<<1) | (multiple & 0x1);

    sensor_size = get_sensor_data(encoded+4, 0, 200);
    ret=sensor_size+4;
    *(encoded+2) = (((sensor_size)<<1)&0xff) | 0x01;
    *(encoded+3) = ((sensor_size)>>7)&0xff;
    // Sensor data list

    return ret;
}

static uint16_t prvTelmProt_Encode_Dashboard(Telm_Upload_Info const * uploadInfo, uint8_t multiple, uint8_t* const encoded, uint16_t bufSize)
{
	uint16_t ret = TELM_INFO_LEN_DASHBOARD+3;
    Telm_Data_Dashboard *pdata=(Telm_Data_Dashboard *)(encoded+3);
    *encoded = (TELM_INFO_ID_SENSOR>>2)&0xff;
    *(encoded+1) = ((TELM_INFO_ID_SENSOR<<6)&0xff) | (TELM_INFO_CMD_RESP<<1) | (multiple & 0x1);
    *(encoded+2) = (TELM_INFO_LEN_DASHBOARD<<1)&0xff;

    pdata->structData.acc_status=rl_get_acc_status();
    pdata->structData.window=0;
    pdata->structData.door=0;
    return ret;
}

/*******************************************************************************
*    Function:  prvTelmProt_Encode_Remap
*
*  Parameters:  uint8_t * const:buffer to remap
*  Parameters:  uint8_t :length to remap
*  Parameters:  uint8_t :max length of buffer
*     Returns:  None
* Description:  
*******************************************************************************/
static uint16_t prvTelmProt_Encode_Remap(uint8_t * const buf,uint16_t len, uint16_t bufSize)
{
    uint16_t i = 0;
    uint16_t j = 0;
    uint8_t temp;
    uint8_t inserted = 0x00;
    for (i = 0; (i < len) && (len <= bufSize); i++)
    {
        switch (buf[i])
        {
            /* 0x1C->0x1B,0x1E */
            case 0x1C:
                buf[i] = 0x1B;
                inserted = 0x1E;
                break;
            /* 0x1B->0x1B,0x1D */
            case 0x1B:
                inserted = 0x1D;
                break;
            default:
                break;
        }
        /* insert additional charator */
        if (inserted != 0x00)
        {
            for (j = i + 1; j <= len; j++)
            {
                temp = buf[j];
                buf[j] = inserted;
                inserted = temp;
            }
            i++;
            len++;
            inserted = 0x00;
        }
    }
    return len;
}

static uint8_t prvTelmProt_Encode_Header(Telm_Upload_Info const * uploadInfo, uint8_t* const encoded, uint16_t len)
{
    // return encoded length
    uint8_t ret=0;
    uint8_t chk_len=0;
    uint8_t len_ext=0;
    *encoded=HEADER_STX;

    *(encoded+1)=0;
    if(uploadInfo->imei_flag)
    {
        *(encoded+1) |= HEADER_IMEI_FLAG;
    }
    else
    {
        *(encoded+1) &= ~HEADER_IMEI_FLAG;
    }
    if(uploadInfo->ts_flag)
    {
        *(encoded+1) |= HEADER_TS_FLAG;
    }
    else
    {
        *(encoded+1) &= ~HEADER_TS_FLAG;
    }
    if(uploadInfo->msgttl_flag)
    {
        *(encoded+1) |= HEADER_MSGTTL_FLAG;
    }
    else
    {
        *(encoded+1) &= ~HEADER_MSGTTL_FLAG;
    }
    if(uploadInfo->chk_flag)
    {
        *(encoded+1) |= HEADER_CHK_FLAG;
    }
    else
    {
        *(encoded+1) &= ~HEADER_CHK_FLAG;
    }
    *(encoded+1) |= (uploadInfo->ver<<2) & HEADER_VER;
    ret+=2;

    // If exceed max length, set to max length
    if (len>32740)
    {
        len=32740;
    }
    // one or two byte encode
    if (len <= 113)
    {
//        *(encoded+ret)=(len<<1) & 0xFE;
        ret+=1;
        len_ext=0;
    }
    else
    {
//        *(encoded+ret)=(len<<1) & 0xFE;
//        *(encoded+ret+1)=(len>>7) & 0xFF;
        ret+=2;
        len_ext=1;
    }

    *(encoded+ret)=0;
    *(encoded+ret)|=(uploadInfo->packet_type & 0x1) << 7;
    *(encoded+ret)|=(uploadInfo->ack_flag & 0x01) <<6;
    *(encoded+ret)|=(uploadInfo->trans_mode & 0x07) << 3;
    *(encoded+ret)|=(uploadInfo->priority & 0x07);
    ret++;
    *(encoded+ret)=uploadInfo->packet_id;
    ret++;

    if (uploadInfo->imei_flag)
    {
        ATProt_Get_Imei(encoded+ret);        
        ret+=8;
    }
    if (uploadInfo->ts_flag)
    {
        memcpy(encoded+ret,uploadInfo->utcTime,4);
        ret+=4;
    }
    if (uploadInfo->msgttl_flag)
    {
        memcpy(encoded+ret,uploadInfo->msgttl,2);
        ret+=2;
    }
    if(uploadInfo->chk_flag)
    {
        chk_len=1;
    }

    if (0==len_ext)
    {
        *(encoded+2)=((ret+len+chk_len-3)<<1) & 0xFE;
    }
    else
    {
        *(encoded+2)=((ret+len+chk_len-4)<<1) & 0xFE;
        *(encoded+2)|=1;
        *(encoded+3)=((ret+len+chk_len-4)>>7) & 0xFF;
    }
    return ret;
}

#ifdef TCOM_DATA_DES_ENCDEC

uint8_t prvTelmProt_DES_CheckKey(void);
/*******************************************************************************
*    Function:  prvTelmProt_DES_CheckKey
*
*  Parameters:  void
*     Returns:  the key result
* Description:  
*******************************************************************************/
uint8_t prvTelmProt_DES_CheckKey(void)
{
   uint8_t i;
//   uint8_t const *pImsi;
   char const *pImsi = "460100200300400";
   uint32_t int_Imsi;
   
   /*First check if the enc key has been made*/
   if (enc_key_made == 0)
   {
      /*Make enc-dec key*/

      /* Now we use fixed IMSI */
#ifdef USE_SIMCARD_IMSI
      pImsi = pcATProt_getImsiData();
#endif
      int_Imsi = (pImsi[6]-'0')*100000000 + (pImsi[7]-'0') * 10000000 + (pImsi[8]-'0') * 1000000 + \
         (pImsi[9]-'0') * 100000 + (pImsi[10]-'0') * 10000 + (pImsi[11]-'0') * 1000 + (pImsi[12]-'0') * 100 +\
         (pImsi[13]-'0') * 10 + (pImsi[14]-'0') ;

      memset(enc_key_56bits, 0, 26); // High 26 bit is 0

      for (i = 0 ; i < 30;i++) // i is from 0 to 29, 30bits
      {
         enc_key_56bits[55- i] =  (uint8_t)int_Imsi & 1;
         int_Imsi >>= 1;
      }      
   
      DES_MakeSubKeys_56bit(enc_key_56bits,enc_sub_keys);

      enc_key_made = 1;
   }
   return 1;
}

/*******************************************************************************
*    Function:  prvTelmProt_DES_EncPadding
*
*  Parameters:  uint8_t * const:buffer to fill padding
*  Parameters:  uint8_t len:the length before filling
*     Returns:  the filling length
* Description:  
*******************************************************************************/
static uint8_t prvTelmProt_DES_EncPadding(uint8_t * const buf,const uint8_t len)
{
    uint8_t ori_len= len;
    uint8_t des_times = 0;
    uint8_t* pdes;

    /*First check if the des key is created*/
    prvTelmProt_DES_CheckKey();
   
    /*Second fill the paddings*/
    for (;ori_len%8 != 0;ori_len++)
    {
        *(buf+ori_len) = TELM_INFO_PADDING;
    }
    /*Third do DES enc for each 8 bytes*/
    while (des_times < ori_len/8)
    {
        pdes = buf+des_times * 8;
        DES_EncryptBlock(pdes, enc_sub_keys, pdes);
        des_times ++;
    }

    return (ori_len - len);
}
/*******************************************************************************
*    Function:  prvTelmProt_DES_Dec
*
*  Parameters:  uint8_t * const:buffer to fill padding
*  Parameters:  uint8_t len:the length before filling
*     Returns:  void
* Description:  
*******************************************************************************/
static void prvTelmProt_DES_Dec(uint8_t * const buf,const uint16_t len)
{
    uint16_t des_times = 0;
    uint8_t* pdes;

    /*First check if the des key is created*/
    prvTelmProt_DES_CheckKey();

    /*Second do DES enc for each 8 bytes*/
    while (des_times < len/8)
    {
        pdes = buf+des_times * 8;
        DES_DecryptBlock(pdes, enc_sub_keys, pdes);
        des_times ++;
    }
}
#endif

static bool prvTelmProt_parse_header(uint8_t const * const data, uint8_t *service_data, uint16_t *data_len)
{
    bool ret=false;
    uint8_t imei_flag=0;
    uint8_t ts_flag=0;
    uint8_t ttl_flag=0;
    uint8_t len_ext=0;
    uint8_t service_data_ptr=0;
    uint8_t ver=0;
    uint16_t total_len=0;
    if ((*data) != HEADER_STX)
        return ret;
    imei_flag=*(data+1) & HEADER_IMEI_FLAG;
    ts_flag=*(data+1) & HEADER_TS_FLAG;
    ttl_flag=*(data+1) & HEADER_MSGTTL_FLAG;
    ver=(*(data+1) & HEADER_VER) >> 2;
    service_data_ptr+=2;
    len_ext=*(data+service_data_ptr) & 0x1;
    if (len_ext)
    {
        total_len=(*(data+service_data_ptr) >> 1) + (*(data+service_data_ptr+1) << 7);
        service_data_ptr+=2;
    }
    else
    {
        total_len=(*(data+service_data_ptr) >> 1);
        service_data_ptr+=1;
    }
    service_data_ptr+=2;
    total_len-=2;

    if (imei_flag)
    {
        service_data_ptr+=8;
        total_len-=8;
    }
    else
    {
    }

    if (ts_flag)
    {
        service_data_ptr+=4;
        total_len-=4;
    }
    else
    {
    }

    if (ttl_flag)
    {
        service_data_ptr+=2;
        total_len-=2;
    }
    else
    {
    }

    *data_len=total_len;
    memcpy(service_data, data+service_data_ptr, total_len);
    ret=true;
    return ret;
}

/*******************************************************************************
*    Function: prvTelmProt_DecodeAndHandleAck
*
*  Parameters: uint8_t* const:decoded information
*     Returns: none
* Description: decode and inform App the ack
*******************************************************************************/
static void prvTelmProt_DecodeAndHandleAck(uint8_t *data)
{
    TelmApp_Set_Server_Ack_State(true);
    // Check if last engine on/off message sent
    // If sent, clear last engine on/off message in buffer
//    travel_start_backup_msg_cb();
}

/*******************************************************************************
*    Function: prvTelmProt_DecodeAndHandleCmd
*
*  Parameters: uint8_t* const:decoded information
*     Returns: none
* Description: decode and inform App the command
*******************************************************************************/
static void prvTelmProt_DecodeAndHandleCmd(uint8_t *data,uint16_t len)
{
	bool decodeRes = false;
	/* ack data */
//	Telm_Data_Ack	telm_my_ack;
//	callBack		afterAckSent;
	/* clear last command data before decode */
	memset(&telm_last_command, 0x00,sizeof(telm_last_command));

	/* analyze command contents */
	decodeRes = prvTelmProt_Decode_Command((uint8_t*)data,len,&telm_last_command);

	/* prepare ack data:set result */
	if (decodeRes == true)
	{
//		telm_my_ack.structData.result = TELM_ACK_RCV_OK;
		/* if command is decoded correctly, execute it after ACK is sent back to server */
//		afterAckSent = prvTelmProt_commandAckSend;
	}
	else
	{
//		telm_my_ack.structData.result = TELM_ACK_RCV_NG;
		/* if command is decoded incorrectly, just sent NAK */
//		afterAckSent = NULL;
	}

	/* return decode result as ACK */
//    vATApp_IPSEND_Ack(telm_my_ack.byte,TELM_INFO_LEN_ACK, afterAckSent);
}

/*******************************************************************************
*    Function: prvTelmProt_commandAckSend
*
*  Parameters: AT_Cmd_Res res:Ack send result
*     Returns: none
* Description: decode and inform App the command
*******************************************************************************/
static void prvTelmProt_commandAckSend(AT_Cmd_Res res)
{
	/* when command ack sent ok, execute command */
        DEBUG_PRINT1(DEBUG_MEDIUM,"[2G] command ACK sent [%d]!\n\r", res);
	if (res == true)
	{
		/* set command call back which is called to send 
			result back to server when command is executed  */
		telm_last_command.cmdExecuted = prvTelmProt_Return_Result;
		/* notify app the command to execute */
		vTelmApp_Inform_Received_Command(&telm_last_command);
	}
    else
    {
		vTelmApp_Inform_Received_Command(&telm_last_command);
    }
}

/*******************************************************************************
*    Function: prvTelmProt_Decode_Command
*
*  Parameters: uint8_t* const:decoded information
*     Returns: bool true:decode ok/false:decode ng
* Description: encode data according to protocol
*******************************************************************************/
static bool prvTelmProt_Decode_Command(uint8_t const * const data,uint16_t len, Telm_Rcvd_Command * const decoded)
{
    /* for test, we should get vin code from can modual */
    uint16_t i;
    uint16_t encdataLen;

    uint16_t checksum = 0;
    bool result = true;

    if ((decoded == NULL) || (data == NULL))
    {
        return false;
    }

    prvTelmProt_ParseData((uint8_t *)data,len);

    return result;
}

uint8_t* Telm_Get_DEVID(void)
{
    return lastInfo.devId;
}

void TelmProt_Set_New_OTA(uint8_t flag)
{
    ota_packet_flag = flag;
}

uint8_t TelmProt_Is_New_OTA(void)
{
    if(ota_packet_flag)
    {
        ota_packet_flag--;
        return 0;//0: Sending
    }
    else
    {
        return 1;//1: can send next packet.
    }
}

void TelmProt_Sleep_NV_Write(void)
{
//    eng_on_off_ram_copy_to_nv();
//    travel_start_ram_copy_to_nv();
}

// Save travel summary data
void TelmProt_Travel_Summary_Record(void)
{
//    uint16_t rpm_max = 0;
//    uint16_t rpm_now = 0;
//    uint8_t speed_now = 0;
//    uint8_t temp_now = 0;
//    Dcm_DspPidType *current_pid_table = DspGetCurrentPidTable();
//    rpm_max = (travel_summary.structData.rpm[0]<<8) + travel_summary.structData.rpm[1];
//    speed_now = IF_VehSpeed_B;
//    rpm_now = (IF_EngRPM_B1 << 8) + IF_EngRPM_B0;
//    if (temp_now > travel_summary.structData.temp)
//    {
//        travel_summary.structData.temp = temp_now;
//    }
}

// Clear travel summary data
void TelmProt_Clear_Travel_Summary(void)
{
/*    travel_summary.structData.speed_utc[0] = 0xff;
    travel_summary.structData.speed_utc[1] = 0xff;
    travel_summary.structData.speed_utc[2] = 0xff;
    travel_summary.structData.speed_utc[3] = 0xff;
    travel_summary.structData.speed = 0;
    travel_summary.structData.rpm_utc[0] = 0xff;
    travel_summary.structData.rpm_utc[1] = 0xff;
    travel_summary.structData.rpm_utc[2] = 0xff;
    travel_summary.structData.rpm_utc[3] = 0xff;
    travel_summary.structData.rpm[0] = 0;
    travel_summary.structData.rpm[1] = 0;
    travel_summary.structData.temp_utc[0] = 0xff;
    travel_summary.structData.temp_utc[1] = 0xff;
    travel_summary.structData.temp_utc[2] = 0xff;
    travel_summary.structData.temp_utc[3] = 0xff;
    travel_summary.structData.temp = 0;*/
}

/*=======================================================================================*\
 * File Revision History
 *=======================================================================================
 *
\*=======================================================================================*/
