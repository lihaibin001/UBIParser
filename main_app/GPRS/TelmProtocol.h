#ifndef _TELM_PROTOCOL_H_
#define _TELM_PROTOCOL_H_
/**********************************************************************
   Title                                   : TelmApp.h         
                                                                         
   Module Description         : Telematics module header file. used for Telematic internal 
                                                function and external module.

   Author                               : 
   
 *********************************************************************/

/**********************************************************************
 * Include header files
 *********************************************************************/
#include "TelmApp.h"
 /******************************************************************************
* Constant and Macro Definitions using #define
*****************************************************************************/
//#define TELM_INFO_LEN_VIN			(17)
//#define TELM_INFO_LEN_LAT			(16)
//#define TELM_INFO_LEN_LON			(16)
//#define TELM_INFO_LEN_UTC			(4)
#define TELM_INFO_LEN_UTC			(4)
//#define TELM_INFO_LEN_ALT			(8)
#define TELM_INFO_LEN_ODO			(3)
#define TELM_INFO_LEN_AIR_FLOW			(5)
#define TELM_INFO_LEN_DEVID			(16)
//#define TELM_INFO_LEN_DEVID			(8)
//#define TELM_INFO_LEN_MBSN                      (20)

#define TELM_INFO_IN_PACK_MAX_NUM (5)

#define GPS_BACKUP_STATE_IDLE (0x00)
#define GPS_BACKUP_STATE_PUSH (0x01)
#define GPS_BACKUP_STATE_PULL (0x02)

/**********************************************************************
 * Global Enumerations and Structures and Typedefs
 *********************************************************************/
/* struct to define which info is going to be uploaded to server */
typedef struct Telm_Upload_Info_Request_tag
{
	notifyResult informDataUploaded;		/* process after data is uploaded to server 	*/
	Telm_Security_Event	evt;				/* event which caused data upload 				*/
	uint8_t reqTable[TELM_SEC_EVT_NUM][TELM_INFO_IN_PACK_MAX_NUM];			/* define what information to upload			*/
}Telm_Upload_Info_Request;

/* struct to store the info to be uploaded to server */
typedef struct Telm_Upload_Info_tag
{
    uint8_t utcTime[TELM_INFO_LEN_UTC];	/* UTC Time. the second offset from 2000/01/01 00:00:00, 	*/
    uint8_t devId[TELM_INFO_LEN_DEVID];		/* Device ID 								*/
    uint8_t activated; /* device activated flag */
//    uint8_t multiple;
    uint8_t imei_flag;
    uint8_t ts_flag;
    uint8_t msgttl_flag;
    uint8_t ver;
    uint8_t chk_flag;
    uint8_t packet_type;
    uint8_t ack_flag;
    uint8_t trans_mode;
    uint8_t priority;
    uint8_t packet_id;
    uint8_t msgttl[2];
}Telm_Upload_Info;

extern Telm_Upload_Info lastInfo;
/*===========================================================================*\
 * Function Prototypes for Functions with File Level Scope
\*===========================================================================*/

extern void	vTelmProt_informData(uint8_t const * data,uint16_t len);
extern bool vTelmProt_UploadInfo(Telm_Upload_Info_Request const * const req,Telm_Upload_Info const * const info);
extern  bool seach_pid_support_list(uint8_t pid_idx);

//extern void Telm_test_res(void);
extern void Clear_OTASector(void);
extern void Load_OTAHeader(void);
extern void Telm_Test_Write(void);
extern void speed_add_new(uint8_t speed);
extern uint8_t check_speed_exception(uint8_t *high, uint8_t *low);
extern uint8_t speed_check_func(void);

extern void gps_write_nv(void);
extern uint8_t get_nv_gpsdata(uint8_t *data);
extern void clear_nv_gpsdata(uint8_t sector);
extern void set_gps_upload_backup(uint8_t enable);
extern uint8_t gps_upload_status(void);

extern uint8_t check_init_flag(void);
extern uint8_t set_init_flag(void);
extern uint8_t set_dev_id(uint8_t *data);
extern void write_init_config(void);
extern void clear_nv_config(void);
extern uint8_t get_sys_config(uint8_t *config);

extern uint8_t get_dev_info(uint8_t *info);
extern void set_activated(void);
extern void set_deactivate(void);
extern uint8_t get_activated(void);

extern uint8_t batt_volt_nv_check_write_timer(void);
extern void batt_volt_nv_read_info(void);
extern void batt_volt_nv_check_tx(void);
extern void batt_volt_nv_erase(void);
extern void TelmProt_Send_GTimes(void);

extern uint8_t gps_backup_empty(void);
extern uint8_t get_gps_upload_freq(void);
extern void load_from_nv(void);
extern void load_from_rom(void);
extern uint8_t set_mbsn(uint8_t *data);
extern uint8_t gps_buffer_empty(void);
/*
extern uint8_t travel_start_ram_get_status(void);
extern void travel_start_ram_set_status(uint8_t val);
extern uint8_t travel_start_ram_empty(void);
extern uint8_t travel_start_nv_get_status(void);
extern void travel_start_nv_set_status(uint8_t val);
extern uint8_t travel_start_nv_empty(void);
extern void travel_start_ram_copy_to_nv(void);
*/
extern uint8_t set_factory_test(uint8_t pass);
extern void set_car_crash(uint8_t flag);

extern void set_battery_status(uint8_t status);
extern uint8_t get_battery_status(void);
extern void set_check_engine_light(uint8_t status);
extern void set_engine_startup(uint8_t status);
extern void set_go_sleep(uint8_t status);

extern void save_gps_buffer(void);

extern uint8_t TelmProt_Is_New_OTA(void);
extern void TelmProt_Set_New_OTA(uint8_t status);

extern uint8_t* Telm_Get_DEVID(void);

extern void store_last_pos(uint8_t *data);
extern void write_flash_last_pos(void);

extern void save_current_except(void);
extern uint8_t except_backup_empty(void);
#if 0
extern void eng_on_off_nv_write(int16_t data);
#endif
extern uint8_t eng_on_off_nv_empty(void);
extern uint8_t eng_on_off_nv_get_status(void);
extern void eng_on_off_nv_set_status(uint8_t val);

extern void eng_on_off_ram_write(int16_t data);
extern uint8_t eng_on_off_ram_empty(void);
extern uint8_t eng_on_off_ram_get_status(void);
extern void eng_on_off_ram_set_status(uint8_t val);
extern void eng_on_off_ram_copy_to_nv(void);

extern uint8_t TelmProt_Get_Data_MiningEnb(void);
extern uint8_t TelmProt_Get_G_LogEnb(void);
extern void TelmProt_Sleep_NV_Write(void);
extern void Save_RTC_Ref(uint32_t counter);
extern void Read_RTC_Ref(uint32_t *data);
//extern void TelmProt_Check_Dev_Trace(void);
extern void TelmProt_Travel_Summary_Record(void);
extern void TelmProt_Clear_Travel_Summary(void);
#endif

/*=======================================================================================*\
 * File Revision History
 *=======================================================================================
 * ----------  ------   ---------------------------------------------
 *
\*=======================================================================================*/
