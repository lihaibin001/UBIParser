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
   Title                      : record_task.c         
                                                                         
   Module Description         : Handle timing/distance related record tasks.

   Author                     : 
   
 *********************************************************************/

/**********************************************************************
 * Include files                                                       
 *********************************************************************/
#include "standard.h"
#include "record_task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gps.h"
#include "GPRS.h"
#include "ATProtocol.h"

#define USE_DEBUG
#include "Debug.h"

/**********************************************************************
 * Constant and Macro Definitions using #define
 *********************************************************************/
#define DEVICE_UNMOUNT_LIMIT (1)
#define AD_TEMP 0 /* temperature channel */

// 1s timer for upload check
#define GPS_UPLOAD_TIME_1 (1000)

// 3s for gps data upload
#define TRAVEL_UPLOAD_INTERVAL (3)
#define IGNITION_OFF_UPLOAD_INTERVAL (60)

#define EXT_BATT_LOW_LIMIT (1100)
#define EXT_BATT_HIGH_LIMIT (3200)
#define INT_BATT_LOW_LIMIT (300)
#define INT_BATT_HIGH_LIMIT (400)
#define INT_BATT_CHARGE_LIMIT (330)

#define TRIP_STATUS_GOING   (2)
#define TRIP_STATUS_END     (1)

/*********************************************************************/
/* Enumerations and Structures and Typedefs                          */
/*********************************************************************/
typedef struct Driving_BHV_GPS_tag
{
    uint8_t gps_data[26];
}Driving_BHV_GPS_t;

typedef struct Driving_BHV_Status_tag
{
    uint8_t timestamp[4];
    uint8_t gps_num;
    uint8_t dbe_duration[2];
    uint8_t dbe_event;
    uint8_t dbe_max_speed;
    uint8_t dbe_min_speed;
    uint8_t dbe_accel_num;
    uint8_t dbe_accel_setting;
    uint8_t dbe_gyro_num;
    uint8_t dbe_gyro_setting;
}Driving_BHV_Status_t;

typedef struct Driving_BHV_Sensor_tag
{
    uint8_t sensor_data[6];
}Driving_BHV_Sensor_t;

/**********************************************************************
 * Global and Const Variable Defining Definitions / Initializations
 *********************************************************************/

//static uint8_t device_unmounted=1;
static uint8_t trace_mode_on=0;
static uint8_t int_bat_charging=0;
// 0=off; 1=on
static uint8_t ignition_status=0;
static Config_t config_data;

static Last_Trip_t trip_info;
#if 0
static uint8_t trip_status=TRIP_STATUS_END;
static uint32_t trip_start_time;
static uint32_t trip_end_time;
static uint32_t trip_start_odo;
static uint32_t trip_end_odo;
static uint32_t trip_current_odo;
static uint16_t average_speed;
static uint8_t accel_num;
static uint8_t brake_num;
static uint8_t fturn_num;
#endif

static Driving_BHV_Status_t dbe_status;
static Driving_BHV_GPS_t dbe_gps_data[100];
static Driving_BHV_Sensor_t dbe_accel_data[100];
static Driving_BHV_Sensor_t dbe_gyro_data[100];

static Driving_BHV_GPS_t crash_gps_data[100];
static Driving_BHV_Sensor_t crash_accel_data[100];
static uint8_t crash_accel_num=0;
static uint8_t crash_accel_setting=4;
static uint8_t crash_id=0;
static uint32_t crash_time;
static uint8_t crash_gps_num;
static uint16_t crash_duration;

//static crash_sampling_type crash_sampling_schedule = crash_sampling_init;

/**********************************************************************
 * Function Definitions
 *********************************************************************/
static void prvRecord_evt_nop(int16_t data);
static void prvRecord_evt_acc_on(int16_t data);
static void prvRecord_evt_acc_off(int16_t data);
static void prvRecord_evt_acc_source_set(int16_t data);
static void prvRecord_evt_gps_fixed(int16_t data);
static void prvRecord_evt_crash(int16_t data);
static void prvRecord_evt_dbe(int16_t data);

//static uint32_t gps_record_interval(void);
static uint32_t gps_upload_interval(void);
static void test_rtc(void);
static void record_check(void);
//static void vDev_Go_Sleep(void);

static bool Record_flush_Crash_buffer(int16_t *pData, uint8_t Cnt);

/*********************************************************************/
/* Static Variables and Const Variables With File Level Scope        */
/*********************************************************************/

static void_int16_fptr record_event_handler[]=
{
    prvRecord_evt_nop,					// EVT_NOP
    prvRecord_evt_acc_on,                // ACC on event
    prvRecord_evt_acc_off,               // ACC off event
    prvRecord_evt_acc_source_set,        // ACC source setting
    prvRecord_evt_gps_fixed,             // Update trip start time event
    prvRecord_evt_crash,                 // Crash event from BMA253 sensor
    prvRecord_evt_dbe,                   // DBE
    
};
/*********************************************************************/
/* User file include                                                 */
/*********************************************************************/


/*******************************************************************************
*    Function:  Record_Task
*
*  Parameters:  None
*     Returns:  None
* Description:  TASK to handle timing/distance records.
*******************************************************************************/
extern void Record_Task(void* pvParameters)
{
    Data_Message_T msg;
    uint32_t csq_time = OS_Time();

    #ifdef USE_DEBUG
    DEBUG_PRINT0( DEBUG_MEDIUM, "[Record]:Record TASK Started!\r\n");
    #endif

#if 0
    test_rtc();
#endif
    
//    sFLASH_Init();
    
//    Get_Config(&config_data);
    Get_config_data(&config_data);
    
    TMR_Start_Timer(GPS_UPLOAD_TIMER, GPS_UPLOAD_TIME_1, record_check);
    while(PS_Running())
    {
        if(E_OK == OS_Wait_Message(OS_RECORD_TASK, &msg.all, MSec_To_Ticks(500)))
        {
            if(msg.parts.msg < RECORD_NUM_EVENTS)
            {
                if(NULL != record_event_handler[msg.parts.msg])
                {
                    (*record_event_handler[msg.parts.msg])(msg.parts.data);
                }
            }
        }
        if ((csq_time + MSec_To_Ticks(300)) < OS_Time())
        {
            csq_time = OS_Time();
            // Check messages in flash
            if ((Get_GPS_Data_Total_Number()>0) && (0!=GPRS_server_connected()))
            {
                OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_BACKUP_GPS_UP, 1));
            }
        }
    }
    OS_Terminate_Task();
}

/*******************************************************************************
*    Function:  prvRecord_evt_nop
*
*  Parameters:  data
*     Returns:  None
* Description:  No run.
*******************************************************************************/
static void prvRecord_evt_nop(int16_t data)
{
}

/*******************************************************************************
*    Function:  prvRecord_evt_acc_on
*
*  Parameters:  data
*     Returns:  None
* Description:  Acc on event, start trip statistic.
*******************************************************************************/
static void prvRecord_evt_acc_on(int16_t data)
{
    // Start trip statistic works
    uint32_t sec_offset;
    sec_offset = sys_get_cur_sec_offset();

    ignition_status=1;
    // load trip data
    if (0==Get_Last_Trip_Info(&trip_info))
    {
        trip_info.trip_start_odo=0;
    }
    trip_info.trip_end_odo=0;
    trip_info.trip_end_time=0;
    trip_info.accel_num=0;
    trip_info.brake_num=0;
    trip_info.fturn_num=0;
    trip_info.trip_start_time=sec_offset;
    gps_set_trip_start(1);
    trip_info.trip_status=TRIP_FLAG_STARTED;
    OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_TRIP, 2));
}

/*******************************************************************************
*    Function:  prvRecord_evt_acc_off
*
*  Parameters:  data
*     Returns:  None
* Description:  Acc off event, stop trip statistic.
*******************************************************************************/
static void prvRecord_evt_acc_off(int16_t data)
{
    uint32_t sec_offset;
    sec_offset = sys_get_cur_sec_offset();
    // Trip statistic works over
    ignition_status=0;
    trip_info.trip_end_odo=trip_info.trip_start_odo+(gps_get_trip_distance()/10000);
    trip_info.trip_current_odo=trip_info.trip_end_odo;
    trip_info.trip_end_time=sec_offset;
    gps_set_trip_start(0);
    trip_info.trip_status=TRIP_FLAG_ENDED;
    trip_info.average_speed=((trip_info.trip_current_odo-trip_info.trip_start_odo)*3600)/(trip_info.trip_end_time-trip_info.trip_start_time);
    Set_Last_Trip_Info(&trip_info);
    OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_TRIP, 1));
}

/*******************************************************************************
*    Function:  prvRecord_evt_gps_fixed
*
*  Parameters:  data
*     Returns:  None
* Description:  gps_fixed information.
*******************************************************************************/
static void prvRecord_evt_gps_fixed(int16_t data)
{
    if (trip_info.trip_status == TRIP_FLAG_STARTED)
    {
        uint32_t sec_offset;
        sec_offset = sys_get_cur_sec_offset();
        trip_info.trip_start_time=sec_offset;
    }
}

/*******************************************************************************
*    Function:  prvRecord_evt_acc_source_set
*
*  Parameters:  None
*     Returns:  None
* Description:  ACC source event.
*******************************************************************************/
static void prvRecord_evt_acc_source_set(int16_t data)
{
    if (ACC_Set_Source())   // wirte to conifg data to flash
    {
        Get_config_data(&config_data);  // reflesh local config data
        
#ifdef USE_DEBUG
        DEBUG_PRINT1(DEBUG_MEDIUM,"[RECORD]:ACC source is initialied to %d\n\r",config_data.structData.ignition_detection_source);
#endif       
    }  
}

/*******************************************************************************
*    Function:  prvRecord_evt_crash
*
*  Parameters:  None
*     Returns:  None
* Description:  handle crash event.
*******************************************************************************/
static void prvRecord_evt_crash(int16_t data)
{
    if (1== data)
    {
        crash_gps_num = 0;
    }
    else if (!data)
    {
        Sensor_crash_data_type * pCrash;
        
        pCrash = (Sensor_crash_data_type *)Sensor_get_data(SENSOR_TYPE_CRASH_ACCEL);
        crash_id = pCrash->crash_id;
        crash_time = pCrash->crash_time;
        crash_accel_num = pCrash->crash_accel_buf_frame_cnt;
        memcpy(crash_accel_data, pCrash->crash_accel_buffer, crash_accel_num*3*2);
        
        OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_CAR_CRASHED, 0));
    }

    gps_data_t gpsInfo;
    GPS_log_t gps_log;
    vGps_Get_Gps_Info(&gpsInfo);
    Switch_GPS_Sturcture_data_to_log(gpsInfo,&gps_log);
    ATProt_Get_Loc(gps_log.structData.lac,gps_log.structData.cell_id);
    if (!gpsInfo.valid)
    {
        uint32_t timestamp=0;
        uint8_t clk_tmp[14];
        if (0!=ATProt_Get_Clock(clk_tmp))
        {
            timestamp=sys_get_sec_offset(clk_tmp);
        }
        gps_log.structData.postion_time[0]=(timestamp>>24) & 0xff;
        gps_log.structData.postion_time[1]=(timestamp>>16) & 0xff;
        gps_log.structData.postion_time[2]=(timestamp>>8) & 0xff;
        gps_log.structData.postion_time[3]=(timestamp) & 0xff;
    }   
    
    memcpy(crash_gps_data + crash_gps_num, &gps_log, sizeof(gps_log));
    crash_gps_num++;
}

/*******************************************************************************
*    Function:  prvRecord_evt_dbe
*
*  Parameters:  None
*     Returns:  None
* Description:  handle crash event.
*******************************************************************************/
static void prvRecord_evt_dbe(int16_t data)
{
    if (1== data)
    {
        dbe_status.gps_num = 0;
    }
    else if (!data)
    {
        Sensor_dbe_data_type * pDbe;
        
        pDbe = (Sensor_dbe_data_type *)Sensor_get_data(SENSOR_TYPE_DBE_ACCEL);
        dbe_status.timestamp[0]=(pDbe->dbe_time>>24) & 0xff;
        dbe_status.timestamp[1]=(pDbe->dbe_time>>16) & 0xff;
        dbe_status.timestamp[2]=(pDbe->dbe_time>>8) & 0xff;
        dbe_status.timestamp[3]=(pDbe->dbe_time) & 0xff;
        
        dbe_status.dbe_accel_num = pDbe->buf_frame_cnt;
        dbe_status.dbe_gyro_num = pDbe->buf_frame_cnt;
        dbe_status.dbe_event =  pDbe->dbe_id;
        memcpy(dbe_accel_data, pDbe->dbe_accel_buffer, dbe_status.dbe_accel_num*3*2);
        memcpy(dbe_gyro_data, pDbe->dbe_gyro_buffer, dbe_status.dbe_gyro_num*3*2);
        
        dbe_status.dbe_accel_setting = BMI160_ACCEL_RANGE_2G;
        dbe_status.dbe_gyro_setting = BMI160_GYRO_RANGE_2000_DPS;
        
        OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_DBE, 0));
    }

    gps_data_t gpsInfo;
    GPS_log_t gps_log;
    vGps_Get_Gps_Info(&gpsInfo);
    Switch_GPS_Sturcture_data_to_log(gpsInfo,&gps_log);
    ATProt_Get_Loc(gps_log.structData.lac,gps_log.structData.cell_id);
    if (!gpsInfo.valid)
    {
        uint32_t timestamp=0;
        uint8_t clk_tmp[14];
        if (0!=ATProt_Get_Clock(clk_tmp))
        {
            timestamp=sys_get_sec_offset(clk_tmp);
        }
        gps_log.structData.postion_time[0]=(timestamp>>24) & 0xff;
        gps_log.structData.postion_time[1]=(timestamp>>16) & 0xff;
        gps_log.structData.postion_time[2]=(timestamp>>8) & 0xff;
        gps_log.structData.postion_time[3]=(timestamp) & 0xff;
    }   
    
    memcpy(dbe_gps_data + dbe_status.gps_num, &gps_log, sizeof(gps_log));
    dbe_status.gps_num++;
}


static void record_save_current_pos(void)
{
    static gps_data_t gpsInfo;
    GPS_log_t gps_log;
    vGps_Get_Gps_Info(&gpsInfo);
    Switch_GPS_Sturcture_data_to_log(gpsInfo,&gps_log);
    ATProt_Get_Loc(gps_log.structData.lac,gps_log.structData.cell_id);
    if (!gpsInfo.valid)
    {
        uint32_t timestamp=0;
        uint8_t clk_tmp[14];
        if (0!=ATProt_Get_Clock(clk_tmp))
        {
            timestamp=sys_get_sec_offset(clk_tmp);
        }
        gps_log.structData.postion_time[0]=(timestamp>>24) & 0xff;
        gps_log.structData.postion_time[1]=(timestamp>>16) & 0xff;
        gps_log.structData.postion_time[2]=(timestamp>>8) & 0xff;
        gps_log.structData.postion_time[3]=(timestamp) & 0xff;
    }
    Write_GPS_Data(&gps_log);
}

static void record_check(void)
{
    static uint8_t gps_record_count=0;
    // Travel gps check
    
#if 0
    // Check ingnition status
    uint16_t ext_vol=Pwr_Fail_Get_Voltage();
    uint16_t int_vol=Pwr_Fail_Get_Int_Voltage();
    static uint8_t ext_vol_normal=1;
    static uint8_t int_vol_normal=1;
    uint8_t ignition_on_abs=config_data.structData.ignition_on_absolute_voltage;
    uint8_t ignition_off_abs=config_data.structData.ignition_off_absolute_voltage;
#endif    
    
    if (rl_get_acc_status() != OFF)
    {
        // Ignition on
        if (!ignition_status)
        {
            OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_BACKUP_GPS_UP, 1));
        }
        ignition_status = 1;
    }
    else
    {
        // Ignition off
        if (ignition_status)
        {
            OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_BACKUP_GPS_UP, 0));
        }
        ignition_status = 0;
    }

    
    if (0==GPRS_server_connected())
    {
        // Start new timer to send data
//        TMR_Start_Timer(GPS_UPLOAD_TIMER, GPS_UPLOAD_TIME_1, record_check);
    }
    else if ((gps_record_count > TRAVEL_UPLOAD_INTERVAL) && (ignition_status))
    {
        // send travel gps data
        if (0 < Get_GPS_Data_Total_Number())
        {
            if(vGps_Get_Gps_Status())
            {
                record_save_current_pos();
            }
            OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_BACKUP_GPS_UP, 1));
        }
        else
        {
            if(vGps_Get_Gps_Status())
            {
                OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_GPS_UPLOAD, 1));
                #ifdef EV_TEST_DATA
                OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_ENV_TEST, 1));    
                #endif
            }
            else
            {
                #if 1
                OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_GPS_UPLOAD, 1));//?? upload GPS???, ywf
                OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_TRIP, 1));
                #endif
            }
        }
        gps_record_count=0;
    }
    else if ((gps_record_count > IGNITION_OFF_UPLOAD_INTERVAL) && (ignition_status==0))
    {
        // send ignition off gps data
        OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_GPS_UPLOAD, 1));
        OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_TRIP, 1));
        gps_record_count=0;
    }
    gps_record_count++;
    TMR_Start_Timer(GPS_UPLOAD_TIMER, GPS_UPLOAD_TIME_1, record_check);
}

static uint32_t gps_upload_interval(void)
{
    Config_t cfg;
    Get_config_data(&cfg);
    if (cfg.structData.GPS_upload_interval <= 2)
    {
        return 20000;
    }
    else
    {
        return (cfg.structData.GPS_upload_interval*1000);
    }
}

static uint32_t gps_record_interval(void)
{
    Config_t cfg;
    Get_config_data(&cfg);
    if (cfg.structData.GPS_record_interval <=1)
    {
        return 10000;
    }
    else
    {
        return (cfg.structData.GPS_record_interval*1000);
    }
//    return 10000;
}

static void test_rtc(void)
{
    DEBUG_PRINT1(DEBUG_MEDIUM,"RTC:%x\n\r",RTC_GetCounter());
}

void get_trip_data(uint8_t *data)
{
    if (ignition_status)
    {
        *(data)=TRIP_STATUS_GOING;
    }
    else
    {
        *(data)=TRIP_STATUS_END;
    }
    trip_info.trip_current_odo=trip_info.trip_start_odo+(gps_get_trip_distance()/10000);
    trip_info.trip_end_odo=trip_info.trip_current_odo;
    memcpy(data+1, &trip_info.trip_start_time, 4);
    memcpy(data+5, &trip_info.trip_start_odo, 4);
    if (trip_info.trip_current_odo > trip_info.trip_start_odo)
    {
        uint32_t tmp_mileage=trip_info.trip_current_odo-trip_info.trip_start_odo;
        memcpy(data+9,&tmp_mileage,4);
    }
    else
    {
        memset(data+9,0,4);
    }
    *(data+13)=trip_info.average_speed;
    memcpy(data+14,&trip_info.trip_end_time,4);
    memcpy(data+18,&trip_info.trip_end_odo,4);
    *(data+22)=trip_info.accel_num;
    *(data+23)=trip_info.brake_num;
    *(data+24)=trip_info.fturn_num;
}

extern uint16_t get_dbe_data(uint8_t *data, uint16_t max_size)
{
    uint16_t ret_size=(dbe_status.gps_num*26)+(dbe_status.dbe_accel_num*6)+(dbe_status.dbe_gyro_num*6);
    if (ret_size > max_size)
    {
        return 0;
    }
    else
    {
        uint16_t pos=0;
        memcpy(data,dbe_status.timestamp,4);
        pos+=4;
        *(data+pos)=dbe_status.gps_num;
        pos+=1;
        memcpy(data+pos,dbe_gps_data,26*dbe_status.gps_num);
        pos+=26*dbe_status.gps_num;
        memcpy(data+pos,dbe_status.dbe_duration,2);
        pos+=2;
        *(data+pos)=dbe_status.dbe_event;
        pos+=1;
        *(data+pos)=dbe_status.dbe_max_speed;
        pos+=1;
        *(data+pos)=dbe_status.dbe_min_speed;
        pos+=1;
        *(data+pos)=dbe_status.dbe_accel_num;
        pos+=1;
        *(data+pos)=dbe_status.dbe_accel_setting;
        pos+=1;
        memcpy(data+pos,dbe_accel_data,6*dbe_status.dbe_accel_num);
        pos+=6*dbe_status.dbe_accel_num;
        *(data+pos)=dbe_status.dbe_gyro_num;
        pos+=1;
        *(data+pos)=dbe_status.dbe_gyro_setting;
        pos+=1;
        memcpy(data+pos,dbe_gyro_data,6*dbe_status.dbe_gyro_num);
        pos+=6*dbe_status.dbe_gyro_num;
        return ret_size;
    }
}

extern uint16_t get_sensor_data(uint8_t *data, uint8_t sensor_type, uint16_t max_size)
{
    uint16_t ret_size=35;
    uint8_t data_count=0;
    switch(sensor_type)
    {
        case SENSOR_TYPE_DBE_ACCEL:
            data_count = 6*dbe_status.dbe_accel_num;
            ret_size += data_count;
            break;
        case SENSOR_TYPE_DBE_GYRO:
            data_count = 6*dbe_status.dbe_gyro_num;
            ret_size += data_count;
            break;
        case SENSOR_TYPE_CRASH_ACCEL:
            data_count = 6*crash_accel_num;
            ret_size += data_count;
            break;
        default:
            ret_size+=0;
            break;
    }
    if (ret_size > max_size)
    {
        return 0;
    }
    else
    {
        gps_data_t gps_info;
        uint32_t timestamp;
        uint16_t speed;
        vGps_Get_Gps_Info(&gps_info);
        *data = sensor_type;

        timestamp = sys_get_cur_sec_offset();
        speed = (gps_info.speed[0]<<8)+gps_info.speed[1];
        if (!gps_info.valid)
        {
            uint8_t clk_tmp[14];
            if (0!=ATProt_Get_Clock(clk_tmp))
            {
                timestamp=sys_get_sec_offset(clk_tmp);
            }
        }
        *(data+1)=(timestamp>>24) & 0xff;
        *(data+2)=(timestamp>>16) & 0xff;
        *(data+3)=(timestamp>>8) & 0xff;
        *(data+4)=(timestamp) & 0xff;
        memcpy(data+5,gps_info.longitude,4);
        memcpy(data+9,gps_info.latitude,4);
        memcpy(data+13,gps_info.altitude,4);
        *(data+17)=gps_info.gnss_sat_info.used_sat_num + gps_info.bd_sat_info.used_sat_num;
        if (false==gps_info.valid)
            *(data+18)=0;
        else
            *(data+18)=1;
        memcpy(data+19,gps_info.cog,2);
        if (speed<=255)
            *(data+21)=speed;
        else
            *(data+21)=255;
        *(data+22)=gps_info.pdop[1];
        ATProt_Get_Loc(data+23,data+25);

        // timestamp
        timestamp -= (data_count/100);
        *(data+27)=(timestamp>>24) & 0xff;
        *(data+28)=(timestamp>>16) & 0xff;
        *(data+29)=(timestamp>>8) & 0xff;
        *(data+30)=(timestamp) & 0xff;
        // duration
        if (data_count>100)
        {
            *(data+31)=(data_count/100) >> 8;
            *(data+32)=(data_count/100) & 0xff;
        }
        else
        {
            *(data+31)=0x00;
            *(data+32)=0x01;
        }
        // total package
        *(data+33) = data_count;
        
        switch(sensor_type)
        {
            case SENSOR_TYPE_DBE_ACCEL:
                *(data+34) = dbe_status.dbe_accel_setting;
                memcpy(data, dbe_accel_data, ret_size);
                break;
            case SENSOR_TYPE_DBE_GYRO:
                *(data+34) = dbe_status.dbe_gyro_setting;
                memcpy(data, dbe_gyro_data, ret_size);
                break;
            case SENSOR_TYPE_CRASH_ACCEL:
                *(data+34) = crash_accel_setting;
                memcpy(data, crash_accel_data, ret_size);
                break;
            default:
                break;
        }
        return ret_size;
    }
}

extern uint16_t get_crash_data(uint8_t *data, uint16_t max_size)
{
    uint16_t ret=10+26*crash_gps_num+6*crash_accel_num;
    uint16_t pos=0;
    if (ret > max_size)
        return 0;
    *data = crash_id;
//    crash_id++;
    *(data+1)=(crash_time>>24) & 0xff;
    *(data+2)=(crash_time>>16) & 0xff;
    *(data+3)=(crash_time>>8) & 0xff;
    *(data+4)=(crash_time) & 0xff;

    *(data+5)=crash_gps_num;
    pos=6;
    memcpy(data+pos,crash_gps_data,(26*crash_gps_num));
    pos+=26*crash_gps_num;
    *(data+pos)=crash_accel_setting;
    pos++;
    *(data+pos)=crash_duration>>8;
    pos++;
    *(data+pos)=crash_duration & 0xff;
    pos++;
    *(data+pos)=crash_accel_num;
    pos++;
    memcpy(data+pos,crash_accel_data,(6*crash_accel_num));
    return ret;
}

/*******************************************************************************
*    Function:  Record_flush_Crash_buffer
*
*  Parameters:  'pData' point to the buffer that store the crash data
*               'Cnt' express the how much data to store to the buffer          
*     Returns:  None
* Description:  add some crash data to the specified buffer
*******************************************************************************/
#if 0
static bool Record_flush_Crash_buffer(int16_t *pData, uint8_t Cnt)
{
    if(pData == NULL || Cnt > 100)
    {
        return false;
    }
    else
    {
        uint8_t idx;
        for(idx=crash_accel_num; idx<Cnt; idx++)
        {
            //encode axix-x
            crash_accel_data[idx].sensor_data[0] = (*pData >> 8) & 0xFF;
            crash_accel_data[idx].sensor_data[1] = *pData & 0xFF;
            pData++;
            //encode axix-y
            crash_accel_data[idx].sensor_data[2] = (*pData >> 8) & 0xFF;
            crash_accel_data[idx].sensor_data[3] = *pData & 0xFF;
            pData++;
            //encode axix-z
            crash_accel_data[idx].sensor_data[4] = (*pData >> 8) & 0xFF;
            crash_accel_data[idx].sensor_data[5] = *pData & 0xFF;
            pData++;
        }  
    }  
    return true;
}
#endif
/*====================================================================================*\
 * File Revision History
 *====================================================================================
 *
 * Date        userid  (Description on following lines:)
 * ----------- ------  ---------------------------------------------
 *
  ====================================================================================*/

