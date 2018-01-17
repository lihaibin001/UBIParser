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

#include "standard.h"
#include "prj_config.h"
#include <time.h>
#include <string.h>
#include "ATProtocol.h"

#include "Debug.h"

#define FLASH_SPI_SECTORSIZE 			(4 * 1024)
#define GPS_LOG_NUMBER_PER_SECOTR		(146)
#define GPS_LOG_MAX_SECTOR_NUM (128)
#define MAX_GPS_LOG_NUMBER				(GPS_LOG_MAX_SECTOR_NUM * GPS_LOG_NUMBER_PER_SECOTR)
#define OTADATAPERPACKAGE				(1 * 1024)
#define FLASHWRITEREPEATTIMES 			5
/* INITIALIZE DATA */
#define INITIALIZE_DATA							(0xa5a5)
#define START_OTA_FLAG 							(0x55AA)
#define INIT_FLAG_DATA 							INITIALIZE_DATA
#define BATTERY_LOG_MAX    (819)

#define OTA_MAX_SIZE (256 * 1024)

#define RTC_REF_DELTA 600

/************************ Global vatiables *****************************/
typedef struct gps_backup_info_tag
{
    uint16_t log_total_number;
    uint16_t log_sector_read_ptr;
    uint16_t log_read_sector;
    uint16_t log_sector_write_ptr;
    uint16_t log_write_sector;
}gps_backup_info_t;

static uint8_t default_devinfo[]={0xa5,0xa5,
                                0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                0x00,0x00,0x00,0x02,
                                0x04,0x00,0x00,0x00,0x01,0x01};
/* *****
 * 2017/2/9 LiHaibin modify
 * Add wakeup_type=0x00, wakeup_time=0x0000
 * ****/
static uint8_t default_config[]={0x00,0x01,
                                0x00,90u,200u,10u,5u,150u,3u,130u,118u,
                                0x00,29,
                                0x00,0x78,0x01,
                                0x05,0xA0,0xF0,
                                30,0x01,0x90,30,5,
                                0x00,0xB4,0x02,0x1C,
                                0x00,0x00,
                                30,0x0F,0xA1,
                                '1', '3', '9', '.', '1', '9', '6', '.', '4', '.', '1', '0', '5',0x00,0x00,0x00,
                                0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                0x02,0x00,0x00};

static DevInfo_Sct_t g_devinfo_sct;
static Config_t g_config_data;

static gps_backup_info_t g_gps_info;
static OTAHdr_Sct_t g_ota_info;

static uint8_t last_gps_uploaded=0;
static uint8_t low_voltage_uploaded=0;

static Last_Trip_t last_trip_info;

static uint32_t ref_rtc = 0; // Reference clock counter
static uint8_t rtc_inited = 0;

static const uint16_t tcom_month_tab[2][12]={ 
   {0,31,59,90,120,151,181,212,243,273,304,334},//No leap year
   {0,31,60,91,121,152,182,213,244,274,305,335}, //Leap year
};

/***************** Internal functions ****************/
static bool Set_Manufacture_Setting(DevInfo_Sct_t devinfo_sct);
static void Init_Param_In_Flash(void);
static void Get_default_config(Config_t *config);


static uint16_t ArrayToUint16(uint8_t *bytes)
{
    uint16_t data ;

    if(bytes == NULL)
        return 0 ;
    data = bytes[0];
    data <<= 8;
    data += bytes[1] ;
    return data ;
}

static bool Uint16ToArray(uint16_t data , uint8_t *array)
{
    if (array == NULL)
        return false ;
    *array = (uint8_t)(data >> 8) & 0xff;
    *(array+1) = data & 0xff;
    return true ;
}

static void Init_Manufacture_Setting(void)
{

    sFLASH_EraseSector(FLASH_MANUFACTURE_SETTING_OFFSET);
    DEBUG_PRINT0(DEBUG_MEDIUM, "[PARAM]:Manufacture setting Data initialized!\n\r");
}

// Load setting from flash, address 0
bool Get_Manufacture_Setting(DevInfo_Sct_t *devinfo_sct)
{
    uint16_t crc = 0;

    if(devinfo_sct == NULL)
        return false ;
    sFLASH_ReadBuffer(devinfo_sct->byte, FLASH_MANUFACTURE_SETTING_OFFSET, sizeof(DevInfo_Sct_t));

    crc = crc_ccitt(crc, devinfo_sct->byte, sizeof(DevInfo_Sct_t) - 2) ;
    if ((((devinfo_sct->structData.initialized[0] << 8) + \
        devinfo_sct->structData.initialized[1]) != INITIALIZE_DATA) ||
        (crc != ((devinfo_sct->structData.crc[0] << 8) + devinfo_sct->structData.crc[1]))
        )
    {
        // read flash error, use default setting
        memcpy(g_devinfo_sct.byte,default_devinfo,sizeof(DevInfo_Sct_t) - 2);
        ATProt_Get_Imei(g_devinfo_sct.structData.dev_sn);
        crc = crc_ccitt(crc, g_devinfo_sct.byte, sizeof(DevInfo_Sct_t) - 2) ;
        g_devinfo_sct.structData.crc[0]=(crc>>8) & 0xff;
        g_devinfo_sct.structData.crc[1]=(crc) & 0xff;
        Init_Param_In_Flash();
        Set_Manufacture_Setting(g_devinfo_sct);
        memcpy(devinfo_sct->byte,g_devinfo_sct.byte,sizeof(DevInfo_Sct_t));
        Get_default_config(&g_config_data);
        return false ;
    }

    return true ;
}

static bool Set_Manufacture_Setting(DevInfo_Sct_t devinfo_sct)
{
    uint16_t crc = 0;
    DevInfo_Sct_t tmp_g_devinfo_sct;

    crc = crc_ccitt(crc, devinfo_sct.byte, sizeof(DevInfo_Sct_t) - 2);
    devinfo_sct.structData.crc[0] = crc >> 8;
    devinfo_sct.structData.crc[1] = crc >> 0;

    {
        sFLASH_EraseSector(FLASH_MANUFACTURE_SETTING_OFFSET);
        OS_Sleep(100);
        sFLASH_WriteBuffer(devinfo_sct.byte,
                            FLASH_MANUFACTURE_SETTING_OFFSET,
                            sizeof(DevInfo_Sct_t));
        OS_Sleep(100);
        sFLASH_ReadBuffer(tmp_g_devinfo_sct.byte,
                            FLASH_MANUFACTURE_SETTING_OFFSET,
                            sizeof(DevInfo_Sct_t));
        if(memcmp(devinfo_sct.byte, tmp_g_devinfo_sct.byte, sizeof(DevInfo_Sct_t))==0)
            return true ;
    }
    return false ;
}


// Get hardware version
extern bool Get_Hardware_Version(uint8_t *hv)
{
    if(hv == NULL)
        return false ;

    memcpy(hv, g_devinfo_sct.structData.mb_ver, 4) ;
    return true ;
}

extern void Set_Activation_Status(uint8_t status)
{
    g_devinfo_sct.structData.activation_status = status;
    Set_Manufacture_Setting(g_devinfo_sct);
}

// Activation status. 0x04 = activated
extern uint8_t Get_Activation_Status(void)
{
//    return g_devinfo_sct.structData.activation_status ;
    return g_devinfo_sct.structData.activation_status;
}

// Get firmware version
extern bool Get_Firmware_Version(uint8_t* fv)
{
    if(fv == NULL)
        return false;

    memcpy(fv, g_devinfo_sct.structData.sw_ver, 4);
    return true;
}

// Get device type
extern bool Get_Device_Type(uint8_t *type)
{
    if(type == NULL)
        return false ;

    *type = g_devinfo_sct.structData.dev_type ;
    return true ;
}
/************************Manufacture Settings*****************************/

/************************Config Settings**********************************/

static void Init_Config(void)
{
    uint16_t crc = 0 ;

    crc = crc_ccitt(crc , g_config_data.byte , sizeof(Config_t) - 2) ;
    g_config_data.structData.crc[0] = crc >> 8 ;
    g_config_data.structData.crc[1] = crc >> 0 ;
    sFLASH_EraseSector(FLASH_CONFIG_OFFSET);
    sFLASH_WriteBuffer(g_config_data.byte,
                        FLASH_CONFIG_OFFSET,
                        sizeof(Config_t));
#if 0
    DEBUG_PRINT0(DEBUG_MEDIUM, "[PARAM]:Config Data initialized!\n\r");
#endif
}

// Get default config from RAM
static void Get_default_config(Config_t *config)
{
    uint16_t crc = 0 ;
    memcpy(config->byte,default_config,sizeof(Config_t)-2);
    crc = crc_ccitt(crc , config->byte , sizeof(Config_t) - 2) ;
    config->structData.crc[0]=(crc>>8) & 0xff;
    config->structData.crc[1]=(crc) & 0xff;
    Set_Config(*config);
}

// Read config data from flash
bool Get_Config(Config_t *config)
{
    uint16_t crc = 0;

    if(config == NULL)
        return false;

#ifdef USE_DEFAULT_CONFIG
    Get_default_config(config);
    return true;
#endif
    
    sFLASH_ReadBuffer(config->byte, FLASH_CONFIG_OFFSET, sizeof(Config_t));

    crc = crc_ccitt(crc, config->byte, sizeof(Config_t)-2);
    if(crc != ((config->structData.crc[0] << 8) + config->structData.crc[1]))
    {
        Get_default_config(config);
        return false;
    }

    return true;
}

// Save config data to flash
bool Set_Config(Config_t config)
{
    uint16_t crc = 0;
//    int count = 0;
    Config_t tmp_config;

    crc = crc_ccitt(crc , config.byte , sizeof(Config_t) - 2);
    config.structData.crc[0] = crc >> 8;
    config.structData.crc[1] = crc;

    memcpy(g_config_data.byte, config.byte, sizeof(Config_t) - 2);
    {
        sFLASH_EraseSector(FLASH_CONFIG_OFFSET);
        OS_Sleep(100);
        sFLASH_WriteBuffer(config.byte, FLASH_CONFIG_OFFSET, sizeof(Config_t));
        OS_Sleep(100);
        sFLASH_ReadBuffer(tmp_config.byte, FLASH_CONFIG_OFFSET, sizeof(Config_t));
        if(memcmp(config.byte, tmp_config.byte, sizeof(Config_t))==0)
            return true ;
//        count++;
    }
    return false ;
}

// Get config version
extern bool Get_Config_Version(uint8_t *cfv)
{
    if(cfv == NULL)
        return false ;

    memcpy(cfv, g_config_data.structData.config_ver, 2);
    return true;
}

extern uint8_t Get_IP_config(uint8_t *ip, uint8_t *port)
{
    uint8_t rt=0;
    uint8_t port_tmp[2];
    rt=strlen(g_config_data.structData.network_connection_IP);
    strcpy(ip,g_config_data.structData.network_connection_IP);
    port_tmp[0]=g_config_data.structData.network_connection_port[1];
    port_tmp[1]=g_config_data.structData.network_connection_port[0];
    DECtoStr(port, port_tmp, 2);
    
    return rt;
}

extern void Get_config_data(Config_t *config)
{
    memcpy(config->byte, g_config_data.byte, sizeof(Config_t));
}

extern void Reset_default_config(void)
{
    Get_default_config(&g_config_data);
}

/************************Config Settings end******************************/

/************************GPS Info*****************************************/

/*******************************************************************************
*    Function:  sys_get_sec_offset
*
*  Parameters:  time_buffer is the string like "20120304,123456"
*     Returns:  None
* Description:  get system time, convert to second.
*******************************************************************************/
uint32_t sys_get_sec_offset(uint8_t *time_buffer)
{
    uint8_t year_offset; // current day offset from 2000 to the start of current year
    uint16_t day_offset; // current day offset from the start of current year

    uint32_t cur_day_offset;//the second offset from 2000/01/01 to  the start of the current  day 2012/03/04
    uint32_t cur_sec_offset; //the second offset from the 00:00:00  to the current time 12:34:56
    uint16_t t;

    if (time_buffer == NULL)
    {
        return 0;
    }

    /* Check the GPS data and  */
    if (time_buffer[0]=='2')
    {
        year_offset = (time_buffer[2] - '0') * 10 + time_buffer[3] - '0';
        if (year_offset%4)
        {
            day_offset = tcom_month_tab[0][(time_buffer[4] - '0') * 10 + time_buffer[5] - '0' -1 ];
        }
        else
        {
            day_offset = tcom_month_tab[1][(time_buffer[4] - '0') * 10 + time_buffer[5] - '0' -1];
        }
        day_offset += (time_buffer[6] - '0') * 10 + time_buffer[7] - '0' - 1;

        cur_day_offset = (year_offset*365ul + ((year_offset-1)/4 + 1) + day_offset)*86400ul;
        cur_sec_offset = ((time_buffer[8] - '0')*10+ time_buffer[9] - '0') * 3600ul + \
              ((time_buffer[10] - '0')*10+ time_buffer[11] - '0')* 60ul + \
              ((time_buffer[12] - '0')*10+ time_buffer[13] - '0');

        cur_sec_offset += cur_day_offset;
        for(t=1970;t<2000;t++) //
        {
            if(Is_Leap_Year(t))
                cur_sec_offset+=31622400; //
            else
                cur_sec_offset+=31536000; //
        }
        // Then plus the RTC adjustment data
    }
    else
    {
        year_offset = ((time_buffer[2] - '0') * 10 + time_buffer[3] - '0')-70;
        if (year_offset%4)
        {
            day_offset = tcom_month_tab[0][(time_buffer[4] - '0') * 10 + time_buffer[5] - '0' -1 ];
        }
        else
        {
            day_offset = tcom_month_tab[1][(time_buffer[4] - '0') * 10 + time_buffer[5] - '0' -1];
        }
        day_offset += (time_buffer[6] - '0') * 10 + time_buffer[7] - '0' - 1;

        if (year_offset>0)
            cur_day_offset = (year_offset*365ul + ((year_offset-1)/4 + 1) + day_offset)*86400ul;
        else
            cur_day_offset = day_offset*86400ul;
        cur_sec_offset = ((time_buffer[8] - '0')*10+ time_buffer[9] - '0') * 3600ul + \
              ((time_buffer[10] - '0')*10+ time_buffer[11] - '0')* 60ul + \
              ((time_buffer[12] - '0')*10+ time_buffer[13] - '0');

        cur_sec_offset += cur_day_offset;
    }
    return cur_sec_offset;
}

/*******************************************************************************
*    Function:  sys_get_cur_sec_offset
*
*  Parameters:  
*     Returns:  None
* Description:  get current second offset from system.
*******************************************************************************/
extern uint32_t sys_get_cur_sec_offset(void)
{
//    uint8_t curGpsUtc[15];
    uint8_t curGpsUtc[17];  // ywf, size 15 is overlapped
   /* Get GPS data from gps modual */
   vGps_Get_Gps_Utc(curGpsUtc);

   if (curGpsUtc[0] == true)
   {
      /* Get GPS data from gps modual */
      if (rtc_inited == 0)
      {
          uint16_t utc_year = (((curGpsUtc[1] - '0') * 1000) +
                      ((curGpsUtc[2] - '0') * 100) +
                      ((curGpsUtc[3] - '0') * 10) +
                      (curGpsUtc[4] - '0'));
          uint8_t utc_month = (((curGpsUtc[5] - '0') * 10) +
                      (curGpsUtc[6] - '0'));
          uint8_t utc_day = (((curGpsUtc[7] - '0') * 10) +
                      (curGpsUtc[8] - '0'));
          uint8_t utc_h = (((curGpsUtc[9] - '0') * 10) +
                      (curGpsUtc[10] - '0'));
          uint8_t utc_m = (((curGpsUtc[11] - '0') * 10) +
                      (curGpsUtc[12] - '0'));
          uint8_t utc_s = (((curGpsUtc[13] - '0') * 10) +
                      (curGpsUtc[14] - '0'));

          RTC_Set(utc_year,utc_month,utc_day,utc_h,utc_m,utc_s);
          // Get RTC adjustment, save with RTC counter
          ref_rtc = RTC_GetCounter();
          rtc_inited = 1;
      }
      else
      {
          // Check RTC counter, if exceed 10 minutes after first located, set RTC again
          if (ref_rtc > 0)
          {
              if ((RTC_GetCounter()-ref_rtc) > RTC_REF_DELTA)
              {
                  uint16_t utc_year = (((curGpsUtc[1] - '0') * 1000) +
                      ((curGpsUtc[2] - '0') * 100) +
                      ((curGpsUtc[3] - '0') * 10) +
                      (curGpsUtc[4] - '0'));
                  uint8_t utc_month = (((curGpsUtc[5] - '0') * 10) +
                      (curGpsUtc[6] - '0'));
                  uint8_t utc_day = (((curGpsUtc[7] - '0') * 10) +
                      (curGpsUtc[8] - '0'));
                  uint8_t utc_h = (((curGpsUtc[9] - '0') * 10) +
                      (curGpsUtc[10] - '0'));
                  uint8_t utc_m = (((curGpsUtc[11] - '0') * 10) +
                      (curGpsUtc[12] - '0'));
                  uint8_t utc_s = (((curGpsUtc[13] - '0') * 10) +
                      (curGpsUtc[14] - '0'));

                  RTC_Set(utc_year,utc_month,utc_day,utc_h,utc_m,utc_s);
                  ref_rtc = RTC_GetCounter();
              }
          }
      }
      return sys_get_sec_offset(&curGpsUtc[1]);   
   }
   else
   {
      /* Get RTC from Local system time */
      uint8_t utc_buff[20];
      if (0 == RTC_Get(utc_buff))
         return sys_get_sec_offset(utc_buff);
   }
   return 0;
}

static void Init_GPS_Info(void)
{
    sFLASH_EraseSector(FLASH_GPS_INFO_OFFSET);
}

bool Switch_GPS_Sturcture_data_to_log(gps_data_t gps_data, GPS_log_t *gps_log)
{
    uint32_t sec_offset;

    if(gps_log == NULL)
        return false;
    memset(gps_log ->byte, 0, sizeof(GPS_log_t));
    memcpy(gps_log->structData.latitude, gps_data.latitude, 4);
    memcpy(gps_log->structData.longitude, gps_data.longitude, 4);
    sec_offset = sys_get_cur_sec_offset();
    if (!gps_data.valid)
    {
        uint8_t clk_tmp[14];
        ATProt_Get_Clock(clk_tmp);
        sec_offset=sys_get_sec_offset(clk_tmp);
    }
    gps_log ->structData.postion_time[0] = sec_offset >> 24;
    gps_log ->structData.postion_time[1] = sec_offset >> 16;
    gps_log ->structData.postion_time[2] = sec_offset >> 8;
    gps_log ->structData.postion_time[3] = sec_offset >> 0;
    memcpy(gps_log->structData.altitude, gps_data.altitude, 4);
    memcpy(gps_log->structData.cog, gps_data.cog, 2) ;

    gps_log ->structData.speed = gps_data.speed[1] ;
    gps_log ->structData.pdop  = gps_data.pdop[1] ;

    gps_log ->structData.used_satellite_number = gps_data.gps_sat_info.used_sat_num;

    return true ;
}

static bool Get_GPS_Info(void)
{
    GPS_Info_t gps_info;
    sFLASH_ReadBuffer(gps_info.byte, FLASH_GPS_INFO_OFFSET, sizeof(GPS_Info_t));

    if ((gps_info.byte[0]==0xff) && (gps_info.byte[1]==0xff))
    {
        memset(&g_gps_info,0,sizeof(gps_backup_info_t));
        return false;
    }
    g_gps_info.log_read_sector = ArrayToUint16(gps_info.structData.log_read_sector);
    g_gps_info.log_sector_read_ptr = ArrayToUint16(gps_info.structData.log_sector_read_pointer);
    g_gps_info.log_write_sector = ArrayToUint16(gps_info.structData.log_write_sector);
    g_gps_info.log_sector_write_ptr = ArrayToUint16(gps_info.structData.log_sector_write_pointer);
    g_gps_info.log_total_number = ArrayToUint16(gps_info.structData.log_total_number);

    if(g_gps_info.log_read_sector > 128)
    {
        g_gps_info.log_read_sector=0;
        g_gps_info.log_sector_read_ptr=0;
        return false;		/* 512K , max: 128sector */
    }
    if(g_gps_info.log_sector_read_ptr > GPS_LOG_NUMBER_PER_SECOTR)
    {
        g_gps_info.log_sector_read_ptr=0;
        return false;
    }
    if(g_gps_info.log_write_sector > 128)
    {
        g_gps_info.log_write_sector=0;
        g_gps_info.log_sector_write_ptr=0;
        return false;		/* 512K , max: 128sector */
    }
    if(g_gps_info.log_sector_write_ptr >= GPS_LOG_NUMBER_PER_SECOTR)
    {
        g_gps_info.log_sector_write_ptr=0;
        return false;
    }
    if(g_gps_info.log_total_number > MAX_GPS_LOG_NUMBER)
    {
        g_gps_info.log_total_number=0;
        return false;
    }

    return true;
}

uint16_t Get_GPS_Data_Total_Number(void)
{
    return g_gps_info.log_total_number;
}

static bool Set_GPS_Info(GPS_Info_t *gps_info_t)
{
    if (gps_info_t->structData.log_total_number == 0)
        return false;

    {
        sFLASH_EraseSector(FLASH_GPS_INFO_OFFSET);
        sFLASH_WriteBuffer(gps_info_t->byte,
                            FLASH_GPS_INFO_OFFSET,
                            sizeof(GPS_Info_t));
    }
    return true;
}

// write gps data to flash
bool Write_GPS_Data(GPS_log_t *gps_log_t)
{
    /* arrive new sector */
    if(g_gps_info.log_sector_write_ptr == 0)
    {
        sFLASH_EraseSector(FLASH_GPS_LOG_OFFSET + g_gps_info.log_write_sector * FLASH_SPI_SECTORSIZE);
        if (g_gps_info.log_total_number==MAX_GPS_LOG_NUMBER)
        {
            g_gps_info.log_total_number-=GPS_LOG_NUMBER_PER_SECOTR;
            if (g_gps_info.log_write_sector>=(GPS_LOG_MAX_SECTOR_NUM-1))
                g_gps_info.log_read_sector = 0;
            else
                g_gps_info.log_read_sector = g_gps_info.log_write_sector+1;
            g_gps_info.log_sector_read_ptr = 0;
        }
    }
    sFLASH_WriteBuffer(gps_log_t->byte,
                    FLASH_GPS_LOG_OFFSET + g_gps_info.log_write_sector * FLASH_SPI_SECTORSIZE + \
                    g_gps_info.log_sector_write_ptr * sizeof(GPS_log_t), \
                    sizeof(GPS_log_t));
    /* 
      one sector store 146(4k / 28) packages , gps data cover 128(512k/4k) sector 
      max package number = 146 * 128 = 18688
    */
    g_gps_info.log_sector_write_ptr++;
    /* if write pointer reach end of flash area , reset to 0 */
    if(g_gps_info.log_sector_write_ptr >= GPS_LOG_NUMBER_PER_SECOTR)
    {
        g_gps_info.log_sector_write_ptr = 0;
        g_gps_info.log_write_sector ++;
    }
    if(g_gps_info.log_write_sector >= GPS_LOG_MAX_SECTOR_NUM)
    {
        g_gps_info.log_write_sector = 0;
        g_gps_info.log_sector_write_ptr = 0;
    }

    if(g_gps_info.log_total_number < MAX_GPS_LOG_NUMBER)
        g_gps_info.log_total_number ++;
    else 
    {
        /* read pointer == write pointer */
        g_gps_info.log_total_number = MAX_GPS_LOG_NUMBER;
        g_gps_info.log_read_sector = g_gps_info.log_write_sector;
        g_gps_info.log_sector_read_ptr = g_gps_info.log_sector_write_ptr;
    }
    return true ;
}

// set read pointer to the next N
// num shall not greater than (GPS_LOG_NUMBER_PER_SECOTR-log_sector_read_ptr)
bool Set_GPS_Data_Next_Read_Pointer(uint8_t num)
{
    if((g_gps_info.log_total_number == 0) || (g_gps_info.log_total_number <= num))
    {
        memset(&g_gps_info, 0, sizeof(gps_backup_info_t));
        return false;
    }
    g_gps_info.log_sector_read_ptr+=num;
    if(g_gps_info.log_sector_read_ptr >= GPS_LOG_NUMBER_PER_SECOTR)
    {
        g_gps_info.log_sector_read_ptr=0;
        g_gps_info.log_read_sector++;
    }
    if(g_gps_info.log_read_sector >= GPS_LOG_MAX_SECTOR_NUM)
    {
        g_gps_info.log_read_sector=0;
        g_gps_info.log_sector_read_ptr=0;
        return true;
    }
    if(g_gps_info.log_total_number > 0)
    {
        g_gps_info.log_total_number -= num;
        if (g_gps_info.log_total_number == 0)
        {
            g_gps_info.log_read_sector = 0;
            g_gps_info.log_write_sector = 0;
            g_gps_info.log_sector_read_ptr = 0;
            g_gps_info.log_sector_write_ptr = 0;
            Init_GPS_Info();
            return true;
        }
    }
    if((g_gps_info.log_read_sector == g_gps_info.log_write_sector) &&
       (g_gps_info.log_sector_read_ptr == g_gps_info.log_sector_write_ptr))
    {
        g_gps_info.log_total_number = 0;
        g_gps_info.log_read_sector = 0;
        g_gps_info.log_write_sector = 0;
        g_gps_info.log_sector_read_ptr = 0;
        g_gps_info.log_sector_write_ptr = 0;
        Init_GPS_Info();
    }

    return true;
}

uint8_t Read_GPS_Data(uint8_t *gps_log_t, uint8_t num)
{
    uint16_t read_sector;
    uint16_t read_sector_pointer;
    uint8_t i=0;

    read_sector_pointer = g_gps_info.log_sector_read_ptr;
    read_sector = g_gps_info.log_read_sector;

    if (num > g_gps_info.log_total_number)
    {
        num = g_gps_info.log_total_number;
    }
    if ((num+g_gps_info.log_sector_read_ptr) >= GPS_LOG_NUMBER_PER_SECOTR)
    {
        num = (GPS_LOG_NUMBER_PER_SECOTR-read_sector_pointer);
    }
    memset(gps_log_t, 0, sizeof(GPS_log_t)*num);
    for (i=0;((i+read_sector_pointer) < GPS_LOG_NUMBER_PER_SECOTR) && (i<num); i++)
    {
        sFLASH_ReadBuffer(gps_log_t+(i*sizeof(GPS_log_t)),
                        FLASH_GPS_LOG_OFFSET + read_sector * FLASH_SPI_SECTORSIZE + (read_sector_pointer+i) * sizeof(GPS_log_t),
                        sizeof(GPS_log_t));
    }

    return num;
}

/************************GPS Info end*************************************/

/************************OTA Info*****************************************/

static void Init_OTA_Info(void)
{
    sFLASH_EraseSector(FLASH_OTA_INFO_OFFSET);
    DEBUG_PRINT0(DEBUG_MEDIUM, "[PARAM]:OTA Info Data initialized!\n\r");
}

bool Set_OTA_Info(OTAHdr_Sct_t ota_info)
{
    memcpy(g_ota_info.byte, ota_info.byte, sizeof(OTAHdr_Sct_t));
    sFLASH_EraseSector(FLASH_OTA_INFO_OFFSET);
    sFLASH_WriteBuffer(ota_info.byte,
                        FLASH_OTA_INFO_OFFSET,
                        sizeof(OTAHdr_Sct_t));
    return true ;
}

bool Get_OTA_Info(OTAHdr_Sct_t *ota_info) 
{
    if(ota_info == NULL)
        return false ;

    sFLASH_ReadBuffer(ota_info->byte, FLASH_OTA_INFO_OFFSET, sizeof(OTAHdr_Sct_t));
    
    return true ;
}

bool Get_OTA_Target_Ver(uint16_t *ver)
{
    OTAHdr_Sct_t ota_info;

    if(ver == NULL)
        return false ;
    if(!(Get_OTA_Info(&ota_info)))
        return false ;

    *ver = (ota_info.structData.ota_target_ver[0] << 8 ) + ota_info.structData.ota_target_ver[1] ;

    return true ;
}

bool Start_Write_OTA_Data(uint32_t total_bytes)
{
    OTAHdr_Sct_t ota_info;
    int i = 0 ;
    uint16_t need_sector = 0 ;

    if(total_bytes > OTA_MAX_SIZE)
        return false ;

    if(!(Get_OTA_Info(&ota_info)))
        return false ;

    ota_info.structData.ota_total_bytes[0] = total_bytes >> 24 ;
    ota_info.structData.ota_total_bytes[1] = total_bytes >> 16 ;
    ota_info.structData.ota_total_bytes[2] = total_bytes >> 8 ;
    ota_info.structData.ota_total_bytes[3] = total_bytes >> 0 ;

    if ((total_bytes % FLASH_SPI_SECTORSIZE)==0)
    {
        need_sector = total_bytes / FLASH_SPI_SECTORSIZE;
    }
    else
    {
        need_sector = total_bytes / FLASH_SPI_SECTORSIZE + 1 ;
    }
    for(i = 0 ; i < need_sector ; i ++)
    {
        sFLASH_EraseSector(FLASH_OTA_DATA_OFFSET + (i * FLASH_SPI_SECTORSIZE)) ;
    }

    Set_OTA_Info(ota_info) ;
    return true ;
}

bool Write_OTA_Data(uint8_t *data , uint16_t len)
{
    uint16_t ota_total_package;
    uint16_t ota_current_package_index;
    uint32_t ota_total_bytes;
    uint32_t ota_downloaded_bytes;
    OTAHdr_Sct_t ota_info;
    uint16_t crc;

    if(data == NULL)
        return false ;

    Get_OTA_Info(&ota_info) ;
    ota_current_package_index = (ota_info.structData.ota_current_package_index[0] << 8) + \
                                 ota_info.structData.ota_current_package_index[1] ;
    ota_total_package         = (ota_info.structData.ota_total_package[0] << 8) + \
                                 ota_info.structData.ota_total_package[1];
    ota_total_bytes           = (ota_info.structData.ota_total_bytes[0] << 24) + \
                                (ota_info.structData.ota_total_bytes[1] << 16) + \
                                (ota_info.structData.ota_total_bytes[2] << 8) + \
                                 ota_info.structData.ota_total_bytes[3];
    ota_downloaded_bytes      = (ota_info.structData.ota_downloaded_bytes[0] << 24) + \
                                (ota_info.structData.ota_downloaded_bytes[1] << 16) + \
                                (ota_info.structData.ota_downloaded_bytes[2] << 8) + \
                                 ota_info.structData.ota_downloaded_bytes[3];
    if(ota_current_package_index > ota_total_package)
        return false;
    if(ota_downloaded_bytes > ota_total_bytes)
        return false;

    sFLASH_WriteBuffer(data , FLASH_OTA_DATA_OFFSET + ota_downloaded_bytes , len);

    ota_current_package_index ++ ;
    ota_downloaded_bytes += len ;

    ota_info.structData.ota_downloaded_bytes[0] = ota_downloaded_bytes >> 24 ;
    ota_info.structData.ota_downloaded_bytes[1] = ota_downloaded_bytes >> 16 ;
    ota_info.structData.ota_downloaded_bytes[2] = ota_downloaded_bytes >> 8 ;
    ota_info.structData.ota_downloaded_bytes[3] = ota_downloaded_bytes >> 0 ;

    ota_info.structData.ota_current_package_index[0] = ota_current_package_index >> 8 ;
    ota_info.structData.ota_current_package_index[1] = ota_current_package_index >> 0 ;

    Set_OTA_Info(ota_info) ;

    return true ;
}

bool End_Write_OTA_Data(uint16_t crc)
{
    OTAHdr_Sct_t ota_info ;

    if(!(Get_OTA_Info(&ota_info)))
        return false ;

    return true ;
}

bool Read_OTA_Data(uint8_t *data , uint16_t len)
{
    if(data == NULL)
        return false ;

    //ota_read_bytes_pointer
    return true ;
}

/************************OTA Info end*************************************/

/************************Battery Info*************************************/
/*
	bettery log
	----------------------------------------------------------------------
	 name	vol1	time1	vol2	time2	...
	----------------------------------------------------------------------
	 size	 1		  4		 1		 4		...
	----------------------------------------------------------------------
	max 819 battery data
	battery_log_numberÔÚconfigÄÚ
*/

bool WriteBatteryData(Battery_log_t g_battery_log_t)
{
    uint16_t battery_log_number = 0;

    if(battery_log_number >= BATTERY_LOG_MAX)
    {
        battery_log_number = 0 ;
        sFLASH_EraseSector(FLASH_BATTERY_LOG_OFFSET);
    }
    sFLASH_WriteBuffer(g_battery_log_t.byte,
                        FLASH_BATTERY_LOG_OFFSET + battery_log_number * sizeof(Battery_log_t),
                        sizeof(Battery_log_t));
    battery_log_number ++;
    return true ;
}

static Battery_log_t ReadBatteryData(uint16_t start)
{
    Battery_log_t g_bettery_log_t ;

    sFLASH_ReadBuffer(g_bettery_log_t.byte,
                    FLASH_BATTERY_LOG_OFFSET + start * sizeof(Battery_log_t) ,
                    sizeof(Battery_log_t));

    return g_bettery_log_t;
}

uint16_t Get_Battery_Log_Number(void)
{
    return 0 ;
}


void Set_Battery_Log_Number(uint16_t bettery_log_number)
{

}

/************************Battery Info end*********************************/

/******************Device started flag*************************/
static uint8_t Get_Dev_Start(void)
{
    uint8_t rt=0;
    uint8_t data[2];
    sFLASH_ReadBuffer(data,FLASH_DEV_START_OFFSET,2);
    if ((data[0]==0x55) && (data[1]==0xaa))
        rt=1;
    return rt;
}

static void Set_Dev_Start(void)
{
    uint8_t data[2]={0x55,0xaa};
    sFLASH_EraseSector(FLASH_DEV_START_OFFSET);
    sFLASH_WriteBuffer(data,FLASH_DEV_START_OFFSET,2);
}
/******************Device started flag end*********************/

/*******************Trip Info**********************************/
void Set_Last_Trip_Info(Last_Trip_t *info)
{
    memcpy(&last_trip_info, info, sizeof(Last_Trip_t));
}

uint8_t Get_Last_Trip_Info(Last_Trip_t *info)
{
    if ((last_trip_info.trip_status!=1) && (last_trip_info.trip_status!=2))
    {
        memset(info, 0, sizeof(Last_Trip_t));
        return 0;
    }
    else
    {
        memcpy(info, &last_trip_info, sizeof(Last_Trip_t));
        return 1;
    }
}

static void Load_Trip_Info(void)
{
    sFLASH_ReadBuffer((uint8_t *)&last_trip_info, FLASH_TRIP_INFO_OFFSET, sizeof(Last_Trip_t));
}

static void Store_Trip_Info(void)
{
    sFLASH_WriteBuffer((uint8_t *)&last_trip_info, FLASH_TRIP_INFO_OFFSET, sizeof(Last_Trip_t));
}

/******************Flash data initialization********************/
static void Init_Param_In_Flash(void)
{
    Init_Manufacture_Setting();
    Init_Config();
    Init_GPS_Info();
    Init_OTA_Info();
}

void Save_Param(void)
{
    GPS_Info_t gps_info;

    Uint16ToArray(g_gps_info.log_total_number, gps_info.structData.log_total_number);
    Uint16ToArray(g_gps_info.log_read_sector, gps_info.structData.log_read_sector);
    Uint16ToArray(g_gps_info.log_sector_read_ptr, gps_info.structData.log_sector_read_pointer);
    Uint16ToArray(g_gps_info.log_write_sector, gps_info.structData.log_write_sector);
    Uint16ToArray(g_gps_info.log_sector_write_ptr, gps_info.structData.log_sector_write_pointer);

    Store_Trip_Info();
    Set_GPS_Info(&gps_info);
}

bool Load_Param(void)
{
    bool res = true;

    if(!(Get_Manufacture_Setting(&g_devinfo_sct)))
    {
        res = false ;
    }
    if(!(Get_Config(&g_config_data)))
    {
        res = false ;
    }
    if(!(Get_GPS_Info()))
    {
        res = false;
    }
    if(!(Get_OTA_Info(&g_ota_info)))
    {
        res = false ;
    }
    Load_Trip_Info();

    if (0==Get_Dev_Start())
        Set_Dev_Start();
    return res ;
}

uint8_t Get_Last_GPS_uploaded(void)
{
    return last_gps_uploaded;
}

void Set_Last_GPS_uploader(uint8_t status)
{
    last_gps_uploaded=status;
}

// 0=GPS point not uploaded, 1=GPS point uploaded
uint8_t Get_Low_Voltage_Uploaded(void)
{
    return low_voltage_uploaded;
}

void Set_Low_Voltage_Uploaded(uint8_t status)
{
    low_voltage_uploaded=status;
}


/* end of file */

