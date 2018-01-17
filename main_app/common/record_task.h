#ifndef _RECORD_TASK_H_
#define _RECORD_TASK_H_
/**********************************************************************
   Title                      : record_task.h         
                                                                         
   Module Description         : This file is the task for data records
	
   Author                     : 
   
 *********************************************************************/

typedef enum record_event_tag
{
    RECORD_EVT_NOP,
    RECORD_EVT_ACC_ON,
    RECORD_EVT_ACC_OFF,
    RECORD_EVT_ACC_SOURCE_SET,
    RECORD_EVT_GPS_FIXED,
    RECORD_EVT_CRASH,
    RECORD_EVT_DBE,
    
    RECORD_NUM_EVENTS
}record_event_t;

typedef enum
{
    crash_sampling_init,
    crash_sampling_start,
    crash_sampling_end,
}crash_sampling_type;

/*********************************************************************/
/* interface for modules                                             */
/*********************************************************************/

extern void Record_Task(void* pvParameters);
extern uint8_t device_unmount_status(void);
extern void get_trip_data(uint8_t *data);
extern uint16_t get_dbe_data(uint8_t *data, uint16_t max_size);
extern uint16_t get_sensor_data(uint8_t *data, uint8_t sensor_type, uint16_t max_size);
extern uint16_t get_crash_data(uint8_t *data, uint16_t max_size);

/*=======================================================================================*\
 * File Revision History
 *=======================================================================================
 * ----------  ------   ---------------------------------------------
 *
\*=======================================================================================*/
#endif //_GPS_H_
