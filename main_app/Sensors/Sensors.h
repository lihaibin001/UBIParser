#ifndef _SENSORS_H_
#define _SENSORS_H_
/**********************************************************************
   Title					: Sensors.h         
                                                                         
   Module Description		: interface of Sensors.c
   
   Author					: 
   
 *********************************************************************/

/**********************************************************************
 * Include header files
 *********************************************************************/
#include "bma2x2_app.h"
#include "bmi160_app.h"
/**********************************************************************
 * Constant and Macro Definitions using #define
 *********************************************************************/
// This header file contains definitions for registers that do not
// exist in FXLS8471Q.  If there is a confusion FXLS8471Q datahseet
// should take precedence.

#define CONFIG_DBE_ACCEL_SENSOR_UPLOAD
#define CONFIG_DBE_GYRO_SENSOR_UPLOAD
#define CONFIG_CRASH_ACCEL_SENSOR_UPLOAD

#define SENSOR_DATA_AXIS (3)    
#define SENSOR_DATA_FIFO_LEN (32*3)
#define SENSOR_DATA_BUF_LEN (SENSOR_DATA_FIFO_LEN*3)

/**********************************************************************
 * Global Enumerations and Structures and Typedefs
 *********************************************************************/
// Sensor type
typedef enum {
    SENSOR_TYPE_DBE_ACCEL=1,
    SENSOR_TYPE_DBE_GYRO,
    SENSOR_TYPE_CRASH_ACCEL,
    SENSOR_TYPE_MAX
}Sensor_Type_T;

// Range
typedef enum {
    ACCEL_RANGE_2G=0,
    ACCEL_RANGE_4G,
    ACCEL_RANGE_8G,
    ACCEL_RANGE_16G,
    ACCEL_RANGE_MAX
}Sensor_Accel_Range_T;

typedef enum {
    GYRO_RANGE_2000=0,
    GYRO_RANGE_1000,
    GYRO_RANGE_500,
    GYRO_RANGE_250,
    GYRO_RANGE_125,
    GYRO_RANGE_MAX
}Sensor_Gyro_Range_T;

// ODR setting
typedef enum {
    ODR_6_25=0,
    ODR_12_5,
    ODR_25,
    ODR_50,
    ODR_100,
    ODR_200,
    ODR_400,
    ODR_800,
    ODR_1600,
    ODR_MAX
}Sensor_ODR_T;

typedef  enum  Sensor_Events_Tag
{
    SENSOR_EVT_NOP,
    SENSOR_EVT_BMA253_CRASH,
    SENSOR_EVT_BMI160_DBE,      //driving behavior
    SENSOR_NUM_EVENTS
} Sensor_Events_Type;


typedef enum
{
    SENSOR_INTR_HIGH_G,
    SENSOR_INTR_FIFO_FULL,
    SENSOR_INTR_NUM,
}Sensor_intr_type;

typedef enum
{
    SENSOR_FIFO_MODE_BYPASS = 0,
    SENSOR_FIFO_MODE_FIFO,
    SENSOR_FIFO_MODE_STREAM,
    SENSOR_FIFO_MODE_NUM,
}Sensor_fifo_mode_type;

typedef struct
{
    uint8_t crash_id;
    uint8_t crash_accel_buf_frame_cnt;
    int16_t crash_accel_buffer[SENSOR_DATA_BUF_LEN];
    uint32_t crash_time;
}Sensor_crash_data_type;


typedef struct
{
    uint16_t dbe_id;
    uint16_t buf_frame_cnt;
    int16_t dbe_accel_buffer[BMI160_FIFO_FRAME_NUM*SENSOR_DATA_AXIS*2];
    int16_t dbe_gyro_buffer[BMI160_FIFO_FRAME_NUM*SENSOR_DATA_AXIS*2];
    uint32_t dbe_time;
}Sensor_dbe_data_type;

    
/**********************************************************************
 * Global Function Prototypes
 *********************************************************************/
extern void Sensors_init(void);
extern void Sensor_Task(void* pvParameters);
extern void Sensors_sleep(void);
//extern void Sensors_getFIFO(uint8_t sensor_type, uint8_t *data);
extern void Sensors_get_DBE_data(uint8_t sensor_type, uint8_t *data);
extern void Sensors_get_crash_data(uint8_t sensor_type, uint8_t *data);
extern void Sensors_set_range(uint8_t sensor_type, uint8_t range);
extern void Sensors_set_odr(uint8_t sensor_type, uint8_t odr);
extern void Sensors_INT(uint8_t sensor_type);

//extern uint16_t Sensor_get_data(Sensor_Type_T sensor_type, int16_t *pData);

extern void Sensors_crash_reset(void);
extern void *Sensor_get_data(Sensor_Type_T sensor_type);
/*====================================================================================*\
 * File Revision History
 *====================================================================================
 *
  ====================================================================================*/
#endif

