#ifndef __BMI160_APP_H__
#define __BMI160_APP_H__
/**********************************************************************
 *  include 
 *********************************************************************/
//#include "standard.h"        /* include standard includes */
#include "stm32f10x_conf.h"
#include "stm32f10x.h"
#include "stm32f10x.h"
#include "bmi160.h"

/**********************************************************************
 *  define
 *********************************************************************/

/* bmi160 parameter definetion*/
#define BMI160_FIFO_WATER_MARK           (uint8_t)75   //75 * 4 = 300 BYTES
#define BMI160_FIFO_FRAME_SIZE            (uint8_t)12
#define BMI160_FIFO_FRAME_NUM            (uint8_t)25
#define BMI160_FIFO_DATA_SIZE           (BMI160_FIFO_FRAME_NUM * BMI160_FIFO_FRAME_SIZE)
/**********************************************************************
 *  typedef 
 *********************************************************************/
typedef struct
{
    uint8_t hg_thres;   //express the high-g threshold
    uint8_t range;
}BMI160_ConfigType;

/**********************************************************************
 *  global function declaration
 *********************************************************************/
bool BMI160_Initialize(void);
void BMI160_LowLevel_Init(void);
uint8_t BMI160_ReadFIFOxyz(void **pAccel_data, void **pGyro_data, uint8_t *pFrame_cnt);
int8_t BMI160_get_intr_stat(uint8_t *intr_status);

int8_t BMI160_enable_high_g_int(bool enable);
int8_t BMI160_enable_fifo_int(uint8_t full_waterMark, bool enable);
void BMI160_rst_int(void);

#endif //__BMI160_APP_H__