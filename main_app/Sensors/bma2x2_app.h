#ifndef __BMA2x2_APP_H__
#define __BMA2x2_APP_H__
/**********************************************************************
 *  include 
 *********************************************************************/
#include "stm32f10x_conf.h"
#include "stm32f10x.h"
#include "bma2x2.h"
/**********************************************************************
 *  define
 *********************************************************************/

/* bma253 parameter definetion*/
#define BMA253_FIFO_RANGE           (uint8_t)32
/**********************************************************************
 *  typedef 
 *********************************************************************/
typedef struct
{
    uint8_t hg_thres;   //express the high-g threshold
    uint8_t range;
}BMA253_ConfigType;

/**********************************************************************
 *  global function declaration
 *********************************************************************/
bool BMA253_Init(void);
void BMA253_LowLevel_Init(void);
uint8_t BMA253_ReadFIFOxyz(int16_t *dst);
#endif //__BMA2x2_APP_H__