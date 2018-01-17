/* $Header:   main.h*/
/**********************************************************************
 *             Title:   MAIN.H
 *
 *       Description:   This file contains all include statements for
 *                      MAIN.  It is included only by MAIN.C.
 *
 *           Created:   
 *
 *            Author:   
 *
 *********************************************************************/
 #ifndef _MAIN_H_
 #define _MAIN_H_
/**********************************************************************
 * Standard defines
 *********************************************************************/

#define IO_4V_CTRL_OUT(value)		        GPIO_WriteBit(GPIOA, GPIO_Pin_11, value);
#define IO_GSM_PWR_ON_OUT(value)			GPIO_WriteBit(GPIOC, GPIO_Pin_7, value);
#define IO_GPS_RESET_OUT(value)	    		GPIO_WriteBit(GPIOB, GPIO_Pin_1, value);
#define IO_3V3_GPS_EN_OUT(value)           	GPIO_WriteBit(GPIOA, GPIO_Pin_8, value);
#define IO_FLASH_WP_OUT(value)		        GPIO_WriteBit(GPIOC, GPIO_Pin_4, value);
#define IO_FLASH_HOLD_OUT(value)		    GPIO_WriteBit(GPIOC, GPIO_Pin_5, value);
#define IO_CHARGE_CTL(value)		        GPIO_WriteBit(GPIOC, GPIO_Pin_0, value);
//#define IO_CHRG_OUT(value)		   GPIO_WriteBit(GPIOC, GPIO_Pin_1, value); //
#define IO_CHRG_CHECK()		                GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1) //
#define IO_3V3_OUT_CHECK()		            GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_8) //

// Red LED
#define IO_MCU_LED1_CTL_OUT(value)			GPIO_WriteBit(GPIOB, GPIO_Pin_6, value);
// Green LED
#define IO_MCU_LED2_CTL_OUT(value)			GPIO_WriteBit(GPIOB, GPIO_Pin_7, value);

/**********************************************************************
 * Standard C Library
 *********************************************************************/

/**********************************************************************
 * Global Function Prototypes                                               
 *********************************************************************/
extern  void mn_Initialize(void);
/**********************************************************************
 * Application specific Header files
 *********************************************************************/

#endif
/**********************************************************************
 *                                                                
 * REVISION RECORDS                                               
 *                                                                
 *********************************************************************/
/**********************************************************************
 * ********************************************************************
 * Date         userid    (Description on following lines: SCR #, etc.)
 * ----------- --------
 *
 *********************************************************************/
