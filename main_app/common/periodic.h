#ifndef  _PERIODIC_H_
#define  _PERIODIC_H_ 

/**********************************************************************
 *       Title:   periodic.h
 *
 *  Description:  This file contains all standard exports for 
 *                periodic.
 *
 *      Author:   Chen Huichao
 *
 *********************************************************************/

/**********************************************************************
 * Include files                                                       
 *********************************************************************/

/**********************************************************************
 * Global Constant and Macro Definitions using #define                        
 *********************************************************************/

/**********************************************************************
 * Global Enumerations and Structures and Typedefs                          
 *********************************************************************/

/**********************************************************************
 * Global Variable extern Declarations                               
 *********************************************************************/

/**********************************************************************
 * Global Function Prototypes                                               
 *********************************************************************/
void Periodic_Task(void *pvParameters);           // Task to run all periodic functions
void Periodic_Tick(void);           // Function call from RTI ISR to schedule periodic task
void Periodic_Clear_Low_Volt_Cnt(void);     
uint32_t Periodic_Get_Low_Volt_Cnt(void);
uint32_t Periodic_Get_GPS_Fixed(void);
/**********************************************************************
 *
 * Modification Record
 *
 *********************************************************************
 * 
 * 
 *********************************************************************/
 
#endif //_PERIODIC_H_
