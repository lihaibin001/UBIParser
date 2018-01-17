/* $Header:   lowpower.h   $*/

#ifndef  LOWPOWER_H_FILE
#define  LOWPOWER_H_FILE 1

/**********************************************************************
 *             Title:   lowpower.h
 *
 *       Description:   This file contains micro specific code to place 
 *                      it in low power state 
 *
 *            Author:  
 *
 *  Configuration ID:   
 *
 *********************************************************************/

/**********************************************************************
 * Include files                                                       
 *********************************************************************/
#include    "standard.h"        /* include standard includes */

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
extern void Micro_Go_Sleep(void);
extern void Micro_Go_Standby(void);
extern void Set_Wkup_Enable(uint8_t setting);

/**********************************************************************
 *
 * Revision History
 *
 *********************************************************************
 *
 *
 *********************************************************************/
 
#endif
