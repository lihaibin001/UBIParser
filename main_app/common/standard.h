/**********************************************************************
                Title : standard.h

   Module Description : This file contains all standard include statements

               Author : 

 *********************************************************************/
#ifndef _STANDARD_H_
#define _STANDARD_H_
/**********************************************************************
 * Standard defines
 *********************************************************************/

/**********************************************************************
 * Compiler / Part Specific Defines
 *********************************************************************/

#include "compiler.h"
//#include "micro.h"

/*********************************************************************
 * Standard C Library
 *********************************************************************/
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
/**********************************************************************
 *  Includes of System setup header
 *********************************************************************/

//#include "module.h"            // include micro specific defines
#include "system.h"          

/**********************************************************************
 * Includes of standard C definitions (Compiler / micro independent)
 *********************************************************************/
#include "definiti.h" 

/**********************************************************************
 * Includes of I/O assignments 
 *********************************************************************/
//#include "io.h"

/**********************************************************************
 * Includes operating system definitions
 *********************************************************************/
#include "rtos.h"
#include "rtos_ext.h"
/**********************************************************************
 * General System includes
 *********************************************************************/
#include "prj_config.h"
#include "int2evt.h"  
#include "timer.h"
#include "lowpower.h"
#include "gensubs.h"
#include "main.h"
#include "iic.h"
#include "regulator.h"
#include "psync.h"
#include "relays.h"
#include "diag_task.h"
#include "ring_buf.h"
#include "delay.h"
#include "stm32f10x.h"
#include "spi_flash.h"
//#include "Gsensor.h"
#include "Sensors.h"
#include "record_task.h"
#include "crc_ccitt.h"
#include "str_lib.h"
//#include "dataencap.h"

/**********************************************************************
 * Application specific Header files
 *********************************************************************/
#include "periodic.h"
#include "powerfail.h"
#include "Diag_man.h"
#include "GPRS.h"
#include "TelmProtocol.h"
#include "ACC_Detect.h"

#endif //(#ifndef _STANDARD_H_)
/**********************************************************************
 *
 * REVISION RECORDS
 *
 *********************************************************************/
 
/**********************************************************************

 *********************************************************************/

