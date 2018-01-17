/*Header$*/

#ifndef  DEFINITION_H_FILE
#define  DEFINITION_H_FILE 1

/**********************************************************************
 *       Title:   definition.h
 *
 *  Description:  Gloabl defines of miscellaneous useful stuff.
 *
 *      Author:   
 *
 *********************************************************************/

/**********************************************************************
 * Include files                                                       
 *********************************************************************/

/**********************************************************************
 * Global Constant and Macro Definitions using #define                        
 *********************************************************************/
#define  To_Boolean(bit)            ((bit) ? true : false)                 /* convert bit/logical to normalized boolean */
#define  Bit_Copy(target, source)   (target = ((source) ? true : false))   /* set target equal to source */
#define  Bit_Toggle(target)         (target = ((target) ? false : true))   /* toggle target              */
#define  FOREVER                    while(1)

#define icat(x, y) x ## y
#define iins(x, y, z) x ## y ## z
#define cat(x, y)  icat(x,y)
/*
 * DESCRIPTION: macros to append symbols together
 * ---------------------------------------------------------------- */

#ifndef  UP
#define  DOWN           false
#define  UP             !DOWN
#endif

#ifndef  OFF
#define  OFF             false
#define  ON              !OFF
#endif

//#ifndef  CLEARED
//#define  CLEARED         false
//#define  SET             !CLEARED
//#endif

/*
 * DESCRIPTION: Standard boolean labels
 * ---------------------------------------------------------------- */

/*********************************************************************/
/* Enumerations and Structures and Typedefs                          */
/*********************************************************************/

typedef enum   Status_Type_Tag                   // Return status from RTOS functions
   {
      E_OK             =  0,                    // NO Error, successful
      E_ERROR,                                  // General Error
      E_OS_ACCESS,
      E_OS_CALLEVEL,
      E_OS_ID,
      E_OS_LIMIT,
      E_OS_NOFUNC,
      E_OS_RESOURCE,
      E_OS_STATE,
      E_OS_VALUE,
      E_COM_NOMSG,
      E_COM_LOCKED,
      E_TASK_SUSPENDED,
      E_TIMEOUT,                                // Error due to timeout
      E_OUT_OF_RANGE,                           // Error due to parameter out of range
      E_DATAEXISTS,                     /* data to be transferred next exists in TXBn register */
      E_INVALID_CONDITION                       // Error due to invalid conditions
   } Status_Type;

typedef  void (*void_fptr) (void);                 // void void Function pointer typedef
typedef  bool (*bool_fptr) (void);                 // boolean void function typedef
typedef  void (*void_int16_fptr) (int16_t data);   // void funtion taking 16 bit integer

/**********************************************************************
 * Global Enumerations and Structures and Typedefs                          
 *********************************************************************/

/**********************************************************************
 * Global Variable extern Declarations                               
 *********************************************************************/

/**********************************************************************
 * Global Function Prototypes                                               
 *********************************************************************/

/**********************************************************************
 *
 * Modification Record
 *
 *********************************************************************
 * 
 *********************************************************************/
 
#endif
