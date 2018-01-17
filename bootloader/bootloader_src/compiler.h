/* $Header:   compiler.h  $*/
/*---------------------------------------------------------------------
 *--------------------------------------------------------------------*/
/**********************************************************************
 *
 * Title                      : compiler.h
 *
 * Module Description         : This file declares compiler-specific data
 *                              that is difficult to determine automatically.
 *
 * Author                     : 
 *
 *********************************************************************/
#ifndef  _COMPILER_H
#define  _COMPILER_H 1

/* 
 * DESCRIPTION: Little Endian memory storage is LSB 1st then MSB
 * ------------------------------------------------------------------*/
#define  No_Elems(arr) (sizeof(arr) / sizeof(arr[0]))

#define  Num_Elems(arr) (sizeof(arr) / sizeof(arr[0]))
                                       /*returns the number of records in an array*/
                                        
/**********************************************************************
 *  Standard Types per C Coding Standard 
 *********************************************************************/

/* -------------------------------------------------------------------
 * Exact Width integer types
 * ---------------------------------------------------------------- */

#define UINT8_MIN    (0)
#define UINT8_MAX    (255)
#define UINT16_MIN   (0)
#define UINT16_MAX   (65535)
#define UINT32_MIN   (0)
#define UINT32_MAX   (4294967295U)
 //short and int are 16 bits type of variable for 16 bit processor
typedef unsigned char   uint8_t;       /*unsigned 8 bits*/
typedef unsigned short  uint16_t;      /*unsigned 16 bits*/
typedef unsigned long    uint32_t;      /*unsigned 32 bits*/
/*typedef unsigned long uint64_t;*/    /*NOT AVAILABLE ON V850 unsigned 64 bits*/

#define INT8_MIN     (-128)
#define INT8_MAX     (127)
#define INT16_MIN    (-32768)
#define INT16_MAX    (32767)
#define INT32_MIN    (-2147483648)
#define INT32_MAX    (2147483647)

typedef signed char     int8_t;        /*signed 8 bits*/
typedef signed short    int16_t;       /*signed 16 bits*/
typedef signed long      int32_t;       /*signed 32 bits*/

/* -------------------------------------------------------------------
 * Fastest minimum-width integer types: need check --scb
 * ---------------------------------------------------------------- */
typedef unsigned int uint_fast8_t;     // unsigned fast 8 bits
typedef unsigned int uint_fast16_t;    // unsigned fast 16 bits

typedef signed long   int_fast8_t;      // signed fast 8 bits 
typedef signed long   int_fast16_t;     // signed fast 16 bits

/* -------------------------------------------------------------------
 * Bit-field types:
 * ---------------------------------------------------------------- */
typedef unsigned char   bitfield8_t;      // unsigned 8 bits
typedef unsigned int     bitfield16_t;     // unsigned 16 bits
typedef unsigned long    bitfield32_t;     // unsigned 32 bits
// typedef unsigned long   bitfield64_t;  NOT AVAILABLE ON V850  unsigned 64 bits

/* -------------------------------------------------------------------
 * Boolean type:
 * ---------------------------------------------------------------- */
typedef unsigned int    bool;             // logical true/false (0 = false)
 
#define false  (0)
#define true   (!false)

#ifndef NULL
    #define NULL            0x00000000
#endif
/**********************************************************************
 *  Old Types
 *********************************************************************/

/* fixed-bit-width type defs */
typedef  uint8_t  uint8;                  //  unsigned 8-bit quantity 0 - 255
typedef  int8_t   sint8;                   //  signed 8-bit quantity -128 - 127

typedef  uint16_t uint16;                 //  unsigned 16-bit quantity 0 - 65535
typedef  int16_t  sint16;                  //  signed 16-bit quantity -32768 - 32767

typedef  uint32_t uint32;                 //  unsigned 32-bit quantity 0 - 4294967295
typedef  int32_t  sint32;                  //  signed 32-bit quantity -2147483648 - 2147483647

#define  FALSE          0
#define  TRUE           (!FALSE)

enum Bit_Mask_Enum                           /* define bit masks */
{
   BIT0  = 0x00000001,
   BIT1  = 0x00000002,
   BIT2  = 0x00000004,
   BIT3  = 0x00000008,
   BIT4  = 0x00000010,
   BIT5  = 0x00000020,
   BIT6  = 0x00000040,
   BIT7  = 0x00000080,
   BIT8  = 0x00000100,
   BIT9  = 0x00000200,
   BIT10 = 0x00000400,
   BIT11 = 0x00000800,
   BIT12 = 0x00001000,
   BIT13 = 0x00002000,
   BIT14 = 0x00004000,
   BIT15 = 0x00008000,
   BIT16 = 0x00010000,
   BIT17 = 0x00020000,
   BIT18 = 0x00040000,
   BIT19 = 0x00080000,
   BIT20 = 0x00100000,
   BIT21 = 0x00200000,
   BIT22 = 0x00400000,
   BIT23 = 0x00800000,
   BIT24 = 0x01000000,
   BIT25 = 0x02000000,
   BIT26 = 0x04000000,
   BIT27 = 0x08000000,
   BIT28 = 0x10000000,
   BIT29 = 0x20000000,
   BIT30 = 0x40000000,
   BIT31 = 0x80000000
};

/*-----------------------------------
  Data Structure for INT8
-----------------------------------*/

typedef struct                            /* generic type to access INT8 in BIT size */
{
   bitfield8_t Bit0 : 1;
   bitfield8_t Bit1 : 1;
   bitfield8_t Bit2 : 1;
   bitfield8_t Bit3 : 1;
   bitfield8_t Bit4 : 1;
   bitfield8_t Bit5 : 1;
   bitfield8_t Bit6 : 1;
   bitfield8_t Bit7 : 1;
}  INT8_2_Bits_Type;

typedef union                             /* generic type to access INT8 */
{
   int8_t            INT8;
   INT8_2_Bits_Type  Bits;

}  Mixed_INT8_Type;


/*-----------------------------------
  Data Structure for Word
-----------------------------------*/

typedef struct                            /* generic type to access INT16 in INT8 size */
{
   uint8_t LS;
   uint8_t MS;
}   INT16_2_INT8_Type;

typedef struct                            /* generic type to access INT16 in BIT size */
{
   bitfield16_t Bit0     :1;
   bitfield16_t Bit1     :1;
   bitfield16_t Bit2     :1;
   bitfield16_t Bit3     :1;
   bitfield16_t Bit4     :1;
   bitfield16_t Bit5     :1;
   bitfield16_t Bit6     :1;
   bitfield16_t Bit7     :1;
   bitfield16_t Bit8     :1;
   bitfield16_t Bit9     :1;
   bitfield16_t Bit10    :1;
   bitfield16_t Bit11    :1;
   bitfield16_t Bit12    :1;
   bitfield16_t Bit13    :1;
   bitfield16_t Bit14    :1;
   bitfield16_t Bit15    :1;
}  INT16_2_Bits_Type;


typedef union                             /* generic type to access INT16 */
{
   uint16_t          Int16;
   INT16_2_INT8_Type Int8;
   INT16_2_Bits_Type Bits;

}  Mixed_INT16_Type;

typedef struct                            /* generic type to access INT32 in INT16 size */
{
   uint16_t LS;
   uint16_t MS;
}  INT32_2_INT16_Type;

typedef struct                            /* generic type to access INT32 to INT8s size */
{
   uint8_t  Int8_0;
   uint8_t  Int8_1;
   uint8_t  Int8_2;
   uint8_t  Int8_3;
}  INT32_2_INT8_Type;

typedef struct                            /* generic type to access INT32 to BIT size*/
{
   bitfield32_t Bit0     :1;
   bitfield32_t Bit1     :1;
   bitfield32_t Bit2     :1;
   bitfield32_t Bit3     :1;
   bitfield32_t Bit4     :1;
   bitfield32_t Bit5     :1;
   bitfield32_t Bit6     :1;
   bitfield32_t Bit7     :1;
   bitfield32_t Bit8     :1;
   bitfield32_t Bit9     :1;
   bitfield32_t Bit10    :1;
   bitfield32_t Bit11    :1;
   bitfield32_t Bit12    :1;
   bitfield32_t Bit13    :1;
   bitfield32_t Bit14    :1;
   bitfield32_t Bit15    :1;
   bitfield32_t Bit16    :1;
   bitfield32_t Bit17    :1;
   bitfield32_t Bit18    :1;
   bitfield32_t Bit19    :1;
   bitfield32_t Bit20    :1;
   bitfield32_t Bit21    :1;
   bitfield32_t Bit22    :1;
   bitfield32_t Bit23    :1;
   bitfield32_t Bit24    :1;
   bitfield32_t Bit25    :1;
   bitfield32_t Bit26    :1;
   bitfield32_t Bit27    :1;
   bitfield32_t Bit28    :1;
   bitfield32_t Bit29    :1;
   bitfield32_t Bit30    :1;
   bitfield32_t Bit31    :1;
}  INT32_2_Bits_Type;


typedef union                             /* generic type to access INT32 */
{
    uint32_t           Int32;
    INT32_2_INT16_Type Int16;
    INT32_2_INT8_Type  Int8;
    INT32_2_Bits_Type  Bits;
}   Mixed_INT32_Type;



/**********************************************************************
 *  STANDARD MACRO FOR AUTOSAR COM                                         
 *********************************************************************/

/**********************************************************************
 *  Generic Pragma Definitions                                             
 *********************************************************************/

/**********************************************************************
 *  Global Pragmas                                                         
 *********************************************************************/

/**********************************************************************
 * Function Prototypes                                                 
 *********************************************************************/
#endif   /* #ifndef  _COMPILER_H */

/**********************************************************************
 *                 
 * REVISION RECORDS
 *                 
 *********************************************************************/
/* $Log:   compiler.h  $
 *
 * ********************************************************************
 * Date         userid    (Description on following lines: SCR #, etc.)
 * -----------  --------
 *
 * 
 *********************************************************************/
