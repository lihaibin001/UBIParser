#ifndef  _IIC_H_
#define  _IIC_H_ 
/*===========================================================================*\
 * FILE: iic.h
 *===========================================================================
 *
 * DESCRIPTION:
 *   
 *
 * AUTHOR:
 *   Chen Huichao
 *
\*===========================================================================*/

/*---------------------------------------------------------------------
 *   Instructions for using this module if any:
 *
 *   Add Interrupt Handler for each hardware channel
 *   used.
 *
 * ------------------------------------------------------------------*/

/**********************************************************************
 * Include files                                                       
 *********************************************************************/

/**********************************************************************
 * Constant and Macro Definitions using #define                        
 *********************************************************************/

#define IIC_MIN_DMA_SIZE     500

//#define IIC_Send_Stop(chan) IIC_Write( IIC_STOP, 0, NULL, (chan) )

/*---------------------------------------------------------------------
 * Control byte constants
 *****WARNING - DO NOT CHANGE THE VALUES OF THESE #defines****
 * ------------------------------------------------------------------*/
#define READ_RSTRT_BIT_MASK   0xB8
#define IIC_START             0x80
#define IIC_STOP              0x40
#define IIC_RESTRT            0x10
//IIC_SLOW - fxx/330 => ~96 KHz @ 32 MHz
#define IIC_SLOW              0x20
//IIC_FAST - fxx/330 => ~96 KHz @ 32 MHz.
//IIC_SUPER_FAST - fxx/90 => ~355 KHz @ 32 MHz.
#define IIC_FAST              0x08
#define IIC_SUPER_FAST        IIC_FAST
#define IIC_SENDED_ADDRESS_FLAG 0x02
/*---------------------------------------------------------------------
 * Device address modifier for reads
 * ------------------------------------------------------------------*/
#define IIC_READ              0x01 

/**********************************************************************
 * Enumerations and Structures and Typedefs
 *********************************************************************/

/**********************************************************************
 * Global Variable extern Declarations
 *********************************************************************/

/**********************************************************************
 * Function Prototypes
 *********************************************************************/
extern Status_Type IIC_Allocate(uint_fast8_t chan_num);

extern void IIC_Release(uint_fast8_t chan_num);

extern void IIC_Go_Idle(void);

extern Status_Type IIC_Read(uint8_t dev_addr,
                            uint8_t control,
                            uint16_t num_bytes,
                            void *  dat_buf, 
                            uint_fast8_t chan_num);


extern Status_Type IIC_Write( uint8_t control,
                              uint16_t num_bytes,
                              const void * dat_buf,
                              uint_fast8_t chan_num);

extern void iic_init(uint8_t control);
/*---------------------------------------------------------------------
 *  Interrupt handlers called from interrupt
 * ------------------------------------------------------------------*/
extern void vIIC1_ISR_Handler (void); //channel zero
//extern void IIC_ISR_1(void); //channel one

/*===========================================================================*\
 * File Revision History
 *===========================================================================
 *
\*===========================================================================*/
#endif  // _IIC_H_
