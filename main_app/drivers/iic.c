/*===========================================================================*\
 * FILE: iic.c
 *===========================================================================
 * DESCRIPTION:
 *   Main C file for IIC module for NEC V850ES core micros
 *   This module has the capability for five separate channels.
 *    - Channels 0, 1 are hardware IIC modules.
 *    - Channel 3 is a bit bang IIC using digital I/O if defined IIC_BB_MASTER
 *   This module performs IIC data transmisson over a two-wire (clock
 *    and data lines) bus via IIC (Inter-Integrated Circuit) Serial Bus periperal
 *    embedded in microprocessor
 *   This module is interrupt driven and will indicate busy until the xmit/read is complete
 *   The calling routine is responsible to load data to transmit into designated
 *    registers in proper order
 *   The calling routine will also be responsible for specifying the location and
 *    the number of bytes to send
 *   IIC_Read and IIC_Write will then transmit or read data before clearing the IIC as not busy
 *
 * AUTHOR:
 *   Chen Huichao
 *
\*===========================================================================*/

/**********************************************************************
 * Installation Instructions (periodic tasks, etc.)
 * Channel - Set I/O pins for the channel in IO.H
 * Ensure that RES_IIC_0, RES_IIC_1 and RES_IIC_BB are defined in rtos.c
 * and rtos.h
 *Interrupt file should be modified for IIC interrupts for channel actually used (0 or 1, )
 *  - IIC_ISR_0, IIC_ISR_1 interrupt service routines
 * DMA module must be in build 
 * - Currently only IIC writes are DMA supported.
 *
 * Before calling IIC_Read/Write, you must first call IIC_Allocate
 * After finishing call IIC_Release.
 * A parameter for various routines are the channel type defined in IIC.HU
 *********************************************************************/

/**********************************************************************
 * Include header files                                                
 *********************************************************************/
/*********************************************************************/
#include "standard.h"
#include "iic.h"
#include "delay.h"
#include "iic_sau.h" 

/**********************************************************************
 * File level pragmas                                                  
 *********************************************************************/
/**********************************************************************
 * Constant and Macro Definitions using #define                        
 *********************************************************************/
#define RESOURCE_TIMEOUT    ( MSec_To_Ticks(250) )

#define IIC_CONTROL_REG 0x0C
/*
 *            00001100b = 0x0C
 *            !!!!!!!!_______ SPT0  = 0, STOP condition OFF
 *            !!!!!!!________ STT0  = 0, START condition OFF
 *            !!!!!!_________ ACKE0 = 1, Enable ACK
 *            !!!!!__________ WTIM0 = 1, After 9th clock SCLK set to low and WAIT is set
 *            !!!!___________ SPIE0 = 1, Generate IRQ when stop condition is detected
 *            !!!____________ WREL0 = 0, Doesn't cancel wait, 1 = automatically clear wait
 *            !!_____________ LREL0 = 0, Normal operation, 1 = exit
 *            !______________ IICE0 = 1, Enable I2C bus operation
 */

#define start_or_restart(control_byte)   (0 != (control_byte & (IIC_START | IIC_RESTRT) ))
#define restart(control_byte)            (0 != (control_byte & IIC_RESTRT))
#define start(control_byte)              (0 != (control_byte & IIC_START))
#define stop(control_byte)               (0 != (control_byte & IIC_STOP))

#define IIC_INITED         0x55
#define IIC_NOT_INIT       0x00


#define ACKDn()  (0x00 == (SSR11 & 0x0002))
/*---------------------------------------------------------------------
 * The following macros are for delays for bit band IIC
 *-------------------------------------------------------------------*/

/*---------------------------------------------------------------------
 * The following defines are microprocessor dependent. They are not
 * project specific.
 *-------------------------------------------------------------------*/
#define IIC_NUM_HW_CHANNELS   1
#define IICICnIF              0x40     /* interrupt request flag */

/*---------------------------------------------------------------------
 * Define register settings for different bus speeds
 * See hardware manual for register definitions
 *-------------------------------------------------------------------*/

const Resource_Type iic_resource_ids[] =
{
   RES_IIC_0,
 //  RES_IIC_1,
};

/**********************************************************************
 * Enumerations and Structures and Typedefs                            
 *********************************************************************/
typedef struct iic_chan_info_tag
{
   uint8_t *data_ptr;                     /* pointer to data to send / receive */
   volatile uint16_t number_bytes;        /* number of bytes left to process */
   uint16_t error_count;                  /* channel error count */
   uint8_t ready;                         /* 0x55 if channel has been initalized */
   bool stop;                             // Flag to ISR to neg ack last read byte
   uint8_t reading;                          // Flag to ISR if doing a read (true) or write (false)
} iic_chan_info_t;

/*---------------------------------------------------------------------
 * The following tables are index by channel and returns hardware value
 *-------------------------------------------------------------------*/


/**********************************************************************
 * Global and Const Variable Defining Definitions / Initializations    
 *********************************************************************/
#ifndef IIC_MIN_DMA_SIZE
#define IIC_MIN_DMA_SIZE   5  /* default minimum size to do DMA transfer */ 
#endif 

/**********************************************************************
 * Static Variables and Const Variables With File Level Scope          
 *********************************************************************/
static iic_chan_info_t iic_chan_info;  

/**********************************************************************
 * ROM Const Variables With File Level Scope                           
 *********************************************************************/

/**********************************************************************
 * Function Prototypes for Private Functions with File Level Scope     
 *********************************************************************/
static void iic_send_start(void);
static void iic_send_stop(void);
static void iic_isr(uint_fast8_t chan);
/**********************************************************************
 * Add User defined functions                                          
 *********************************************************************/

/**********************************************************************
 * Function Definitions                                                
 *********************************************************************/

/**********************************************************************
 * Default definitions for Constant and Macro Definitions using #define                        
 * not set in CU file
 *********************************************************************/
 
/**********************************************************************
 *    Function: IIC_Read
 *
 *  Parameters:
 *
 *  dev_addr:  If the start condition bit is set (Bit7) then this will be sent
 *                out as the chip address
 *
 *  control:   This parameter will be used to update the IIC control register
 *                with the bits defined as follows:
 *                SPWRFxxx
 *                !!!!!___ 1 = superfast
 *                !!!!____ 1 = Restart needed
 *                !!!_____ 1 = Use slow IIC clock speed
 *                !!______ 1 = Send stop condition when all bytes have been transmitted
 *                !_______ 1 = Send start condition (1 = Send Start)
 *
 *  num_bytes: number of bytes to read
 *
 *  uint_fast8_t chan_num - channel number
 *
 *  dat_buf:   Location to store the data that is read from the slave device
 *
 *     Returns: Status_Type
 *              dat_buf:   Data that is read in will be returned into this buffer
 *                         if successful
 *
 * Description:
 *  - This subroutines perform an IIC read. Once the address of the slave
 *    device is sent out the IIC read routine will begin reading data from the
 *    slave.  The data read from the slave will be saved in buffer[]
 *    If a stop is specified in the IIC_CntrlByte parameter then on the last
 *    byte read in a negative acknowledge will be sent followed by a stop condition
 *
 ***********************************************************************/
extern Status_Type IIC_Read(uint8_t dev_addr, uint8_t control, uint16_t num_bytes,
                            void *dat_buf, uint_fast8_t chan_num)
{
   Status_Type status = E_OK;
   Tick_Type time_out;
   return(status);      
}

/***********************************************************************
 *    Function: IIC_Write
 *
 *  Parameters:
 *
 *  control:    This parameter will be used to update the IIC control register
 *                with the bits defined as follows:
 *                SPWRFxxx
 *                !!!!!___ 1 = Super Fast
 *                !!!!____ 1 = Restart needed
 *                !!!_____ 1 = Use slow IIC clock speed
 *                !!______ 1 = Send stop condition when all bytes have been transmitted
 *                !_______ 1 = Send start condition (1 = Send Start)
 *
 *  num_bytes: number of bytes to write
 *
 *  dat_buf[]:    Chip address followed by the data must be place at location 0
 *                of the buffer[]
 *
 *  uint_fast8_t chan_num - channel number
 *
 *  Returns: Status_Type
 *
 * Description:
 *  - This routine will perform an IIC write.  It will use the number passed in
 *    IIC_CntrlByte for the number of bytes and will use the buffer[] to
 *    get the data to tranmit.
 *
 ***********************************************************************/

extern Status_Type IIC_Write(uint8_t control, uint16_t num_bytes, const void  *dat_buf, uint_fast8_t chan_num)
{
   Status_Type status = E_OK;
   Tick_Type time_out;
   
   if ( chan_num < IIC_NUM_HW_CHANNELS)
   {

   }
   
   return(status);      
}

/***********************************************************************
 * Description: Opens an IIC channel
 *  Parameters: uint_fast8_t chan_num - channel number
 *     Returns: E_OK if channel was allocated successfully
 ***********************************************************************/
extern Status_Type IIC_Allocate(uint_fast8_t chan_num)
{
   Status_Type avail = E_ERROR;

   if ( PS_Awake() && (chan_num < Num_Elems(iic_resource_ids)) )
   {
      avail = OS_Wait_Resource(iic_resource_ids[chan_num], RESOURCE_TIMEOUT);
   }
   else if ( chan_num < IIC_NUM_HW_CHANNELS )       // OS not started yet (allow IIC transmission)
   {
       avail = E_OK;
   }

   return(avail);
}

/***********************************************************************
 * Description: Close an IIC channel
 *  Parameters: uint_fast8_t chan_num - channel number
 *     Returns: void
 ***********************************************************************/
extern void IIC_Release(uint_fast8_t chan_num)
{
   if ( PS_Awake() && (chan_num < Num_Elems(iic_resource_ids)) )
   {
      OS_Release_Resource(iic_resource_ids[chan_num]);
   }
   /* Do nothing if OS not started */
}

/***********************************************************************
 * Description: this routine disables the IIC hardware during idle setup
 *              for low current
 *  Parameters: void
 *     Returns: void
 ***********************************************************************/
extern void IIC_Go_Idle(void)
{
   NOP();
   NOP();
   NOP();
   NOP();
   iic_chan_info.ready = IIC_NOT_INIT;       // mark channel as not initialized  
}
/***********************************************************************
 *    Function: iic_init
 *
 *  Parameters:
 *  control:   This parameter will be used to update the IIC control register
 *                with the bits defined as follows:
 *                SPWRFxxx
 *                !!!!!___ 1 = Super Fast
 *                !!!!____ 1 = Restart needed
 *                !!!_____ 1 = Use slow IIC clock speed
 *                !!______ 1 = Send stop condition when all bytes have been transmitted
 *                !_______ 1 = Send start condition (1 = Send Start)
 *
 *  uint_fast8_t chan_num - channel number
 *
 *     Returns: void
 *
 * Description: this routine initializes the IIC hardware of the INDY1 NEC uP
 *
 ***********************************************************************/
void iic_init( uint8_t control)
{
	//iic_channel_off(chan);                 // disable channel(chan);                         // run the unlock routine
//	SAU_iic_enable_clock(chan);       //supply SAUm(iic) mode  clock
	iic_chan_info.ready = IIC_INITED;       // mark channel as not initialized  
}

/***********************************************************************
 * Description: Generate a start condition FIXME 
 *  Parameters: pointer to IIC regsisters 
 *     Returns: void
 ***********************************************************************/
static void iic_send_start( void)
{

}   

/***********************************************************************
 * Description: Generate a stop condition FIXME 
 *  Parameters: pointer to IIC regsisters 
 *     Returns: void
 ***********************************************************************/
static void iic_send_stop( void)
{

}

/***********************************************************************
 * Description: This routine handles iic channel zero interrupts
 *  Parameters: Channel number 
 *     Returns: void
 ***********************************************************************/
 uint16_t iic_error_count;
static void iic_isr(uint_fast8_t chan)
{
}

/***********************************************************************
 * Description: This routine handles iic channel zero interrupts
 *  Parameters: void (ISR)
 *     Returns: void
 ***********************************************************************/
void vIIC1_ISR_Handler (void)
{
   iic_isr(0);
}   
/**********************************************************************
*
* REVISION RECORDS
*
*********************************************************************/
/**********************************************************************
*
*
*********************************************************************/
