#ifndef _UART_H_
#define _UART_H_
/*===========================================================================*\
 *
 * DESCRIPTION:
 *   This file has all of the standard global defines for module UART.
 *
\*===========================================================================*/


/******************************************************************************/
/* File level pragmas                                                         */
/******************************************************************************/

/******************************************************************************/
/* Constant and Macro Definitions using #define                               */
/******************************************************************************/

#define UART_ERR_OVERFLOW   (0x08)
#define UART_ERR_FRAME      (0x02)
#define UART_ERR_PARITY     (0x01)

// UARTAn control register 0 (UAnCTL0) bits
#define UART_ENABLED        0x80    // UAnPWR bit (1: Enable UARTAn operation)
#define UART_DISABLED       0x00    // UAnPWR bit (0: Disable UARTAn operation)
#define UART_TX_ENABLED     0x40    // UAnTXE bit (1: Enable transmission operation)
#define UART_TX_DISABLED    0x00    // UAnTXE bit (0: Disable transmission operation)
#define UART_RX_ENABLED     0x20    // UAnRXE bit (1: Enable reception operation)
#define UART_RX_DISABLED    0x00    // UAnRXE bit (0: Disable reception operation)
#define UART_LSB_TRANSFER   0x10    // UAnDIR bit (1: LSB-first transfer)
#define UART_MSB_TRANSFER   0x00    // UAnDIR bit (0: MSB-first transfer)
#define UART_NO_PARITY      0x00    // UAnPS1 and UAnPS0 bits (0: No parity output)
#define UART_0_PARITY       0x04    // UAnPS1 and UAnPS0 bits (1: 0 parity output)
#define UART_ODD_PARITY     0x08    // UAnPS1 and UAnPS0 bits (2: odd parity output)
#define UART_EVEN_PARITY    0x0C    // UAnPS1 and UAnPS0 bits (3: even parity output)
#define UART_8_DATA_BITS    0x02    // UAnCL bit (1: 8 data bits)
#define UART_7_DATA_BITS    0x00    // UAnCL bit (0: 7 data bits)
#define UART_2_STOP_BITS    0x01    // UAnSL bit (1: 1 stop bit)
#define UART_1_STOP_BIT     0x00    // UAnSL bit (0: 2 stop bits)

/******************************************************************************/
/* Constant and Macro Definitions using #define                               */
/******************************************************************************/
// The following #defines should be used to configure Tx/Rx buffer sizes for all UART channels
//  This allows each buffer for each channel to be a configurable size for saving RAM
//  If a channel is not used, size must be at least 1 to compile
#define UART0_RX_BUF_SIZE   150 
//#define UART0_RX_BUF_SIZE   300
#define UART0_TX_BUF_SIZE   150  // UART channel 0 is debug channel
#define UART1_RX_BUF_SIZE   1100
#define UART1_TX_BUF_SIZE   1000     //GSM
#define UART2_RX_BUF_SIZE   600   // UART channel 2 is GPS channel
#define UART2_TX_BUF_SIZE   100
#define UART3_RX_BUF_SIZE   20   // UART channel 3 is reserved
#define UART3_TX_BUF_SIZE   20

/******************************************************************************/
/* Enumerations and Structures and Typedefs                                   */
/******************************************************************************/
typedef void (*uart_rx_func_ptr) (uint8_t ch, uint8_t err);

typedef enum UART_CHANNEL_Tag
{
    UART_DEBUG_CHANNEL = 0,
    UART_GSM_CHANNEL = 1,
    UART_GPS_CHANNEL = 2,
    UART_RESERVED_CHANNEL = 3,
    UART_NUM_CHANNELS
}UART_CHANNEL_T;
/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
//#include "uart.hu"  // include application specific header definitions


/******************************************************************************/
/* Function Prototypes                                                        */
/******************************************************************************/
extern bool Uart_Put_Char (uint8_t chan, uint8_t data);
extern bool Uart_Get_Char (uint8_t chan, uint8_t* ptr);
extern void Uart_Initialize (uint8_t chan);
extern bool UART_Rx_Empty (uint8_t chan);
extern void UART_Reset_Buf(uint8_t chan);

extern void UART_TX_ISR(uint8_t chan); // Common Transmit Interrupt Service Routine
extern void UART_RX_ISR(uint8_t chan); // Common Receive Interrupt Service Routine
extern void UART_ERR_ISR(uint8_t chan);

extern uint16_t UART_DMA_tx(const char* tx_buf, uint16_t bytes);
extern void UART_DMA_tx_Running(bool status);
/******************************************************************************/
/* Function Prototypes for Private Functions with File Level Scope            */
/******************************************************************************/
extern uint8_t UART_Transmit(uint8_t channel, const uint8_t* tx_buf, uint8_t bytes);
extern void UART_ReConfig(uint32_t baudrate,uint16_t len,uint16_t stopbit,uint16_t parity);
/*=======================================================================================*\
 * File Revision History
 *=======================================================================================
 *
\*=======================================================================================*/
#endif //_UART_H_
