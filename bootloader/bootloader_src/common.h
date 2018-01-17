#ifndef _COMMON_H
#define _COMMON_H
/**********************************************************************
 *       Title:   common.h
 *  Description:  This file contains all standard exports for 
 *                common.c
 *      Author:   
 *********************************************************************/

/**********************************************************************
 * Include files                                                       
 *********************************************************************/
#include "compiler.h"
//#include "stdio.h"
//#include "string.h"
#include "stm32f10x.h"
#include "spi_flash.h"
/**********************************************************************
 * Global Constant and Macro Definitions using #define                        
 *********************************************************************/
//#define  UART2_DEBUG
//#define  UART1_DEBUG

#define  ROM_APP_Start_Address	0x08002000
#define  ROM_APP_end_Address     0x0803FFFF
#define  Nor_FLASH_Sector_size	0x00001000 // 4k
#define  NOR_FLASH_OTA_Addr     (528 * 1024)
#define  NOR_FLASH_OTA_Data_Addr   (532 * 1024)
#define  NOR_FLASH_OTA_Backup_Addr   (788 * 1024)
#define  FLASH_DEV_START_OFFSET	   (1052 * 1024)

/**********************************************************************
 * Global Enumerations and Structures and Typedefs                          
 *********************************************************************/
typedef  void (*pFunction)(void);

typedef struct
{
//	uint16_t update_flag;	/*  if update_flag=0x55AA,bootloader will upgrade the app*/
    uint8_t ota_ctrl;
    uint8_t ota_target_ver[4];
    uint8_t ota_total_bytes[4];
    uint8_t ota_total_package[2];
    uint8_t ota_current_package_index[2];
    uint8_t ota_downloaded_bytes[4];
//	uint32_t sw_size;		/* software size   */
}SW_infor_t;
/**********************************************************************
 * Global Variable extern Declarations                               
 *********************************************************************/
extern SW_infor_t sw_infor;
extern bool FlashProtection;
/**********************************************************************
 * Global Function Prototypes                                               
 *********************************************************************/
/* Exported functions ------------------------------------------------------- */
#ifdef UART2_DEBUG
void SerialPutChar(u8 c);
void SerialPutString(u8 *s);
#endif
#ifdef UART1_DEBUG
void SerialPutChar(u8 c);
void SerialPutString(u8 *s);
#endif
u32 FLASH_PagesMask(vu32 Size);
void FLASH_DisableWriteProtectionPages(void);
extern void FLASH_GetWriteProtectionStatus(void);
extern void JumpToApp(void);
extern void Internal_Flash_ErasePage(void);
extern void Dev_App_sw_upgrade(void);
extern uint8_t Dev_App_Started(void);
extern void Dev_Set_upd_flag(void);
extern void Dev_App_sw_recovery(void);
/**********************************************************************
 *
 * Revision Record
 *
 *********************************************************************
 * 
 *********************************************************************/
#endif  /* _COMMON_H */


