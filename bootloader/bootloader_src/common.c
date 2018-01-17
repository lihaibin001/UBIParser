/**********************************************************************
   Title                    : Common.c
   Module Description       : This is the standard code file for PERIODIC.
   Author                   : 
   Created                  : 
   Configuration ID         : 

 *********************************************************************/

/**********************************************************************
 * Include header files
 *********************************************************************/
#include "common.h"

/**********************************************************************
 * Constant and Macro Definitions using #define
 *********************************************************************/
#define FLASH_PAGE_SIZE 0X800 // 2KB

/**********************************************************************
 * Enumerations and Structures and Typedefs
 *********************************************************************/

/**********************************************************************
 * Global and Const Variable Defining Definitions / Initializations
 *********************************************************************/

/**********************************************************************
 * Static Variables and Const Variables With File Level Scope
 *********************************************************************/
pFunction Jump_To_Application;
uint32_t JumpAddress;
uint32_t BlockNbr = 0, UserMemoryMask = 0;
bool FlashProtection = FALSE;

uint32 FlashDestination = ROM_APP_Start_Address; /* Flash user program offset */
uint16_t PageSize = 0x800; // 2k
uint32_t EraseCounter = 0x0;
uint32_t NbrOfPage = 0;
FLASH_Status FLASHStatus = FLASH_COMPLETE;
u32 RamSource;
/**********************************************************************
 * ROM Const Variables With File Level Scope
 *********************************************************************/

SW_infor_t sw_infor;
/**********************************************************************
 * Function Prototypes for Private Functions with File Level Scope
 *********************************************************************/
void JumpToApp(void)
{
    /* Test if user code is programmed starting from address "ApplicationAddress" */
    if (((*(vu32*)ROM_APP_Start_Address) & 0x2FFF0000 ) == 0x20000000)
    { /* Jump to user application */

        JumpAddress = *(vu32*) (ROM_APP_Start_Address + 4);
        Jump_To_Application = (pFunction) JumpAddress;
        /* Initialize user application's Stack Pointer */
        __MSR_MSP(*(vu32*) ROM_APP_Start_Address); 
        Jump_To_Application();
    }
}

void FLASH_GetWriteProtectionStatus(void)
{
    /* Get the number of block (4 pages) from where the user program will be loaded */
    BlockNbr = (FlashDestination - 0x08000000) >> 12;

    /* Compute the mask to test if the Flash memory, where the user program will be
       loaded, is write protected */
    UserMemoryMask = ((u32)~((1<<BlockNbr)-1));
    /* Test if any page of Flash memory where program user will be loaded is write protected */
    if ((FLASH_GetWriteProtectionOptionByte() & UserMemoryMask) != UserMemoryMask)
        FlashProtection = TRUE;
    else
        FlashProtection = FALSE;
}

void Internal_Flash_ErasePage(void)
{
    FLASH_Unlock();//add by scb
    /* Erase all the  pages where the user application will be loaded */
    /* Define the number of page to be erased */
    //NbrOfPage = FLASH_PagesMask(sw_infor.sw_size);
    //Erase the whole flash ,in case the updated falsh is smaller than the current. This may cause checksum mismatch.
    NbrOfPage=124;  // 256k -8k= 248k/2k=124 pages
    /* Erase the FLASH pages */
    for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
    {
        FLASHStatus = FLASH_ErasePage(FlashDestination + (PageSize * EraseCounter));
    }
}

void Dev_Set_upd_flag(void)
{
    uint8_t flag_data[2]={0x01,0x03};
    sFLASH_EraseSector(FLASH_DEV_START_OFFSET);
    sFLASH_WriteBuffer(flag_data,FLASH_DEV_START_OFFSET,2);
}

// 0:invalid data, shall copy from backup; 
// 1:normal and start app; 
// 2: start app fail, copy from backup
uint8_t Dev_App_Started(void)
{
    uint8_t flag_data[2];
    sFLASH_ReadBuffer(flag_data, FLASH_DEV_START_OFFSET, 2);
    if ((flag_data[0]==0x55) && (flag_data[1]==0xaa))
    {
        return 1;
    }
    else if ((flag_data[0]==0x01) && (flag_data[1]==0x03))
    {
        return 2;
    }
    else
    {
        return 0;
    }
}

void Dev_App_sw_recovery(void)
{
    uint32_t words_number;
    uint32_t sw_data;
    uint32_t NorFlashAddr=NOR_FLASH_OTA_Backup_Addr;
    uint32_t size=0x3FFE0;
    
//    if(sw_infor.sw_size>=0x3FFE0)
//    {
//        sw_infor.sw_size=0x3FFE0;  // ota header size is 0x20 in nor flash
//    }
    FlashDestination = ROM_APP_Start_Address;
//    NorFlashAddr=NOR_FLASH_OTA_Data_Addr;
    for (words_number = 0;(FlashDestination <  ROM_APP_Start_Address + size)&&(FlashDestination <  ROM_APP_end_Address);words_number += 4)
    {
        /* Read data from SPI FLASH memory */
        sFLASH_ReadBuffer((uint8_t *)&sw_data, NorFlashAddr, 4);
        /* Program the data in NorFlash into STM32F10x Flash */
        FLASH_ProgramWord(FlashDestination, sw_data);
        /* read data  and check    */
        if (*(u32*)FlashDestination != sw_data)
        {
//			retry_times++;
//			if(retry_times>3) break;
//			/* if wrong, retry  */
        }
        NorFlashAddr += 4;
        FlashDestination += 4;
    }
    /* Erase SPI FLASH first Sector of OTA area to clear init_flag */
    sFLASH_EraseSector(FLASH_DEV_START_OFFSET);
    Dev_Set_upd_flag();
}

void Dev_App_sw_upgrade(void)
{
    uint32_t words_number;
    uint32_t sw_data;
    uint32_t NorFlashAddr=NOR_FLASH_OTA_Data_Addr;
    uint32_t sw_size = (sw_infor.ota_total_bytes[0] <<24)+
                       (sw_infor.ota_total_bytes[1] <<16)+
                       (sw_infor.ota_total_bytes[2] <<8)+
                       sw_infor.ota_total_bytes[3];
    if(sw_size>=0x3FFE0)
    {
        sw_size=0x3FFE0;  // ota header size is 0x20 in nor flash
    }
    FlashDestination = ROM_APP_Start_Address;
//    NorFlashAddr=NOR_FLASH_OTA_Data_Addr;
    for (words_number = 0;(FlashDestination <  ROM_APP_Start_Address + sw_size)&&(FlashDestination <  ROM_APP_end_Address);words_number += 4)
    {
        /* Read data from SPI FLASH memory */
        sFLASH_ReadBuffer((uint8_t *)&sw_data, NorFlashAddr, 4);
        /* Program the data in NorFlash into STM32F10x Flash */
        FLASH_ProgramWord(FlashDestination, sw_data);
        /* read data  and check    */
        if (*(u32*)FlashDestination != sw_data)
        {
//			/* if wrong, retry  */
        }
        NorFlashAddr += 4;
        FlashDestination += 4;
    }
    /* Erase SPI FLASH first Sector of OTA area to clear init_flag */
    sFLASH_EraseSector(NOR_FLASH_OTA_Addr);
    Dev_Set_upd_flag();
}
/* Private functions ---------------------------------------------------------*/

#ifdef UART2_DEBUG
/*******************************************************************************
* Function Name  : SerialPutChar
* Description    : Print a character on the HyperTerminal
* Input          : - c: The character to be printed
* Output         : None
* Return         : None
*******************************************************************************/
void SerialPutChar(u8 c)
{
    USART_SendData(USART2, c);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

/*******************************************************************************
* Function Name  : SerialPutString
* Description    : Print a string on the HyperTerminal
* Input          : - s: The string to be printed
* Output         : None
* Return         : None
*******************************************************************************/
void SerialPutString(u8 *s)
{
    while (*s != '\0')
    {
        SerialPutChar(*s);
        s ++;
    }
}
#endif

#ifdef UART1_DEBUG
/*******************************************************************************
* Function Name  : SerialPutChar
* Description    : Print a character on the HyperTerminal
* Input          : - c: The character to be printed
* Output         : None
* Return         : None
*******************************************************************************/
void SerialPutChar(u8 c)
{
    USART_SendData(USART1, c);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

/*******************************************************************************
* Function Name  : SerialPutString
* Description    : Print a string on the HyperTerminal
* Input          : - s: The string to be printed
* Output         : None
* Return         : None
*******************************************************************************/
void SerialPutString(u8 *s)
{
    while (*s != '\0')
    {
        SerialPutChar(*s);
        s ++;
    }
}
#endif

/*******************************************************************************
* Function Name  : FLASH_PagesMask
* Description    : Calculate the number of pages
* Input          : - Size: The image size
* Output         : None
* Return         : The number of pages
*******************************************************************************/
u32 FLASH_PagesMask(vu32 Size)
{
    u32 pagenumber = 0x0;
    u32 size = Size;

    if((size % FLASH_PAGE_SIZE) != 0)  //page size is 2KB
    {
        pagenumber = (size / FLASH_PAGE_SIZE) + 1;
    }
    else
    {
        pagenumber = size / FLASH_PAGE_SIZE;
    }
    return pagenumber;
}

/*******************************************************************************
* Function Name  : FLASH_DisableWriteProtectionPages
* Description    : Disable the write protection of desired pages
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FLASH_DisableWriteProtectionPages(void)
{
    u32 useroptionbyte = 0, WRPR = 0;
    u16 var1 = OB_IWDG_SW, var2 = OB_STOP_NoRST, var3 = OB_STDBY_NoRST;
    FLASH_Status status = FLASH_BUSY;

    WRPR = FLASH_GetWriteProtectionOptionByte();

    /* Test if user memory is write protected */
    if ((WRPR & UserMemoryMask) != UserMemoryMask)
    {
        useroptionbyte = FLASH_GetUserOptionByte();
        UserMemoryMask |= WRPR;
        status = FLASH_EraseOptionBytes();

        if(UserMemoryMask != 0xFFFFFFFF)
            status = FLASH_EnableWriteProtection((u32)~UserMemoryMask);

        /* Test if user Option Bytes are programmed */
        if((useroptionbyte & 0x07) != 0x07)
        { /* Restore user Option Bytes */
            if((useroptionbyte & 0x01) == 0x0)
            {
                var1 = OB_IWDG_HW;
            }
            if((useroptionbyte & 0x02) == 0x0)
            {
                var2 = OB_STOP_RST;
            }
            if((useroptionbyte & 0x04) == 0x0)
            {
                var3 = OB_STDBY_RST;
            }

            FLASH_UserOptionByteConfig(var1, var2, var3);	
        }

        if (status == FLASH_COMPLETE)
        {
            #ifdef UART2_DEBUG
            SerialPutString("Write Protection disabled...\r\n");
            #endif
            //SerialPutString("...and a System Reset will be generated to re-load the new option bytes\r\n");

            /* Enable WWDG clock */
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);

            /* Generate a system Reset to re-load the new option bytes: enable WWDG and set
              counter value to 0x4F, as T6 bit is cleared this will generate a WWDG reset */
            WWDG_Enable(0x4F);
        }
        else
        {
            #ifdef UART2_DEBUG
            SerialPutString("Error: Flash write unprotection failed...\r\n");
            #endif
        }
    }
    else
    {
        #ifdef UART2_DEBUG
        SerialPutString("Flash memory not write protected\r\n");
        #endif
    }
}

/*******************(C)COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
