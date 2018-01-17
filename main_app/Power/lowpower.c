/* $Header:   lowpower.c   $*/
/**********************************************************************
 *             Title:   lowpower.C
 *
 *       Description:   This file contains micro specific code to place 
 *                      it into a low power state 
 *
 *            Author:  
 *
 *********************************************************************/

/**********************************************************************
 * Installation Instructions (periodic tasks, etc.)
 *
 *********************************************************************/

/**********************************************************************
 * Include header files
 *********************************************************************/
/* Dependent "optimize.cmd"                                          */
/*********************************************************************/
#include    "lowpower.h"

/*********************************************************************
 * File level pragmas
 *********************************************************************/
 
/*---------------------------------------------------------------------
 * pragma statements to keep all "boot" at the beginning of memory 
 *-------------------------------------------------------------------*/

/**********************************************************************
 * Constant and Macro Definitions using #define
 *********************************************************************/
#define PSMR_IDLE (0x02)  /* select idle 2 power save mode */
#define PSMR_STOP (0x01)  /* select stop power save mode */
/**********************************************************************
 * Enumerations and Structures and Typedefs
 *********************************************************************/

/**********************************************************************
 * Global and Const Variable Defining Definitions / Initializations
 *********************************************************************/

/**********************************************************************
 * Static Variables and Const Variables With File Level Scope
 *********************************************************************/
static uint8_t wkup_enable=0;
/**********************************************************************
 * ROM Const Variables With File Level Scope
 *********************************************************************/


/**********************************************************************
 * Function Prototypes for Private Functions with File Level Scope
 *********************************************************************/


/**********************************************************************
 * Add User defined functions
 *********************************************************************/

/**********************************************************************
 * Constant and Macro Definitions using #define
 *********************************************************************/
 
#ifdef STOP_OSCILLATOR
#define PSMR_REG PSMR_STOP
#else
#define PSMR_REG PSMR_IDLE
#endif

/**********************************************************************
 * Function Definitions
 *********************************************************************/

/**********************************************************************
 * Description: Places the processor into STANDBY mode
 *    If not stop oscillator  
 *       Main oscillator running 
 *       Watch timer running  (RTI / TOD)
 *
 *    PLL Stopped 
 *    CPU Clock stopped  
 *    Peripheral Clock stopped
 * 
 *    After wake-up, the PLL will still be disabled.
 *
 *  The user must configure wakeup interrupts before calling this function
 *  
 *  Parameters: None
 *     Returns: None
 *********************************************************************/
void Micro_Go_Standby(void)
{
    Enable_Interrupts();/* ensure interrupts are enabled */
#if 1
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2|RCC_APB1Periph_USART2|RCC_APB1Periph_USART3, DISABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, DISABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, DISABLE );
#endif
    if (0 != wkup_enable)
        PWR_WakeUpPinCmd(ENABLE);
    else
        PWR_WakeUpPinCmd(DISABLE);

    PWR_EnterSTANDBYMode();

    NOP();
    NOP();
    NOP();
    NOP();
    NOP();
    NOP();
    NOP();
    NOP();
}

void Micro_Go_Sleep(void)
{
	Enable_Interrupts();/* ensure interrupts are enabled */
    
    SysTick->CTRL  &= ~SysTick_CTRL_TICKINT_Msk;
    
	PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
	
	NOP();
	NOP();
	NOP();
	NOP();
	NOP();
	NOP();
	NOP();
	NOP();
    
    SysTick->CTRL  |= SysTick_CTRL_TICKINT_Msk;
    
}

void Set_Wkup_Enable(uint8_t setting)
{
    wkup_enable=setting;
}

/**********************************************************************
 *
 * Revision History
 *
 *********************************************************************
 *
 *********************************************************************/
