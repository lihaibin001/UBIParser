/**
  ******************************************************************************
  * @file    CAN/DualCAN/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "uart.h"
#include "standard.h"

#include "Debug.h"

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup CAN_DualCAN
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//extern void LED_Display(uint8_t Ledstatus);

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
        SY_Cold_Start();
    }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
        SY_Cold_Start();
    }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
/*    while (1)
    {
        SY_Cold_Start();
    }*/
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
        SY_Cold_Start();
    }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

void EXTI0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0) != RESET)  // external power on  int0
    {
        /* Clear the  EXTI line 0 pending bit */
        EXTI_ClearITPendingBit(EXTI_Line0);
        Sys_Set_Ext_Batt_On_Wakeup_Flag();
    }
}

void EXTI1_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line1) != RESET)  //
    {
        /* Clear the  EXTI line 1 pending bit */
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

void EXTI2_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line2) != RESET)  // ACC
    {
        /* Clear the  EXTI line 2 pending bit */
        EXTI_ClearITPendingBit(EXTI_Line2);
        Sys_Set_Ignition_Wakeup_Flag();
                
        OS_Send_Message_FromISR(OS_SENSOR_TASK, \
            Build_Message(SENSOR_EVT_BMI160_DBE,0));
       
    }
}

//EXTI5~9 wake up interrupt
void EXTI9_5_IRQHandler(void)
{
    /* from crash sensor (BMA253) */
    if(EXTI_GetITStatus(EXTI_Line6) != RESET)   // Gsensor
    {
        /* Clear the  EXTI line 6 pending bit */
        EXTI_ClearITPendingBit(EXTI_Line6);
        
        Sys_Set_Gsensor_Wakeup_Flag();
        OS_Send_Message_FromISR(OS_SENSOR_TASK, \
                               Build_Message(SENSOR_EVT_BMA253_CRASH,0));
    }
      
    if(EXTI_GetITStatus(EXTI_Line8) != RESET) 
    {
        /* Clear the  EXTI line 8 pending bit */
        EXTI_ClearITPendingBit(EXTI_Line8);
    }
}

/**
  * @brief  This function handles USARTy global interrupt request.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        UART_RX_ISR(UART_DEBUG_CHANNEL);
    }
    else if(USART_GetITStatus(USART1, USART_IT_TC) != RESET)
    {
        UART_TX_ISR(UART_DEBUG_CHANNEL);
    }
}

void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        UART_RX_ISR(UART_GSM_CHANNEL);
    }
    else if(USART_GetITStatus(USART2, USART_IT_TC) != RESET)
    {
        UART_TX_ISR(UART_GSM_CHANNEL);
    }
}

void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        UART_RX_ISR(UART_GPS_CHANNEL);
    }
    else if(USART_GetITStatus(USART3, USART_IT_TC) != RESET)
    {
        UART_TX_ISR(UART_GPS_CHANNEL);
    }
}

void UART4_IRQHandler(void)
{
    if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
    {
        UART_RX_ISR(UART_RESERVED_CHANNEL);
    }
    else if(USART_GetITStatus(UART4, USART_IT_TC) != RESET)
    {
        UART_TX_ISR(UART_RESERVED_CHANNEL);
    }
}

/**
  * @brief  This function handles SPI1 or SPI2 global interrupt request.
  * @param  None
  * @retval None
  */
void SPI2_IRQHandler(void)
{
    if (SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_TXE) != RESET)
    {
        /* Disable SPI_MASTER TXE interrupt */
        SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, DISABLE);
    }
}


/**
  * @brief  This function handles CAN1 RX0 Handler.
  * @param  None
  * @retval None
  */
void CAN1_RX0_IRQHandler(void)
{
}

/**
  * @brief  This function handles CAN2 RX0 Handler.
  * @param  None
  * @retval None
  */

void CAN2_RX0_IRQHandler(void)
{
}

/**
  * @brief  This function handles RTC Alarm interrupt request.
  * @param  None
  * @retval None
  */

void RTCAlarm_IRQHandler(void)
{
    #ifdef USE_DEBUG
    Tick_Type time;
    time=OS_Time();

    DEBUG_PRINT1 (DEBUG_MEDIUM,"\r\n[SYSTEM]:TIME=%d ms# \r\n", time);
    #endif

    if(RTC_GetITStatus(RTC_IT_ALR) != RESET)
    {
        /* Clear EXTI line17 pending bit */
        EXTI_ClearITPendingBit(EXTI_Line17);

        /* Check if the Wake-Up flag is set */
        if(PWR_GetFlagStatus(PWR_FLAG_WU) != RESET)
        {
            /* Clear Wake Up flag */
            PWR_ClearFlag(PWR_FLAG_WU);
            //Feed_Dog();

            if(rl_get_rtc_timeout() == PS_RTC_TICK)
            {
                Sys_Set_RTC_Wakeup_Flag();
            }
            else if(rl_get_rtc_timeout() == PS_RTC_DEEP_STANDBY_TICK)
            {
	            Sys_Set_RTC_Deep_Wakeup_Flag();
            }
        }

        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();   
        /* Clear RTC Alarm interrupt pending bit */
        RTC_ClearITPendingBit(RTC_IT_ALR);
        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();

        /* Wait till RTC Second event occurs */
        //RTC_ClearFlag(RTC_FLAG_SEC);
        //while(RTC_GetFlagStatus(RTC_FLAG_SEC) == RESET);

        /* Set the RTC Alarm after 5s or 5min,depend on mode*/
        RTC_SetAlarm(RTC_GetCounter()+ rl_get_rtc_timeout());
        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();

        rl_rtc_disable();
    }
}

/**
  * @brief  This function handles RTC global interrupt request.
  * @param  None
  * @retval None
  */
void RTC_IRQHandler(void)
{
    if (RTC_GetITStatus(RTC_IT_SEC) != RESET)
    {
        /* Clear Interrupt pending bit */
        RTC_ClearITPendingBit(RTC_FLAG_SEC);
    }
}


/**
  * @brief  This function handles DMA1 Channel7 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Channel7_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_IT_TC7) != RESET)
    {
        DMA_ClearFlag(DMA1_IT_GL7);
        DMA_Cmd(DMA1_Channel7,DISABLE);
        UART_DMA_tx_Running(false);
    }
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
