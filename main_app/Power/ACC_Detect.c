/* $Header:   ACC_Detect.h $*/
/*----------------------------------------------------------------------------/
*  (C)Dedao, 2017
*-----------------------------------------------------------------------------/
*
* Copyright (C) 2017, Dedao, all right reserved.
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this condition and the following disclaimer.
*
* This software is provided by the copyright holder and contributors "AS IS"
* and any warranties related to this software are DISCLAIMED.
* The copyright owner or contributors be NOT LIABLE for any damages caused
* by use of this software.
*----------------------------------------------------------------------------*/
#include "ACC_Detect.h"
#include "timer.h"
/*******************************************************************************
* Definitions
******************************************************************************/
typedef enum
{
    ACC_off = 0,
    ACC_Delay_1,
    ACC_Delay_2,
    ACC_Delay_3,
    ACC_Delay_4,
    ACC_Delay_5,
    ACC_on,
    ACC_Status_Num,
}ACC_Signal_Status;
typedef struct
{
    uint32_t            time;  //express the acc on time
    ACC_Signal_Status   status;
}ACC_Signal_StatusType;

/*   */
typedef enum
{
    ACC_Volt_Step_None = 0,
    ACC_Volt_Step1,
    ACC_Volt_Step2,
    ACC_Volt_Step3,
    ACC_Volt_Step_Num,
}ACC_Volt_Check;

/* ACC source option */
typedef enum
{
    ACC_None = 0,
    ACC_EXT_Signal,
    ACC_Volt_Pulse,
    ACC_Volt_Level,
    ACC_Source_Num,
}ACC_Source;

typedef struct
{
    // Ignition detection parameters
    uint16_t option;
    uint16_t ign_on_threshold_low;
    uint16_t ign_on_threshold_high;
    uint16_t ign_on_down_max;
    uint16_t ign_on_up_max;
    uint16_t ign_on_rise_time;
    uint16_t ign_off_down_max;
    uint16_t ign_on_abs;
    uint16_t ign_off_abs;
}ACC_Config_t;

/*******************************************************************************
* Variables
******************************************************************************/
static ACC_Signal_StatusType ACC_Signal = {0, ACC_off};

static ACC_Volt_Check ACC_volt_step = ACC_Volt_Step_None;

static ACC_Config_t     ACC_Config;

static Config_t         config_data;

static uint16_t     check_timer = 0;

/*******************************************************************************
* Code
******************************************************************************/
/*******************************************************************************
*    Function: ACC_InitializeMonitor
*
*  Parameters: none
*     Returns: none
* Description: initialize the Battery Management System monitor
*******************************************************************************/
//void ACC_InitializeMonitor(void)
//{
//    port_pin_config_t portPinConfig = {0};
//    portPinConfig.pullSelect = kPORT_PullUp;
//    portPinConfig.mux = kPORT_MuxAsGpio;
//    
//    CLOCK_EnableClock(ACC_PORT_CLK);
//    PORT_SetPinConfig(ACC_PORT, ACC_GPIO_PIN, &portPinConfig);
//    PORT_SetPinInterruptConfig(ACC_PORT, ACC_GPIO_PIN, kPORT_InterruptEitherEdge); 
//    EnableIRQ(ACC_PORT_IRQ);
//}
/*******************************************************************************
*    Function: ACC_Signal_Monitor
*
*  Parameters: none
*     Returns: none
* Description: monitor the ACC signal
*******************************************************************************/
void ACC_Signal_Monitor(void)
{
    uint8_t status = GPIO_ReadInputDataBit(ACC_GPIO, ACC_GPIO_PIN);
    
    if ((ACC_None==ACC_Config.option) || (ACC_EXT_Signal==ACC_Config.option))
    {
        
        if(ACC_Signal.status <= ACC_off || ACC_Signal.status >= ACC_Status_Num)
        {
            ACC_Signal.status = ACC_off ;
        }
        
        /* ACC status is gonging to on */
        if(false == status)
        {
            if(ACC_Signal.status == ACC_on)
            {
                return;
            }
            else
            {
                /* if current ACC's signal is  */
                ACC_Signal.status++;
            }
        }
        else /* ACC status is gonging to off */
        {
            if(ACC_Signal.status == ACC_off)
            {
                return;
            }
            else
            {
                /* if current ACC's signal is  */
                ACC_Signal.status--;
            }
        }
        
        
        if(ACC_Signal.status == ACC_on)
        {
            rl_set_acc_status(ON);
            OS_Send_Message(OS_RECORD_TASK, RECORD_EVT_ACC_ON);
            
            /* set ACC source option to ACC signal */
            if ((ACC_EXT_Signal != ACC_Config.option))    
            {
                ACC_Config.option = ACC_EXT_Signal;
                
                OS_Send_Message(OS_RECORD_TASK, RECORD_EVT_ACC_SOURCE_SET);
            }
        }
        else if(ACC_Signal.status == ACC_off)
        {
            rl_set_acc_status(OFF);
            OS_Send_Message(OS_RECORD_TASK, RECORD_EVT_ACC_OFF);
        }
        
    }
}

#if 0
/*******************************************************************************
*    Function: ACC_Is_Signal_On_After_Delay
*
*  Parameters: none
*     Returns: true express ACC is on after latch
* Description: Get the ACC status 
*******************************************************************************/
bool ACC_Is_Signal_On_After_Delay(void)
{
    if(ACC_Signal.status == ACC_on)
    {
        rl_set_acc_status(ON);
        return true;
    }
    return false;
}

/*******************************************************************************
*    Function: ACC_Is_Signal_Off_After_Delay
*
*  Parameters: none
*     Returns: true express ACC is off after latch
* Description: Get the ACC status 
*******************************************************************************/
bool ACC_Is_Signal_Off_After_Delay(void)
{
    if(ACC_Signal.status == ACC_off)
    {
        rl_set_acc_status(OFF);
        return true;
    }
    return false;
}
#endif

/*******************************************************************************
*    Function: ACC_Signal_GetCurrentSignalLevel
*
*  Parameters: none
*     Returns: true express the ACC on 
* Description: Get the current ACC signal level
*******************************************************************************/
bool ACC_Signal_GetCurrentSignalLevel(void)
{
    if(GPIO_ReadInputDataBit(ACC_GPIO, ACC_GPIO_PIN))
    {
        return false;
    }
    return true;
}

/*******************************************************************************
*    Function: ACC_Set_Source
*
*  Parameters: None
*     Returns: true express the ACC on 
* Description: Get the current ACC signal level
*******************************************************************************/
bool ACC_Set_Source(void)
{
    
//    ACC_Config.option = option;
    config_data.structData.ignition_detection_source = ACC_Config.option;

    return Set_Config(config_data);
}


/*******************************************************************************
*    Function: ACC_Voltage_Change_Monitor
*
*  Parameters: None
*     Returns: None
* Description: Monitor ACC signal by detecting external voltage changes
*******************************************************************************/
void ACC_Voltage_Change_Monitor(void)
{ 
    static uint16_t     batt_volt_max = 0;
    static uint16_t     batt_volt_min = 0;
    static uint32_t     period_slice = 0;
//    static uint16_t     check_timer = 0;
    static uint16_t     batt_volt = 0;
    static uint16_t     volt_index = 0;
    static uint16_t     batt_volt_latch[4];
    static uint16_t     batt_volt_last = 0;
    static uint32_t     latch_index = 0;
    static uint32_t     batt_volt_avg=0;
    
    batt_volt = Pwr_Fail_Get_Voltage();
#if 0
    DEBUG_PRINT1 (DEBUG_MEDIUM,"\r\n[SYSTEM]:Batt=%d V \r\n", batt_volt);
#endif
    
    latch_index = (volt_index++)%4 ;
    batt_volt_latch[latch_index++] = batt_volt;
    
    if (latch_index >=4)
    {
        latch_index = 0;
    }
    
    if(batt_volt > batt_volt_max)
    {
        batt_volt_max = batt_volt;
    }
    
    if((batt_volt < batt_volt_min) ||(batt_volt_min == 0))
    {
        batt_volt_min = batt_volt;
    }  
    
    /* if ACC detection is not initialized or by voltage pulse */
    if ((ACC_None==ACC_Config.option) || (ACC_Volt_Pulse==ACC_Config.option))
    {  
        /* check the voltage threshold */
        if ((batt_volt > ACC_Config.ign_on_threshold_high)      \
            || (batt_volt < ACC_Config.ign_on_threshold_low))
        {
            batt_volt_max = 0;
            batt_volt_min = 0;
            period_slice = 0;
            
            volt_index = 0;
            batt_volt_last = 0;
            latch_index = 0;
            
            return ;
        }
        
        /* check the voltage pulse drop*/
        if(((batt_volt_max - batt_volt_min) > ACC_Config.ign_on_down_max) \
            && (ACC_volt_step == ACC_Volt_Step_None))
        {
            batt_volt_last = batt_volt_latch[latch_index] ; 
            
            //start to check the voltage
            ACC_volt_step = ACC_Volt_Step1;
            
            batt_volt_max = batt_volt;
            batt_volt_min = batt_volt; 
            period_slice = 0;
        }
        
        switch(ACC_volt_step)
        {
          case ACC_Volt_Step1:
            {
                period_slice++;

                if((period_slice % ACC_Config.ign_on_rise_time) == 0)//   
                {
                    /*check if battery voltage , indicate that engine is starting up*/
                    if((batt_volt > batt_volt_last) && ((batt_volt - batt_volt_last) >ACC_Config.ign_on_up_max))
                    {
                        rl_set_acc_status(ON);    //ywf
                        OS_Send_Message(OS_RECORD_TASK, RECORD_EVT_ACC_ON);
                        
                        ACC_volt_step = ACC_Volt_Step2;
                        period_slice = 0;
                        
                        if ((ACC_Volt_Pulse != ACC_Config.option))    
                        {
                            ACC_Config.option = ACC_Volt_Pulse; 
                            
                            OS_Send_Message(OS_RECORD_TASK, RECORD_EVT_ACC_SOURCE_SET);   
                        }
                    }
                    else
                    {
                        ACC_volt_step = ACC_Volt_Step_None;
                        period_slice = 0;
                    }
                    
                    batt_volt_max = batt_volt;
                    batt_volt_min = batt_volt;           
                }
                else
                {
                    // do nothing
                }
            }
            break;
            
          case ACC_Volt_Step2:
            {
                period_slice ++;
                if((period_slice%150) == 0)//3s
                {
                    /*check if battery voltage delta>1V, indicate that engine is off*/
                    if((batt_volt_max - batt_volt_min) >=ACC_Config.ign_off_down_max)
                    {
                        rl_set_acc_status(OFF);    //
                        OS_Send_Message(OS_RECORD_TASK, RECORD_EVT_ACC_OFF);
                        
                        ACC_volt_step = ACC_Volt_Step_None;
                        period_slice = 0;
                        
                        batt_volt_max = batt_volt;
                        batt_volt_min = batt_volt;            
                    }  
                }
            }
            break;
          default: 
            {
                ACC_volt_step = ACC_Volt_Step_None;
            }
        }
    }
    else if (ACC_Volt_Level==ACC_Config.option)
    {
        /* ACC is detected by voltage level */
        batt_volt_avg = (batt_volt_latch[0] + batt_volt_latch[1] \
                        + batt_volt_latch[2] + batt_volt_latch[3]) >> 2;
        
        if (batt_volt_avg >= ACC_Config.ign_on_abs)
        {
            rl_set_acc_status(ON);    //ywf
            OS_Send_Message(OS_RECORD_TASK, RECORD_EVT_ACC_ON);
        }
        else if (batt_volt_avg < ACC_Config.ign_off_abs)
        {
            rl_set_acc_status(OFF);    //ywf
            OS_Send_Message(OS_RECORD_TASK, RECORD_EVT_ACC_OFF);
        }        
    }    
}

/*******************************************************************************
*    Function: ACC_Is_Volt_Check
*
*  Parameters: None
*     Returns: true: ACC is in the check process; false: not checking 
* Description: check if ACC is in checking or not
*******************************************************************************/
bool ACC_Is_Volt_Checking(void)
{
    return ((ACC_Volt_Pulse==ACC_Config.option)   \
            && (ACC_volt_step != ACC_Volt_Step_None));
}
/*******************************************************************************
*    Function: ACC_Initialize
*
*  Parameters: None
*     Returns: None 
* Description: Get the ACC setting from config
*******************************************************************************/
void ACC_initialize(void)
{
    Get_config_data(&config_data);
    
    ACC_Config.option = config_data.structData.ignition_detection_source;
    ACC_Config.ign_on_threshold_low = config_data.structData.ignition_on_threshold_low * 10;
    ACC_Config.ign_on_threshold_high = config_data.structData.ignition_on_threshold_high * 10;
    ACC_Config.ign_on_down_max = config_data.structData.ignition_on_down_max * 10;
    ACC_Config.ign_on_up_max = config_data.structData.ignition_on_rise_max * 10;
    ACC_Config.ign_on_rise_time = config_data.structData.ignition_on_rise_time;
    ACC_Config.ign_off_down_max = config_data.structData.ignition_off_down_max * 10;
    ACC_Config.ign_on_abs = config_data.structData.ignition_on_absolute_voltage * 10;
    ACC_Config.ign_off_abs = config_data.structData.ignition_off_absolute_voltage * 10;
}

/*******************************************************/
/*****************file end******************************/
