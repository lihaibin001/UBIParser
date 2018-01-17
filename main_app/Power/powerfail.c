/*===========================================================================*/
/**
 * @file powerfail.c
 *
 *
 */
/*==========================================================================*/

/*===========================================================================*
 * Header Files
 *===========================================================================*/ 
#include    "standard.h"
#define USE_DEBUG
#include "Debug.h"
#include "TelmProtocol.h"

/*===========================================================================*
 * Local Preprocessor #define MACROS
 *===========================================================================*/
#define write_timer(x, y)           (timer[x] = y)
#define read_timer(x)               (timer[x])
#define timer_running(x)            (timer[x] > OS_Time())

#define AD_LVW_THRESHOLD            voltage_limits[V_LOW].lower_voltage
#define AD_RECOVERY_THRESHOLD            voltage_limits[V_LOW].upper_voltage

#define AD_EXT_BATTERY 13 /* battery voltage channel */
#define AD_INT_BATTERY 1 /* internal battery voltage channel */

#define EXT_AD_GAIN() 1804 //for HW  =(65520*(10/(100+10)))/(3.3) ,theoretical value
#define INT_AD_GAIN() 6823 //for HW  =(65520*(100/(100+191)))/(3.3) ,theoretical value


//
#define VE_WORK_LOW              (1150)
#define VI_CHARGE_LOW            (400)
//#define INT_BATT_CHARGE_LIMIT_HIGH  (450)
//#define INT_BATT_CHARGE_CHECK       (0)


#define MAX_DEBOUNCE_WARNING_COUNT          12  

/*===========================================================================*
 * Local Type Declarations
 *===========================================================================*/
enum
{
    MUTE_DELAY_TIMER,
    NUM_PF_TIMERS
};

enum
{
    V_EXT_LOW,
    V_EXT_HIGH,
    V_INT_LOW,
    V_INT_HIGH,
    V_IGN_PULSE,  
    NUM_MAX_VOLTAGE_TRESHOLD
};

typedef struct PF_Voltage_Treshold_Limits_Tag
{
    uint16_t  lower_voltage;
    uint16_t  upper_voltage;
} PF_Voltage_Treshold_Limits_Type;

typedef struct PF_Voltage_Treshold_Data_Tag
{
    uint8_t   count;
    bool      lower_voltage_is;
} PF_Voltage_Treshold_Data_Type;

/*===========================================================================*
 * Local Object Definitions
 *===========================================================================*/
static const PF_Voltage_Treshold_Limits_Type voltage_limits[NUM_MAX_VOLTAGE_TRESHOLD] =
{
    { 600,  650 },  /* V_EXT_LOW          threshold */
    { 3000, 3050},  /* V_EXT_HIGH         threshold */
    { 360,  380 },  /* V_INT_LOW          threshold */
    { 500,  520}    /* V_INT_HIGH         threshold */
//    { 0, 0}  /* V_IGN_PULSE    threshold */
};

static PF_Voltage_Treshold_Data_Type voltage_data[NUM_MAX_VOLTAGE_TRESHOLD] =
{
    {MAX_DEBOUNCE_COUNT, false},    /* V_EXT_LOW */
    {MAX_DEBOUNCE_COUNT, true },    /* V_EXT_HIGH */
    {MAX_DEBOUNCE_COUNT, false},    /* V_INT_LOW */
    {MAX_DEBOUNCE_COUNT, true }     /* V_INT_HIGH */
};

//static bool pwr_fail_mute_is;
static bool pwr_fail_reset_is;
static Tick_Type timer[NUM_PF_TIMERS];

static uint32_t   ext_batt_volt_avg;
static uint16_t   ext_batt_volt_avg_last;

static uint32_t   int_batt_volt_avg;
static uint16_t   int_batt_volt_avg_last;

static uint16_t   pwr_int_bat_charge_request;   //add

/*===========================================================================*
 * Local Function Prototypes
 *===========================================================================*/
static void Pwr_Fail_Voltage_Hysteresis(uint16_t voltage, uint8_t index);
//bool Pwr_Fail_Is_Mute_Condition(void);
//bool Pwr_Fail_Is_Reset_Condition (void);

/*===========================================================================*
 * Function Definitions
 *===========================================================================*/

/**********************************************************************
*
*    Function: Pwr_Fail_Initialize
*
*  Parameters: none
*
*     Returns: none
*
* Description: initializes powerfail monitor module
*
**********************************************************************/
void Pwr_Fail_Initialize(void)
{
    /* configure AD-LVW interrupt */
    //AD_Interrupt_Configure();
    //AD_Interrupt_Enable();
}

/**********************************************************************
*
*    Function: Pwr_Fail_Shutdown
*
*  Parameters: none
*
*     Returns: none
*
* Description: This function configures powerfail module for idle mode.
*
*********************************************************************/
void Pwr_Fail_Shutdown(void)
{
    /* AD conversion interrupt does not work in STOP! */
    //AD_Interrupt_Disable();
}

/**********************************************************************
*
*    Function: Pwr_Fail_Monitor
*
*  Parameters: none
*
*     Returns: none
*
* Description: This function handles all powerfail cases.
*
*********************************************************************/
void Pwr_Fail_Monitor(void)
{
    Pwr_Fail_Check_Voltage();
    // Check RTC counter
    // Read flash to see if full
    // If not full, save current RTC count and voltage into flash
    // If RTC count greater than 1387584000, save count
    // Save voltage with 4 interval(in seconds): 300, 3600, 7200, 14400
    // Only save data when acc off
    /* check powerfail-mute: voltage range that allows mute and stops devices */
//    if (pwr_fail_mute_is)
//    {
//        /* powerfail mute still active? */
//        if (Pwr_Fail_Is_Mute_Condition()== false)
//        {
//            /* reenable powerfail mute monitoring */
//            pwr_fail_mute_is = false;
//
//            /* inform RELAYS */
//            RL_Set_Pwr_Fail_Detected(pwr_fail_mute_is);
//        }
//        else
//        {
//            /* retrigger MUTE_DELAY_TIMER */
//            write_timer(MUTE_DELAY_TIMER, OS_Time() + MUTE_OFF_WAIT_TICKS);
//        }
//    }
//    else
//    {
//        if (Pwr_Fail_Is_Mute_Condition())
//        {
//            if((!PS_Full_System())
//		      &&(Sys_Is_RTC_Deep_Wakeup()))
//            {
//                Sys_Clear_Wakeup_Src_Flags();
//                Sys_Set_Low_Batt_Wakeup_Flag();
//                SY_Warm_Start();
//                pwr_fail_mute_is = true; 
//            }
//            /* trigger MUTE_DELAY_TIMER , delay 500ms unmute operation after recover from power fail*/
//            write_timer(MUTE_DELAY_TIMER, OS_Time() + MUTE_OFF_WAIT_TICKS);
//
//            /* inform RELAYS */
//            RL_Set_Pwr_Fail_Detected(pwr_fail_mute_is);
//        }
//        else
//        {
//            if (TMR_Is_Timer_Active(TELM_BATT_CHECK_TIMER))
//            {
//                TMR_Stop_Timer(TELM_BATT_CHECK_TIMER);
//            }
//        }
//    }

//    /* check powerfail-reset: voltage range that allows reset (warm start) */
//    if (pwr_fail_reset_is)
//    {
//        /* powerfail reset still active? */
//    }
//    else
//    {
//        if (Pwr_Fail_Is_Reset_Condition())
//        {
//            pwr_fail_reset_is = true;
//            /* inform RELAYS */
//            RL_Set_Pwr_Fail_Detected(pwr_fail_reset_is);
//        }
//    }
    
    if (Pwr_Fail_Is_Int_Battery())
    {
      Sys_Set_Int_battery_Flag();
    }
    else if (Sys_Get_Int_battery_Flag())    
    {
      if(Pwr_Fail_AD_get_Voltage() > voltage_limits[V_EXT_LOW].lower_voltage)
      {
#ifdef USE_DEBUG
        DEBUG_PRINT0(DEBUG_HIGH,"[Power]:Warm restart for ext battery on \n\r");
#endif        
        
        Sys_Clear_Int_battery_Flag();
        
        /* warm restart when power changes from internal battery to external */
        Sys_Clear_Wakeup_Src_Flags();
        Sys_Clear_Standby_Req_Flag();
        SY_Warm_Start();
 

      }
    }
    else
    {
      /* do nothing */
    }
    
}

/**********************************************************************
*
*    Function: Pwr_Fail_AD_get_Voltage
*
*  Parameters: none
*
*     Returns: void
*
* Description: get  the AD convertion result and convert to the value fo voltage.
*              example:1200 means 12.00v
**********************************************************************/
uint16_t Pwr_Fail_AD_get_Voltage(void)
{
    uint16_t batt_voltage;

    batt_voltage = AD_Read(AD_EXT_BATTERY);
#if 0
    DEBUG_PRINT1 (DEBUG_MEDIUM,"\r\n[SYSTEM]:EXT VOL AD=%d \r\n", batt_voltage);
#endif
    if (batt_voltage==0)
    {
        return 0;
    }
    else
    {
        batt_voltage = (((uint32_t)(batt_voltage)) * 100)/(EXT_AD_GAIN());
        return batt_voltage;
    }
}

uint16_t Pwr_Fail_AD_get_Int_Voltage(void)
{
    uint16_t batt_voltage;

    batt_voltage = AD_Read(AD_INT_BATTERY);
    if (batt_voltage==0)
    {
        return 0;
    }
    else
    {
        batt_voltage = (((uint32_t)(batt_voltage)) * 100)/(INT_AD_GAIN());
        return batt_voltage;
    }
}

/**********************************************************************
*
*    Function: Pwr_Fail_Get_Voltage
*
*  Parameters: none
*
*     Returns: void
*
* Description: It is the average value of battery voltage in 20ms
**********************************************************************/
uint16_t Pwr_Fail_Get_Voltage(void)
{
    return ext_batt_volt_avg_last;
}

/**********************************************************************
*
*    Function: Pwr_Fail_Get_Int_Voltage
*
*  Parameters: none
*
*     Returns: void
*
* Description: It is the average value of battery voltage in 20ms
**********************************************************************/
uint16_t Pwr_Fail_Get_Int_Voltage(void)
{
    return int_batt_volt_avg_last;
}

/**********************************************************************
*
*    Function: Pwr_Fail_Check_Voltage
*
*  Parameters: none
*
*     Returns: void
*
* Description: Determine's if the voltage is above/below a certain votage.
*
**********************************************************************/
void Pwr_Fail_Check_Voltage(void)
{
    static uint32_t period_slice = 0;
    uint16_t ext_batt_voltage;
    uint16_t int_batt_voltage;

    ext_batt_voltage = Pwr_Fail_AD_get_Voltage();
    ext_batt_volt_avg += ext_batt_voltage;
    int_batt_voltage = Pwr_Fail_AD_get_Int_Voltage();
//    int_batt_voltage=0;
    int_batt_volt_avg += int_batt_voltage;
    
    period_slice ++;
    if(period_slice%4 == 0)//20ms
    {
        ext_batt_volt_avg_last = (ext_batt_volt_avg >> 2);
        ext_batt_volt_avg = 0;

        int_batt_volt_avg_last = (int_batt_volt_avg >> 2);
        int_batt_volt_avg = 0;
        
//        if((!PS_Full_System())
//          &&(Sys_Is_RTC_Deep_Wakeup()))
//        {
//
//        }
    }

    /* first check all LOW thresholds */
    Pwr_Fail_Voltage_Hysteresis(ext_batt_voltage, V_EXT_LOW   );
    Pwr_Fail_Voltage_Hysteresis(int_batt_voltage, V_INT_LOW   );

    /* then check all HIGH thresholds */
    Pwr_Fail_Voltage_Hysteresis(ext_batt_voltage, V_EXT_HIGH   );
    Pwr_Fail_Voltage_Hysteresis(int_batt_voltage, V_INT_HIGH   );

    /* then check ign thresholds on AN_BATTERY line */
//    Pwr_Fail_Voltage_Hysteresis(batt_voltage, V_IGN_PULSE   );
}

/**********************************************************************
*
*    Function: Pwr_Fail_Voltage_Hysteresis
*
*  Parameters: uint16_t voltage - voltage to process
*              uint8_t index    - index to voltage threshold table
*
*     Returns: none
*
* Description: runs the voltage hysteresis
*
**********************************************************************/
static void Pwr_Fail_Voltage_Hysteresis (uint16_t voltage, uint8_t index)
{
    uint8 debounce = 0;

    if(index == (uint8_t)V_IGN_PULSE)
        debounce = MAX_DEBOUNCE_WARNING_COUNT;
    else
        debounce = MAX_DEBOUNCE_COUNT;

    if (voltage_data[index].lower_voltage_is == false)
    {
        if (voltage < voltage_limits[index].lower_voltage)
        {/*count down if voltage below */
            if (!voltage_data[index].count--)
            {
                voltage_data[index].count = debounce;
                voltage_data[index].lower_voltage_is = true;
            }
        }
        else if (voltage_data[index].count < debounce)
        {/* count up if voltage above */
            voltage_data[index].count++;
        }
    }
    else
    {
        if (voltage > voltage_limits[index].upper_voltage)
        {/*count down if voltage above */
            if (!voltage_data[index].count--)
            {
                voltage_data[index].count = debounce;
                voltage_data[index].lower_voltage_is = false;
            }
        }
        else if (voltage_data[index].count < debounce)
        {/*count up if voltage below */
            voltage_data[index].count++;
        }
    }
}

/**********************************************************************
*
*    Function: Pwr_Fail_Is_Mute_Condition
*
*  Parameters: none
*
*     Returns: true/false
*
* Description: returns powerfail mute condition
*
**********************************************************************/
bool Pwr_Fail_Is_Mute_Condition(void)
{
    //return((voltage_data[MUTE_LOW].lower_voltage_is  == true) ||
    //       (voltage_data[MUTE_HIGH].lower_voltage_is == false));
    //return(voltage_data[MUTE_LOW].lower_voltage_is  == true) ;
  return false;
}

/**********************************************************************
*
*    Function: Pwr_Fail_Is_Reset_Condition
*
*  Parameters: none
*
*     Returns: true/false
*
* Description: returns powerfail reset condition
*
**********************************************************************/
bool Pwr_Fail_Is_Reset_Condition (void)
{
  return((voltage_data[V_EXT_HIGH].lower_voltage_is == false) \
        || (voltage_data[V_INT_HIGH].lower_voltage_is == false));
}

/**********************************************************************
*
*    Function: Pwr_Fail_Is_Shutdown
*
*  Parameters: none
*
*     Returns: true/false
*
* Description: returns powerfail and shut down condition
*
**********************************************************************/
bool Pwr_Fail_Is_Shutdown(void)
{
  return(((voltage_data[V_EXT_LOW].lower_voltage_is  == true) 
            ||(voltage_data[V_EXT_HIGH].lower_voltage_is == false))
         && ((voltage_data[V_INT_LOW].lower_voltage_is  == true) 
             ||(voltage_data[V_INT_HIGH].lower_voltage_is == false))
           );
}

/**********************************************************************
*
*    Function: Pwr_Fail_Is_Voltage_Good
*
*  Parameters: None
*
*     Returns: true/false
*
* Description: returns if the voltage is over a working threshold
*
*********************************************************************/
bool Pwr_Fail_Is_Voltage_Good (void)
{
    //return(!Pwr_Fail_Is_Mute_Condition() && !timer_running(MUTE_DELAY_TIMER));
  return true;
}

/**********************************************************************
*
*    Function: Pwr_Fail_AD_ISR
*
*  Parameters: -
*
*     Returns: -
*
* Description: Interrupt Service Routine for Low Voltage Warning.
*              This interrupt is activated when the battery sense line
*              goes low.
*              A flag is set and the relay resoure is released.
*              The A/D interrupt be disabled to reduce the system load.
*              It will be reenabled if the voltage is in an allowed range.
*
**********************************************************************/
//void Pwr_Fail_AD_ISR(void)
//{
//    AD_Interrupt_Disable();
//
//    /* low voltage detected */
//    pwr_fail_reset_is = true;
//    pwr_fail_mute_is  = true;
//
//    /* set hysteresis state machine to start checking if U > 6.5V ! */
//    voltage_data[V_LOW].count = MAX_DEBOUNCE_COUNT;
//    voltage_data[V_LOW].lower_voltage_is = true;
//
//    voltage_data[MUTE_LOW].count = MAX_DEBOUNCE_COUNT;
//    voltage_data[MUTE_LOW].lower_voltage_is = true;
//
//    /* inform RELAYS */
//    /* trigger RELAYS task when powerfail status is updated */
//    OS_Release_Resource(RES_RELAYS);
//}


/**********************************************************************
*
*   Function: Pwr_Charge_Monitor
*
*   Parameters: None
*
*   Returns: None
*
*   Description: internal battery charging monitor
*
*********************************************************************/
void Pwr_Charge_Monitor (void)
{
    static uint32_t period_slice = 0;
    uint16_t vol_ext;
    static uint16_t vol_int=0;

    vol_int += Pwr_Fail_Get_Int_Voltage();
    
    period_slice ++;
    if(period_slice%8 == 0)//2.56s
    {
        vol_ext = Pwr_Fail_Get_Voltage();
        vol_int = (vol_int >> 3);
        if ((vol_int < VI_CHARGE_LOW) && (vol_ext >= VE_WORK_LOW))
        {
          pwr_int_bat_charge_request = true;
        }
        else
        {
          pwr_int_bat_charge_request = false;
        }
        vol_int = 0;
    }
}


/**********************************************************************
*
*   Function: Pwr_Is_Charge_Request
*
*   Parameters: None
*
*   Returns: true:request charge; false: not request
*
*   Description: internal battery charging monitor
*
*********************************************************************/
bool Pwr_Is_Charge_Request (void)
{
    return (pwr_int_bat_charge_request == true);
}

/**********************************************************************
*
*    Function: Pwr_Fail_Is_Int_Battery
*
*  Parameters: none
*
*     Returns: true/false
*
* Description: returns true if internal battery is used and external power down
*
**********************************************************************/
bool Pwr_Fail_Is_Int_Battery(void)
{
    return((voltage_data[V_EXT_LOW].lower_voltage_is  == true) \
            && (voltage_data[V_INT_LOW].lower_voltage_is == false));
}

/**********************************************************************
 *
 * Revision History
 *
 *********************************************************************
 *
 *********************************************************************/
