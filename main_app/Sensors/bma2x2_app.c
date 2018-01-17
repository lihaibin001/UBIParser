/**********************************************************************
 * Include files                                                       
 *********************************************************************/
#include "relays.h"
#include "delay.h"
#include "debug.h"
#include "bma2x2_app.h"
/**********************************************************************
 *  define
 *********************************************************************/
/* BMA253 default config parameter*/
#define BMA253_DEF_COF_RANGE        (uint8_t)0x08   //default range 8g
#define BMA253_DEF_COF_HG_THRES     (uint8_t)200    //default high-g threshold 200*faciend  

/* spi pin map */
#define prv_SPI                          SPI2
#define prv_SPI_CLK                      RCC_APB1Periph_SPI2
#define prv_SPI_SCK_PIN                  GPIO_Pin_13                  
#define prv_SPI_SCK_GPIO_PORT            GPIOB                       
#define prv_SPI_SCK_GPIO_CLK             RCC_APB2Periph_GPIOB
#define prv_SPI_MISO_PIN                 GPIO_Pin_14                 
#define prv_SPI_MISO_GPIO_PORT           GPIOB                       
#define prv_SPI_MISO_GPIO_CLK            RCC_APB2Periph_GPIOB
#define prv_SPI_MOSI_PIN                 GPIO_Pin_15                  
#define prv_SPI_MOSI_GPIO_PORT           GPIOB                       
#define prv_SPI_MOSI_GPIO_CLK            RCC_APB2Periph_GPIOB
#define prv_SPI_CS_PIN                       GPIO_Pin_12                 
#define prv_SPI_CS_GPIO_PORT                 GPIOB                       
#define prv_SPI_CS_GPIO_CLK                  RCC_APB2Periph_GPIOB
/* interrupt pin map */
#define BMA253_INT1_PORT                    GPIOC
#define BMA253_INT1_PIN                     GPIO_Pin_6
#define BMA253_INT1_GPIO_CLK                RCC_APB2Periph_GPIOC
/* function define */
#define BMA253_CS_LOW()       GPIO_ResetBits(prv_SPI_CS_GPIO_PORT, prv_SPI_CS_PIN)
#define BMA253_CS_HIGH()      GPIO_SetBits(prv_SPI_CS_GPIO_PORT, prv_SPI_CS_PIN)  

/**********************************************************************
 *  static variable                                                
 *********************************************************************/
static struct bma2x2_t bma253;
BMA253_ConfigType BMA253_conf;

/**********************************************************************
 *  static function declaration                                                
 *********************************************************************/
//static void BMA253_INT1_Init(void);
static s8 BMA253_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
static s8 BMA253_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
static s8 BMA253_SPI_burst_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u32 cnt);
static void BMA253_delay_msek(u32 msek);
static bool BMA253_fastCompensation(void);
static bool BMA253_SetConfig(void);
static void BMA253_EXTI_config(void);

/**********************************************************************
 *  global function prototype                                                
 *********************************************************************/
/**********************************************************************
 *  static function prototype                                                
 *********************************************************************/
/*******************************************************************************
*    Function: BMA253_LowLevel_Init
*
*  Parameters:  none
*     Returns:  none
* Description:  Initialize the spi bus mapped to bma253
*******************************************************************************/
void BMA253_LowLevel_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;
    
    RCC_APB2PeriphClockCmd(prv_SPI_CS_GPIO_CLK | prv_SPI_MOSI_GPIO_CLK | prv_SPI_MISO_GPIO_CLK |
                         prv_SPI_SCK_GPIO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(prv_SPI_CLK, ENABLE);
  
    GPIO_InitStructure.GPIO_Pin = prv_SPI_SCK_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(prv_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = prv_SPI_MOSI_PIN;
    GPIO_Init(prv_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = prv_SPI_MISO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
    GPIO_Init(prv_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);
  
    GPIO_InitStructure.GPIO_Pin = prv_SPI_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(prv_SPI_CS_GPIO_PORT, &GPIO_InitStructure);
    
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //24/4=6 MHz
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(prv_SPI, &SPI_InitStructure);

    SPI_Cmd(prv_SPI, ENABLE);  
//    BMA253_INT1_Init(); 
}
/*******************************************************************************
*    Function: BMA253_Init
*
*  Parameters:  None
*     Returns:  None
* Description:  initialize the bma253
*******************************************************************************/
bool BMA253_Init(void)
{
    BMA253_CS_HIGH();

    bma253.bus_write = BMA253_SPI_bus_write;
	bma253.bus_read = BMA253_SPI_bus_read;
    bma253.burst_read = BMA253_SPI_burst_read;
	bma253.delay_msec = BMA253_delay_msek;
    
    if(SUCCESS != bma2x2_init(&bma253))
    {
        DEBUG_PRINT0(DEBUG_HIGH, "[BMA253] init err!\r\n");
        return false;
    }
    
    if(!Sys_Is_Gsensor_Wakeup())
    {
        
        if(SUCCESS != bma2x2_set_power_mode(BMA2x2_MODE_NORMAL))
        {
            DEBUG_PRINT0(DEBUG_HIGH, "[BMA253] init err set power mode!\r\n");
            return false;
        }
        
        if(false == BMA253_fastCompensation())
        {
            DEBUG_PRINT0(DEBUG_HIGH, "[BMA253] init err! fast compensation\r\n");
            return false;        
        }
        
        if(false == BMA253_SetConfig())
        {
            DEBUG_PRINT0(DEBUG_HIGH, "[BMA253] init err! Set config\r\n");
            return false;          
        }    
    }
    
    BMA253_EXTI_config();
    
    return true;
}
/*******************************************************************************
*    Function: BMA253_ReadFIFOxyz
*
*  Parameters:  dst point to a buffer which store the read data
*     Returns:  the byte count read
* Description:  read axix-x/y/z data from fifo buffer
*******************************************************************************/
uint8_t BMA253_ReadFIFOxyz(int16_t *dst)
{
    uint8_t frameCnt = 0;
    uint8_t i;
    static uint8_t src[BMA253_FIFO_RANGE * 6] = {0};
    
    if(SUCCESS != bma2x2_get_fifo_frame_count(&frameCnt))
    {
        DEBUG_PRINT0(DEUBG_HIGH, "[BMA253] read fifo err!\r\n");
        return 0;
    }
    
    if(SUCCESS != bma2x2_burst_read(BMA2x2_FIFO_DATA_OUTPUT_ADDR, src, frameCnt * 6))
    {
        DEBUG_PRINT0(DEUBG_HIGH, "[BMA253] read fifo err!\r\n");
        return 0;        
    }
    
    for(i = 0; i < frameCnt * 6; i += 2)
    {
        *dst = *(int16_t *)(&src[i]) >> 4;
        dst++;
    }
    return frameCnt;
}

/*******************************************************************************
*    Function: bma253_fastCompensation
*
*  Parameters:  none
*     Returns:  true or false
* Description:  Compensate  axix-x/y for 0g and axix-z for -1g
*******************************************************************************/
static bool BMA253_fastCompensation(void)
{
    u8 cal_rdy = 0x00;
    uint16_t timeout = 0xFFFF;
    if(SUCCESS != bma2x2_set_range(BMA2x2_RANGE_2G))
    {
        return false;
    }
    
    do
    {
        if(!--timeout)
        {
            // time out return fasle
            return  false;
        }
        bma2x2_get_cal_rdy(&cal_rdy);
    }while(cal_rdy == 0);
    
    timeout = 0xFFFF;
    /* Compensate axix-x*/
    if(SUCCESS != bma2x2_set_offset_target(BMA2x2_OFFSET_TRIGGER_X, 0x00))
    {
        return false;
    }
    
    if(SUCCESS != bma2x2_set_cal_trigger(BMA2x2_OFFSET_TRIGGER_X))
    {
        return false;
    }
    
    do
    {
        if(!--timeout)
        {
            // time out return fasle
            return  false;
        }
        bma2x2_get_cal_rdy(&cal_rdy);

    }while(cal_rdy == 0);
    
    timeout = 0xFFFF;
    /* Compensate axix-x*/
    if(SUCCESS != bma2x2_set_offset_target(BMA2x2_OFFSET_TRIGGER_Y, 0x00))
    {
        return false;
    }
    
    if(SUCCESS != bma2x2_set_cal_trigger(BMA2x2_OFFSET_TRIGGER_Y))
    {
        return false;
    }
    
    do
    {
        if(!--timeout)
        {
            // time out return fasle
            return  false;
        }
        bma2x2_get_cal_rdy(&cal_rdy);

    }while(cal_rdy == 0);

    timeout = 0xFFFF;
    /* Compensate axix-z*/
    if(SUCCESS != bma2x2_set_offset_target(BMA2x2_OFFSET_TRIGGER_Z, 0x00))
    {
        return false;
    }
    
    if(SUCCESS != bma2x2_set_cal_trigger(BMA2x2_OFFSET_TRIGGER_Z))
    {
        return false;
    }
    
    do
    {
        if(!--timeout)
        {
            // time out return fasle
            return  false;
        }
        bma2x2_get_cal_rdy(&cal_rdy);

    }while(cal_rdy == 0);
       
//    /* Compensate axix-y*/
//    bma2x2_set_offset_target(2, 0x00);
//    bma2x2_set_com_trigger(2);
//    timeout = 0xFFFF;
//    do   
//    {
//        if(!--timeout)
//        {
//            // time out return fasle
//            return  false;
//        }        
//        bma2x2_get_cal_rdy(&cal_rdy);
//    }while(cal_rdy == 0);
//    /* Compensate axix-z*/
//    bma2x2_set_offset_target(3, 0x02);
//    bma2x2_set_com_trigger(3);
//    timeout = 0xFFFF;
//    do
//    {
//        if(!--timeout)
//        {
//            // time out return fasle
//            return  false;
//        }
//        bma2x2_get_cal_rdy(&cal_rdy);
//    }while(cal_rdy == 0);
    return true;
}

/*******************************************************************************
*    Function: SPI_READ_WRITE_STRING
*
*  Parameters:  pSend point to the data sent to bus
*               pReceive point to a buffer which to stored the receive data
*               cnt express data's number to be sent
*     Returns:  0 sucess, 1 fail
* Description:  send then read a string by spi bus
*******************************************************************************/
#if 0
static uint8_t SPI_READ_WRITE_STRING(uint8_t *pSend, uint8_t *pReceive, uint16_t cnt)
{
    BMA253_CS_LOW();     
    while(cnt--)
    {
        while (SPI_I2S_GetFlagStatus(prv_SPI, SPI_I2S_FLAG_TXE) == RESET);
        SPI_I2S_SendData(prv_SPI, *pSend);   
        /*!< Wait to receive a byte */
        while (SPI_I2S_GetFlagStatus(prv_SPI, SPI_I2S_FLAG_RXNE) == RESET);
        *pReceive = SPI_I2S_ReceiveData(prv_SPI);    
        pSend++;
        pReceive++;
    }
    BMA253_CS_HIGH();
    return 0;
}
#endif

/*******************************************************************************
*    Function: SPI_WRITE_STRING
*
*  Parameters:  pSend point to the data sent to bus
*               cnt express data's number to be sent
*     Returns:  0 sucess, 1 fail
* Description:  send then read a string by spi bus
*******************************************************************************/
static uint8_t SPI_WRITE_STRING(uint8_t *pSend, uint16_t cnt)
{
    BMA253_CS_LOW();
    while(cnt--)
    {
        while (SPI_I2S_GetFlagStatus(prv_SPI, SPI_I2S_FLAG_TXE) == RESET);
        SPI_I2S_SendData(prv_SPI, *pSend);
        
        while (SPI_I2S_GetFlagStatus(prv_SPI, SPI_I2S_FLAG_RXNE) == RESET);        
        SPI_I2S_ReceiveData(prv_SPI);
        
        pSend++;
    }
    BMA253_CS_HIGH();
    return 0;    
}
///*******************************************************************************
//*    Function: bma253_INT1_Init
//*
//*  Parameters:  none
//*     Returns:  none
//* Description:  initialize the exti which mapped to the bma253 interrupt 1
//*******************************************************************************/
//static void BMA253_INT1_Init(void)
//{
//    EXTI_InitTypeDef   EXTI_InitStructure;
//    GPIO_InitTypeDef   GPIO_InitStructure;
//    NVIC_InitTypeDef   NVIC_InitStructure;    
//    
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
//
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//    GPIO_Init(GPIOC, &GPIO_InitStructure);
//
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
//
//    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource6);
//
//    EXTI_InitStructure.EXTI_Line = EXTI_Line6;
//    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
//    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//    EXTI_Init(&EXTI_InitStructure);
//
//    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
//}

/*******************************************************************************
*    Function: BMA253_SetConfig
*
*  Parameters:  none
*     Returns:  none
* Description:  configure the BMA253
*******************************************************************************/
bool BMA253_SetConfig(void)
{
    //set interrupt latch mode 
    
    if(SUCCESS != bma2x2_set_latch_intr(BMA2x2_LATCH_DURN_LATCH))
    {
        return false;
    }
    //map high-g_interrupt to intr1
    if(SUCCESS != bma2x2_set_intr_high_g(BMA2x2_INTR1_HIGH_G, INTR_ENABLE))
    {
        return false;
    }
    //map fifo_full_interrutp to intr1
    if(SUCCESS != bma2x2_set_intr1_fifo_full(INTR_ENABLE))
    {
        return false;
    }

#if 0
    //map orient intrreutp to intr1
    if(SUCCESS != bma2x2_set_intr_orient(BMA2x2_INTR1_ORIENT, INTR_ENABLE))
    {
        return false;
    }
    //set orient mode to symmetrical
    if(SUCCESS != bma2x2_set_orient_mode(0))
    {
        return false;
    }
#endif
    
    //set fifo mode as stream
    if(SUCCESS != bma2x2_set_fifo_mode(0x02))
    {
        return false;
    }
    //set fifo date select X/Y/Z
    if(SUCCESS != bma2x2_set_fifo_data_select(0x00))
    {
        return false;
    }
    
    //enable high-g x interrupt
    if(SUCCESS != bma2x2_set_intr_enable(BMA2x2_HIGH_G_X_INTR, INTR_ENABLE))
    {
        return false;
    }
    //enable high-g y interrupt
    if(SUCCESS != bma2x2_set_intr_enable(BMA2x2_HIGH_G_Y_INTR, INTR_ENABLE))
    {
        return false;
    }
    //enable high-g z interrupt
    if(SUCCESS != bma2x2_set_intr_enable(BMA2x2_HIGH_G_Z_INTR, INTR_ENABLE))
    {
        return false;
    }
    //disable fifo full interrupt
    if(SUCCESS != bma2x2_set_intr_fifo_full(INTR_DISABLE))
    {
        return false;
    }
    return true;
}

#define SPI_BUFFER_LEN 5
#define BMA2x2_BUS_READ_WRITE_ARRAY_INDEX	1
#define BMA2x2_SPI_BUS_WRITE_CONTROL_BYTE	0x7F
#define BMA2x2_SPI_BUS_READ_CONTROL_BYTE	0x80

/*******************************************************************************
*    Function: BMA2x2_SPI_bus_read
*
*  Parameters:  dev_addr express the devcie address
*               reg_addr express the register address
*               reg_data point to a buffer which stored the data read out 
*               cnt express the data's number to be read
*     Returns:  0 sucess, otherwise fail
* Description:  read data from the specified register
*******************************************************************************/
#if 1
static s8 BMA253_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    BMA253_CS_LOW();
    while (SPI_I2S_GetFlagStatus(prv_SPI, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(prv_SPI, reg_addr | 0x80);
    while (SPI_I2S_GetFlagStatus(prv_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    SPI_I2S_ReceiveData(prv_SPI);
    while(cnt--)
    {
        while (SPI_I2S_GetFlagStatus(prv_SPI, SPI_I2S_FLAG_TXE) == RESET);
        SPI_I2S_SendData(prv_SPI, 0);
        while (SPI_I2S_GetFlagStatus(prv_SPI, SPI_I2S_FLAG_RXNE) == RESET);
        *reg_data = SPI_I2S_ReceiveData(prv_SPI);
        reg_data++;
    }
    BMA253_CS_HIGH();   
    return 0;
}
#endif
#if 0
static s8 BMA253_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMA2x2_INIT_VALUE;
	u8 array[SPI_BUFFER_LEN] = {0xFF};
	u8 stringpos;
	array[BMA2x2_INIT_VALUE] = reg_addr|BMA2x2_SPI_BUS_READ_CONTROL_BYTE;
    SPI_READ_WRITE_STRING(array, array, cnt+1);
	for (stringpos = BMA2x2_INIT_VALUE; stringpos < cnt; stringpos++) {
		*(reg_data + stringpos) = array[stringpos +
		BMA2x2_BUS_READ_WRITE_ARRAY_INDEX];
	}
	return (s8)iError;
}
#endif

/*******************************************************************************
*    Function: BMA2x2_SPI_burst_read
*
*  Parameters:  dev_addr express the devcie address
*               reg_addr express the register address
*               reg_data point to a buffer which stored the data read out 
*               cnt express the data's number to be read
*     Returns:  0 sucess, otherwise fail
* Description:  read data  from the specified register
*******************************************************************************/
static s8 BMA253_SPI_burst_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u32 cnt)
{
    BMA253_CS_LOW();
    while (SPI_I2S_GetFlagStatus(prv_SPI, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(prv_SPI, reg_addr | 0x80);
    while (SPI_I2S_GetFlagStatus(prv_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    SPI_I2S_ReceiveData(prv_SPI);
    while(cnt--)
    {
        while (SPI_I2S_GetFlagStatus(prv_SPI, SPI_I2S_FLAG_TXE) == RESET);
        SPI_I2S_SendData(prv_SPI, 0);
        while (SPI_I2S_GetFlagStatus(prv_SPI, SPI_I2S_FLAG_RXNE) == RESET);
        *reg_data = SPI_I2S_ReceiveData(prv_SPI);
        reg_data++;
    }
    BMA253_CS_HIGH();   
    return 0;
}
/*******************************************************************************
*    Function: BMA2x2_SPI_bus_write
*
*  Parameters:  dev_addr express the devcie address
*               reg_addr express the register address
*               reg_data point to a buffer which stored the data read out 
*               cnt express the data's number to be read
*     Returns:  0 sucess, otherwise fail
* Description:  write data to the specified register
*******************************************************************************/
static s8 BMA253_SPI_bus_write(uint8_t dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	int32_t iError = BMA2x2_INIT_VALUE;
	uint8_t array[SPI_BUFFER_LEN * 2];
	uint8_t stringpos = BMA2x2_INIT_VALUE;
    
	for (stringpos = BMA2x2_INIT_VALUE; stringpos < cnt; stringpos++) {
		array[stringpos * 2] = (reg_addr++) & \
                                BMA2x2_SPI_BUS_WRITE_CONTROL_BYTE;
		array[stringpos * 2 + BMA2x2_BUS_READ_WRITE_ARRAY_INDEX] = \
                                                *(reg_data + stringpos);
	}
    iError = SPI_WRITE_STRING(array, cnt*2);
	return (int8_t)iError;
}
/*******************************************************************************
*    Function: BMA2x2_delay_msek
*
*  Parameters:  msek express msek millisecond 
*     Returns:  none
* Description:  delay mseck ms
*******************************************************************************/
static void BMA253_delay_msek(uint32_t msek)
{
    OS_Sleep(msek);
}

/**
  * @brief  Configures EXTI Lines.
  * @param  None
  * @retval None
  */
static void BMA253_EXTI_config(void)
{
    EXTI_InitTypeDef    EXTI_InitStructure;
    NVIC_InitTypeDef    NVIC_InitStructure; 

#if 1
    /* Gsensor */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource6);
    EXTI_InitStructure.EXTI_Line = EXTI_Line6;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif 
    

}

