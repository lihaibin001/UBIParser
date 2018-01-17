/**********************************************************************
 * Include files                                                       
 *********************************************************************/
#include "bmi160_app.h"
//#include "relays.h"
#include "delay.h"
#include "rtos.h"

/**********************************************************************
 *  define
 *********************************************************************/
#define USE_DEBUG
#include "debug.h"

/* BMI160 default config parameter*/
#define BMI160_DEF_COF_RANGE        (uint8_t)0x08   //default range 8g
#define BMI160_DEF_COF_HG_THRES     (uint8_t)200    //default high-g threshold 200*faciend  

/* spi pin map */
#define prv_SPI                          SPI3
#define prv_SPI_CLK                      RCC_APB1Periph_SPI3
#define prv_SPI_SCK_PIN                  GPIO_Pin_10                  
#define prv_SPI_SCK_GPIO_PORT            GPIOC                       
#define prv_SPI_SCK_GPIO_CLK             RCC_APB2Periph_GPIOC
#define prv_SPI_MISO_PIN                 GPIO_Pin_11               
#define prv_SPI_MISO_GPIO_PORT           GPIOC                       
#define prv_SPI_MISO_GPIO_CLK            RCC_APB2Periph_GPIOC
#define prv_SPI_MOSI_PIN                 GPIO_Pin_12                 
#define prv_SPI_MOSI_GPIO_PORT           GPIOC                      
#define prv_SPI_MOSI_GPIO_CLK            RCC_APB2Periph_GPIOC
#define prv_SPI_CS_PIN                   GPIO_Pin_15                 
#define prv_SPI_CS_GPIO_PORT             GPIOA                       
#define prv_SPI_CS_GPIO_CLK              RCC_APB2Periph_GPIOA
/* interrupt pin map */
#define BMI160_INT1_PORT                 GPIOD
#define BMI160_INT1_PIN                  GPIO_Pin_2
#define BMI160_INT1_GPIO_CLK             RCC_APB2Periph_GPIOD
/* function define */
#define BMI160_CS_LOW()       GPIO_ResetBits(prv_SPI_CS_GPIO_PORT, prv_SPI_CS_PIN)
#define BMI160_CS_HIGH()      GPIO_SetBits(prv_SPI_CS_GPIO_PORT, prv_SPI_CS_PIN)  

/**********************************************************************
 *  static variable                                                
 *********************************************************************/
static struct bmi160_dev bmi160;
//BMI160_ConfigType BMI160_conf;
static  uint8_t fifo_buff[BMI160_FIFO_DATA_SIZE];   //84 frames, and each frame has 12 bytes
static  struct bmi160_fifo_frame fifo_frame;

static struct bmi160_sensor_data   accel_data[BMI160_FIFO_FRAME_NUM];
static struct bmi160_sensor_data   gyro_data[BMI160_FIFO_FRAME_NUM];


/**********************************************************************
 *  static function declaration                                                
 *********************************************************************/
static int8_t BMI160_SPI_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t cnt);
static int8_t BMI160_SPI_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t cnt);
static void BMI160_delay_msek(uint32_t msek);
static int8_t BMI160_fastCompensation(void);
static int8_t BMI160_Set_Int_Config(void);
static void BMI160_EXTI_config(void);
static int8_t BMI160_Set_Accel_Gyo_Config(void);
static int8_t BMI160_Set_FIFO_Config(void);

/**********************************************************************
 *  global function prototype                                                
 *********************************************************************/


/**********************************************************************
 *  static function prototype                                                
 *********************************************************************/
/*******************************************************************************
*    Function: BMI160_LowLevel_Init
*
*  Parameters:  none
*     Returns:  none
* Description:  Initialize the spi bus mapped to bmi160
*******************************************************************************/
void BMI160_LowLevel_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;
 
    GPIO_PinRemapConfig(GPIO_Remap_SPI3, ENABLE);
    
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
}
/*******************************************************************************
*    Function: BMI160_Initialize
*
*  Parameters:  None
*     Returns:  None
* Description:  initialize the bmi160
*******************************************************************************/
bool BMI160_Initialize(void)
{
    bmi160.interface = BMI160_SPI_INTF;
    bmi160.read = BMI160_SPI_bus_read;
	bmi160.write = BMI160_SPI_bus_write;
	bmi160.delay_ms = BMI160_delay_msek;
    
    BMI160_CS_LOW(); 
    uDelay(1);
    BMI160_CS_HIGH();   // make sure there is a rising edege of CSB to use SPI
    
    /* Initialize bmi160 and check the SPI communication */
    if(BMI160_OK != bmi160_init(&bmi160))
    {
        DEBUG_PRINT0(DEBUG_HIGH, "[BMI160] init err!!!!!!!!!!!!!!!!!!!!\r\n");
        return false;
    }
    
    /* Set the sensor configuration */
    if(BMI160_OK != BMI160_Set_Accel_Gyo_Config())
    {
        DEBUG_PRINT0(DEBUG_HIGH, "[BMI160] set conf err!\r\n");
        return false;
    } 
 
    /* Set foc */
    if (BMI160_OK != BMI160_fastCompensation())
    {
        DEBUG_PRINT0(DEBUG_HIGH, "[BMI160] init err! foc\r\n");
        return false;          
    } 
    
    /* Set the sensor interrupt configuration */
    if(BMI160_OK != BMI160_Set_Int_Config())
    {
        DEBUG_PRINT0(DEBUG_HIGH, "[BMI160] init err! Set config\r\n");
        return false;          
    }    

    if( BMI160_OK != BMI160_Set_FIFO_Config())
    {
        DEBUG_PRINT0(DEBUG_HIGH, "[BMI160] init err! FIFO config\r\n");
        return false;          
    } 
    
    /* MCU exti config */
    BMI160_EXTI_config(); 
    
    return true;
}

/*******************************************************************************
*    Function: BMI160_ReadFIFOxyz
*
*  Parameters:  dst point to a buffer which store the read data
*     Returns:  the byte count read
* Description:  read axix-x/y/z data from fifo buffer
*******************************************************************************/
uint8_t BMI160_ReadFIFOxyz(void **pAccel_data, void **pGyro_data, uint8_t *pFrame_cnt)
{
    uint8_t rslt =0;
   
    rslt = bmi160_get_fifo_data(&bmi160);
    if (BMI160_OK == rslt)
    {
        rslt = bmi160_extract_accel(accel_data, pFrame_cnt, &bmi160);
        
        if (BMI160_OK == rslt)
        {
            rslt =bmi160_extract_gyro(gyro_data, pFrame_cnt, &bmi160); 
            
            *pAccel_data = accel_data;
            *pGyro_data = gyro_data;
         }
    }
    
    return rslt;
}

extern void BMI160_Read(void);
void BMI160_Read(void)
{
        uint8_t data_test[10];
        bmi160_get_regs(0x1c, data_test, 10, &bmi160);
        
        //data_test[9]=8;
        
        bmi160_get_regs(0x50, data_test, 10, &bmi160);
        
        //data_test[9]=8;
         bmi160_get_regs(0x45, data_test, 10, &bmi160);
        
        data_test[9]=8;
}

/*******************************************************************************
*    Function: bmi160_fastCompensation
*
*  Parameters:  none
*     Returns:  true or false
* Description:  Compensate  axix-x/y for 0g and axix-z for -1g
*******************************************************************************/
static int8_t BMI160_fastCompensation(void)
{
	int8_t rslt = 0;
	/* FOC configuration structure */
	struct bmi160_foc_conf foc_conf;
	/* Structure to store the offsets */
	struct bmi160_offsets offsets;
	
	/* Enable FOC for accel with target values of z = -1g ; x,y as 0g */
	foc_conf.acc_off_en = BMI160_ENABLE;
	foc_conf.foc_acc_x  = BMI160_FOC_ACCEL_0G;
	foc_conf.foc_acc_y  = BMI160_FOC_ACCEL_0G;
	foc_conf.foc_acc_z  = BMI160_FOC_ACCEL_NEGATIVE_G;
	
	/* Enable FOC for gyro */
	foc_conf.foc_gyr_en = BMI160_ENABLE;
	foc_conf.gyro_off_en = BMI160_ENABLE;

	rslt = bmi160_start_foc(&foc_conf, &offsets, &bmi160);
	
	if (rslt == BMI160_OK) {
		DEBUG_PRINT0(DEBUG_HIGH, "\n FOC DONE SUCCESSFULLY ");
		DEBUG_PRINT0(DEBUG_HIGH, "\n OFFSET VALUES AFTER FOC : ");
		DEBUG_PRINT1(DEBUG_HIGH, "\n OFFSET VALUES ACCEL X : %d",offsets.off_acc_x);
		DEBUG_PRINT1(DEBUG_HIGH,"\n OFFSET VALUES ACCEL Y : %d",offsets.off_acc_y);
		DEBUG_PRINT1(DEBUG_HIGH,"\n OFFSET VALUES ACCEL Z : %d",offsets.off_acc_z);
		DEBUG_PRINT1(DEBUG_HIGH,"\n OFFSET VALUES GYRO  X : %d",offsets.off_gyro_x);
		DEBUG_PRINT1(DEBUG_HIGH,"\n OFFSET VALUES GYRO  Y : %d",offsets.off_gyro_y);
		DEBUG_PRINT1(DEBUG_HIGH,"\n OFFSET VALUES GYRO  Z : %d \n",offsets.off_gyro_z);	
	}
	
	/* After start of FOC offsets will be updated automatically and 
	 * the data will be very much close to the target values of measurement */

	return rslt;
}


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
    BMI160_CS_LOW();
    while(cnt--)
    {
        while (SPI_I2S_GetFlagStatus(prv_SPI, SPI_I2S_FLAG_TXE) == RESET);
        SPI_I2S_SendData(prv_SPI, *pSend);
        
        while (SPI_I2S_GetFlagStatus(prv_SPI, SPI_I2S_FLAG_RXNE) == RESET);        
        SPI_I2S_ReceiveData(prv_SPI);
        
        pSend++;
    }
    BMI160_CS_HIGH();
    return 0;    
}

/*******************************************************************************
*    Function: BMI160_SetConfig
*
*  Parameters:  none
*     Returns:  none
* Description:  configure the BMI160
*******************************************************************************/
static int8_t BMI160_Set_Int_Config(void)
{
    int8_t rslt;
    static struct bmi160_int_settg int_config;
    
    /* Select the Interrupt channel/pin */
    int_config.int_channel = BMI160_INT_CHANNEL_1;// Interrupt channel/pin 1
    
    /* Select the Interrupt type */
    int_config.int_type = BMI160_ACC_HIGH_G_INT;// Choosing high-g interrupt
    
    /* Select the interrupt channel/pin settings */
    int_config.int_pin_settg.output_en = BMI160_ENABLE;// Enabling interrupt pins to act as output pin
    int_config.int_pin_settg.output_mode = BMI160_DISABLE;// Choosing push-pull mode for interrupt pin
    int_config.int_pin_settg.output_type = BMI160_ENABLE;// Choosing active low output
    int_config.int_pin_settg.edge_ctrl = BMI160_ENABLE;// 1: Choosing edge triggered output
    int_config.int_pin_settg.input_en = BMI160_DISABLE;// Disabling interrupt pin to act as input
    int_config.int_pin_settg.latch_dur = BMI160_LATCHED;// 80ms-latched output
    
    /* Select interrupt parameters */
    int_config.int_type_cfg.acc_high_g_int.high_g_x = BMI160_ENABLE;// Enabling x-axis 
    int_config.int_type_cfg.acc_high_g_int.high_g_y = BMI160_ENABLE;// Enabling y-axis 
    int_config.int_type_cfg.acc_high_g_int.high_g_z = BMI160_DISABLE;// Enabling z-axis 
    int_config.int_type_cfg.acc_high_g_int.high_hy = 0x01;// 1*125mg for 2g range
    int_config.int_type_cfg.acc_high_g_int.high_data_src = 0;// 0: filter
    int_config.int_type_cfg.acc_high_g_int.high_thres = 50;// (2-g range) -> (high_thres) * 7.81 mg, 
    int_config.int_type_cfg.acc_high_g_int.high_dur = 0x10;// default: high_dur * 2.5ms
    
    /* Set the interrupt */
    rslt=bmi160_set_int_config(&int_config, &bmi160); /* sensor is an instance of the structure bmi160_dev  */
    
    

   
    /* Select the Interrupt type */
    int_config.int_type = BMI160_ACC_GYRO_FIFO_WATERMARK_INT;// Choosing  interrupt
     
    /* Select interrupt parameters */
    int_config.fifo_WTM_int_en = BMI160_DISABLE;// BMI160_DISABLE
    //int_config.int_pin_settg.output_type = BMI160_DISABLE;
    /* Set the interrupt */
    rslt=bmi160_set_int_config(&int_config, &bmi160); /* sensor is an instance of the structure bmi160_dev  */
 
    /* Select the Interrupt type */
    int_config.int_type = BMI160_ACC_GYRO_FIFO_FULL_INT;// Choosing  interrupt
    
    /* Select interrupt parameters */
    int_config.fifo_full_int_en = BMI160_DISABLE;// BMI160_DISABLE 
     /* Set the interrupt */
    rslt=bmi160_set_int_config(&int_config, &bmi160); /* sensor is an instance of the structure bmi160_dev  */
   
//
//    //map fifo_full_interrutp to intr1
//    if(BMI160_OK != bmi160_set_intr1_fifo_full(INTR_ENABLE))
//    {
//        return false;
//    }
//
//#if 0
//    //map orient intrreutp to intr1
//    if(BMI160_OK != bmi160_set_intr_orient(BMI160_INTR1_ORIENT, INTR_ENABLE))
//    {
//        return false;
//    }
//    //set orient mode to symmetrical
//    if(BMI160_OK != bmi160_set_orient_mode(0))
//    {
//        return false;
//    }
//#endif
//    
//    //set fifo mode as stream
//    if(BMI160_OK != bmi160_set_fifo_mode(0x02))
//    {
//        return false;
//    }
//    //set fifo date select X/Y/Z
//    if(BMI160_OK != bmi160_set_fifo_data_select(0x00))
//    {
//        return false;
//    }
//    
//    //disable fifo full interrupt
//    if(BMI160_OK != bmi160_set_intr_fifo_full(INTR_DISABLE))
//    {
//        return false;
//    }
    return rslt;
}

#define SPI_BUFFER_LEN 5
#define BMI160_BUS_READ_WRITE_ARRAY_INDEX	1
#define BMI160_SPI_BUS_WRITE_CONTROL_BYTE	0x7F
#define BMI160_SPI_BUS_READ_CONTROL_BYTE	0x80
#define BMI160_INIT_VALUE (0)

/*******************************************************************************
*    Function: BMI160_SPI_bus_read
*
*  Parameters:  dev_addr express the devcie address
*               reg_addr express the register address
*               reg_data point to a buffer which stored the data read out 
*               cnt express the data's number to be read
*     Returns:  0 sucess, otherwise fail
* Description:  read data from the specified register
*******************************************************************************/
static int8_t BMI160_SPI_bus_read(uint8_t dev_addr, \
                                  uint8_t reg_addr, \
                                  uint8_t *reg_data, \
                                  uint16_t cnt)
{
    BMI160_CS_LOW();

    while (SPI_I2S_GetFlagStatus(prv_SPI, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(prv_SPI, reg_addr++ | 0x80);
    while (SPI_I2S_GetFlagStatus(prv_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    SPI_I2S_ReceiveData(prv_SPI);
    while(cnt--)
    {
        while (SPI_I2S_GetFlagStatus(prv_SPI, SPI_I2S_FLAG_TXE) == RESET);
        SPI_I2S_SendData(prv_SPI, reg_addr++ | 0x80);
        while (SPI_I2S_GetFlagStatus(prv_SPI, SPI_I2S_FLAG_RXNE) == RESET);

        *reg_data = SPI_I2S_ReceiveData(prv_SPI);
        reg_data++;
    }
       
    BMI160_CS_HIGH();   
    return 0;
}

/*******************************************************************************
*    Function: BMI160_SPI_bus_write
*
*  Parameters:  dev_addr express the devcie address
*               reg_addr express the register address
*               reg_data point to a buffer which stored the data read out 
*               cnt express the data's number to be read
*     Returns:  0 sucess, otherwise fail
* Description:  write data to the specified register
*******************************************************************************/
static int8_t BMI160_SPI_bus_write(uint8_t dev_addr, \
                                   uint8_t reg_addr, \
                                   uint8_t *reg_data, \
                                   uint16_t cnt)
{
	int32_t iError = BMI160_INIT_VALUE;
	uint8_t array[SPI_BUFFER_LEN * 2];
	uint8_t stringpos = BMI160_INIT_VALUE;
    
	for (stringpos = BMI160_INIT_VALUE; stringpos < cnt; stringpos++) {
		array[stringpos * 2] = (reg_addr++) & \
                                BMI160_SPI_BUS_WRITE_CONTROL_BYTE;
		array[stringpos * 2 + BMI160_BUS_READ_WRITE_ARRAY_INDEX] = \
                                                *(reg_data + stringpos);
	}
    iError = SPI_WRITE_STRING(array, cnt*2);
	return (int8_t)iError;
}
/*******************************************************************************
*    Function: BMI160_delay_msek
*
*  Parameters:  msek express msek millisecond 
*     Returns:  none
* Description:  delay mseck ms
*******************************************************************************/
static void BMI160_delay_msek(uint32_t msek)
{
   // uDelay(msek*1000); 
    OS_Sleep(msek);
}

/**
  * @brief  Configures EXTI Lines.
  * @param  None
  * @retval None
  */
static void BMI160_EXTI_config(void)
{
    EXTI_InitTypeDef    EXTI_InitStructure;
    NVIC_InitTypeDef    NVIC_InitStructure; 

#if 1   
    /* Accel & Gyro */ // need to change the sourece, interfere with ACC
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource2);
    EXTI_InitStructure.EXTI_Line = EXTI_Line2;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
}

/*******************************************************************************
*    Function: BMI160_Set_Accel_Gyo_Config
*
*  Parameters:  None 
*     Returns:  none
* Description:  set odr/rang/bandwith/ and power mode
*******************************************************************************/
static int8_t BMI160_Set_Accel_Gyo_Config(void)
{
    int8_t rslt =0;
    
    /* Set the accel and gyro sensor configuration */
    /* Select the Output data rate, range of accelerometer sensor */
    bmi160.accel_cfg.odr = BMI160_ACCEL_ODR_25HZ;
    bmi160.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    bmi160.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    
    /* Select the power mode of accelerometer sensor */
    bmi160.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
    
    /* Select the Output data rate, range of Gyroscope sensor */
    bmi160.gyro_cfg.odr = BMI160_ACCEL_ODR_25HZ;
    bmi160.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    bmi160.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
    
    /* Select the power mode of Gyroscope sensor */
    bmi160.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE; 

    /* Set the sensor configuration */
    rslt= bmi160_set_sens_conf(&bmi160);
    
    return rslt;
}

/*******************************************************************************
*    Function: BMI160_Set_FIFO_Config
*
*  Parameters:  None 
*     Returns:  none
* Description:  set FIFO config
*******************************************************************************/
static int8_t BMI160_Set_FIFO_Config(void)
{
    int8_t rslt = 0;
    
    fifo_frame.data = fifo_buff;
	fifo_frame.length = BMI160_FIFO_DATA_SIZE;
	bmi160.fifo = &fifo_frame;
    
   	/* Configure the sensor's FIFO settings 
    *   Disable header
    *   Enable accel and gyro
    *                                      */
    rslt = bmi160_set_fifo_config(BMI160_FIFO_CONFIG_1_MASK, BMI160_DISABLE, &bmi160); 
     
    if (rslt == BMI160_OK) 
    {
    	rslt = bmi160_set_fifo_config(BMI160_FIFO_GYRO | BMI160_FIFO_ACCEL,
                                      BMI160_ENABLE, &bmi160); 
        if (rslt == BMI160_OK) 
        {
            rslt = bmi160_set_fifo_wm(BMI160_FIFO_WATER_MARK, &bmi160);
        }
    }
   
    return rslt;
}

/*******************************************************************************
*    Function: BMI160_Set_FIFO_Config
*
*  Parameters:  None 
*     Returns:  none
* Description:  set FIFO config
*******************************************************************************/
int8_t BMI160_get_intr_stat(uint8_t *intr_status)
{
    
    return bmi160_get_regs(BMI160_INT_STATUS_0_ADDR+1, intr_status, 1, &bmi160);  
}

/*******************************************************************************
*    Function: BMI160_enable_high_g_int
*
*  Parameters:  None 
*     Returns:  none
* Description:  
*******************************************************************************/
int8_t BMI160_enable_high_g_int(bool enable)
{
	int8_t rslt;
	uint8_t data = 0;
	uint8_t temp = 0;

	/* Enable low-g interrupt in Int Enable 1 register */
	rslt = bmi160_get_regs(BMI160_INT_ENABLE_1_ADDR, &data, 1, &bmi160);

	if (rslt == BMI160_OK)
    {
        if (enable)
        {
            temp = data | (BMI160_HIGH_G_X_INT_EN_MASK | BMI160_HIGH_G_Y_INT_EN_MASK);
        }
		else
        {
            temp = data & ~(BMI160_HIGH_G_X_INT_EN_MASK | BMI160_HIGH_G_Y_INT_EN_MASK |BMI160_HIGH_G_Z_INT_EN_MASK);
        }

		/* write data to Int Enable 0 register */
		rslt = bmi160_set_regs(BMI160_INT_ENABLE_1_ADDR, &temp, 1, &bmi160);
	}

	return rslt;
}

/*******************************************************************************
*    Function: BMI160_enable_fifo_int
*
*  Parameters:  None 
*     Returns:  none
* Description:  
*******************************************************************************/
int8_t BMI160_enable_fifo_int(uint8_t full_waterMark, bool enable)
{
	int8_t rslt;
	uint8_t data = 0;
	uint8_t temp = 0;

	/* Enable low-g interrupt in Int Enable 1 register */
	rslt = bmi160_get_regs(BMI160_INT_ENABLE_1_ADDR, &data, 1, &bmi160);

	if (rslt == BMI160_OK)
    {
        if (enable)
        {
            temp = data | full_waterMark;
        }
		else
        {
            temp = data & ~(full_waterMark);
        }

		/* write data to Int Enable 0 register */
		rslt = bmi160_set_regs(BMI160_INT_ENABLE_1_ADDR, &temp, 1, &bmi160);
	}

	return rslt;
}

/*******************************************************************************
*    Function: BMI160_rst_int
*
*  Parameters:  None 
*     Returns:  none
* Description:  
*******************************************************************************/
void BMI160_rst_int(void)
{
    uint8_t cmd = 0xB1;
    
	/* Start the  process */
	bmi160_set_regs(BMI160_COMMAND_REG_ADDR, &cmd, 1, &bmi160);
    //bmi160.delay_ms(10);
    bmi160_set_fifo_flush(&bmi160);
}

/*******************************************************************************
*    Function: BMI160_data_filter
*
*  Parameters:  None 
*     Returns:  none
* Description:  
*******************************************************************************/
extern void BMI160_data_filter(int16_t *pAccel, int16_t *pGyro, uint16_t count_num );
void BMI160_data_filter(int16_t *pAccel, int16_t *pGyro, uint16_t count_num)
{
    uint16_t idx;
    int32_t tempAcc, tempGyro;
//    
//    tempAcc = *pAccel + *(pAccel+1) + *(pAccel+2) + *(pAccel+3);
//    tempGyro = *pGyro + *(pGyro+1) + *(pGyro+2) + *(pGyro+3);  
     
    for (idx=0;idx < (count_num-3)*3; idx++)
    {
        tempAcc = pAccel[idx] + pAccel[idx+3] + pAccel[idx+6] + pAccel[idx+9];
       
        tempGyro = pGyro[idx] + pGyro[idx+3] + pGyro[idx+6] + pGyro[idx+9];
        
        pAccel[idx] = (int16_t)(tempAcc>>2);
        pGyro[idx] = (int16_t)(tempGyro>>2);
    }
  
}