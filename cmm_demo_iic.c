#include <stdio.h>

#include "dma.h"
#include "i2c.h"
#include "lptim.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "RM3100.h"
#include "stm32l0xx_hal.h"

#define RM3100_I2C              hi2c1

#define CYCLE_COUNTS_X          200
#define CYCLE_COUNTS_Y          200
#define CYCLE_COUNTS_Z          200



/* RM3100 params */
typedef struct rm3100_params {
    /* I2C handle */
    I2C_HandleTypeDef *hi2c;

    /* 8-bit  I2C address */
    uint8_t addr;

    /* I2C transfer timeout */
    uint32_t timeout;   
}RM3100_params;


#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler */
    PNI_PRINTF("[ERROR] Error Handler\r\n");

    /* User can add his own implementation to report the HAL error return state */
    while (1)
    {}
    /* USER CODE END Error_Handler */
}


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

    RCC_OscInitTypeDef       RCC_OscInitStruct;
    RCC_ClkInitTypeDef       RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.LSEState            = RCC_LSE_ON;
    RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_6;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                       | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_USART2
                                         | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_RTC
                                         | RCC_PERIPHCLK_LPTIM1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_PCLK1;
    PeriphClkInit.RTCClockSelection    = RCC_RTCCLKSOURCE_LSE;
    PeriphClkInit.LptimClockSelection  = RCC_LPTIM1CLKSOURCE_LSE;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }

    /**Configure LSE Drive Capability
    */
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Configure the Systick interrupt time
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    /**Configure the Systick
    */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


static int ST_I2C_Read(void *self, uint8_t reg, uint8_t *pData, uint16_t Size)
{
    uint8_t ret = HAL_ERROR;
    RM3100_Iface *iface   = self;
    RM3100_params *params = iface->params;
    
    ret = HAL_I2C_Mem_Read(params->hi2c, params->addr, reg, I2C_MEMADD_SIZE_8BIT, pData, Size, params->timeout);
    return ret;
}


static int ST_I2C_Write(void *self, uint8_t reg, uint8_t *pData, uint16_t Size)
{
    uint8_t ret = HAL_ERROR;
    RM3100_Iface *iface   = self;
    RM3100_params *params = iface->params;

    ret = HAL_I2C_Mem_Write(params->hi2c, params->addr, reg, I2C_MEMADD_SIZE_8BIT, pData, Size, params->timeout);
    return ret;
}

static int rm3100_is_ready(void)
{
    return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
}

/* An interface for the RM3100 3-axis magnetometer CMMode */
int RM3100_CMM_Test(void *self)
{
    int ret             = RM3100_RET_EIO;
    uint8_t revid       = 0x00;
    RM3100_Iface *iface = self;
    uint8_t u8val       = 0;
    
    struct Data    data = { 0 };


    printf("### Hello RM3100 i2c cmm sample ###\r\n");

    revid = RM3100_GetHardwareRevision(iface);
    printf("read ID 0x22 == %x\n", revid);
    if (revid != RM3100_REVID) 
    {
        return RM3100_RET_ENODEV;
    }


    RM3100_SetCycleCounts(iface, CYCLE_COUNTS_X, CYCLE_COUNTS_Y, CYCLE_COUNTS_Z);
    HAL_Delay(1);

    RM3100_SetCMM_SampleRate(iface, 0.5);
    HAL_Delay(1);

    RM3100_StartContinuousMeasurementMode(iface, CMM_AXIS_X|CMM_AXIS_Y|CMM_AXIS_Z|CMM_DRDM_ALL_AXIS);
     
    while (1) 
    {
        while(0 == rm3100_is_ready())
        {
            HAL_Delay(10);
        }
        
        RM3100_GetData(iface, &data);
        printf("x: %f, y: %f, z: %f\r\n", data.x, data.y, data.z);
   }
   
   RM3100_StopContinuousMeasurementMode(iface, CMM_AXIS_X|CMM_AXIS_Y|CMM_AXIS_Z|CMM_DRDM_ALL_AXIS);
    
}


int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
    SystemClock_Config();
    
    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    // !!! user need to add io config when using rdy pin. in demo ,we use PB3
    
    MX_DMA_Init();
    MX_I2C1_Init();
    MX_LPTIM1_Init();

    MX_USART2_UART_Init();
    
    RM3100_params params = {
        .hi2c = &hi2c1,
        .timeout = 1000,
        .addr= 0x40,            // rm3100 I2c addr
    };
    
    RM3100_Iface  iface = {
        .RM3100_read    = &ST_I2C_Read,
        .RM3100_write   = &ST_I2C_Write,
        .params = &params,  
    };

    RM3100_Init(&iface);
    
    RM3100_CMM_Test(&iface);
}


PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */

    HAL_UART_Transmit_DMA(&huart2, (uint8_t *) &ch, 1);

    return ch;
}

