#include <stdio.h>

#include "dma.h"
#include "i2c.h"
#include "lptim.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "RM3100.h"
#include "stm32l0xx_hal.h"


extern SPI_HandleTypeDef                hspi1;

#define RM3100_SPI_CS_SET(x)           (HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, (x)));

#define CYCLE_COUNTS_X                  400
#define CYCLE_COUNTS_Y                  400
#define CYCLE_COUNTS_Z                  400


/* RM3100 params */
typedef struct rm3100_params {
    void *pHandle;
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



static int ST_SPI_Write(void *self, uint8_t reg, uint8_t *pData, uint16_t Size){
    uint8_t ret = HAL_ERROR;
    RM3100_Iface *iface   = self;
    RM3100_params *params = iface->params;

    RM3100_SPI_CS_SET(0);
 
    ret = HAL_SPI_Transmit(params->pHandle, &reg, sizeof(uint8_t), params->timeout);
    ret |= HAL_SPI_Transmit(params->pHandle, pData, Size, params->timeout);

    RM3100_SPI_CS_SET(1);
    
    return ret;
}


static int ST_SPI_Read(void *self, uint8_t reg, uint8_t *pData, uint16_t Size)
{
    uint8_t ret = HAL_ERROR;
    RM3100_Iface *iface   = self;
    RM3100_params *params = iface->params;

    // read bit
    reg = reg | 0x80;
    
    RM3100_SPI_CS_SET(0);

    ret = HAL_SPI_Transmit(params->pHandle, &reg, sizeof(uint8_t), params->timeout);

    ret |= HAL_SPI_Receive(params->pHandle, pData, Size, params->timeout);

    RM3100_SPI_CS_SET(1);

    return ret;
}


int rm3100_test_single_sample(void *self)
{
    int ret             = RM3100_RET_EIO;
    uint8_t revid       = 0x00;
    RM3100_Iface *iface = self;
    uint8_t u8val       = 0;
    
    struct Data    data = { 0 };


    printf("### Hello RM3100 spi smm sample ###\r\n");

    
    revid = RM3100_GetHardwareRevision(iface);
    printf("read ID 0x22 == %x\n", revid);
    if (revid != RM3100_REVID) 
    {
        return RM3100_RET_ENODEV;
    }

    // set scale
    RM3100_SetCycleCounts(iface, CYCLE_COUNTS_X, CYCLE_COUNTS_Y, CYCLE_COUNTS_Z);
    HAL_Delay(10);


    while (1) 
    {
        RM3100_SetSingleMeasurementMode(iface, SMM_AXIS_X|SMM_AXIS_Y|SMM_AXIS_Z);
        
        while(1 != RM3100_GetStatus(iface))
        {
            HAL_Delay(10);
        }
        
        RM3100_GetData(iface, &data);
        printf("x: %f, y: %f, z: %f\r\n", data.x, data.y, data.z);

        HAL_Delay(1000);
    }
 
}


int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
    SystemClock_Config();
    
    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_SPI1_Init();
    MX_LPTIM1_Init();
    MX_USART2_UART_Init();
    
    RM3100_params params = {
        .pHandle = &hspi1,
        .timeout = 1000,
    };
    
    RM3100_Iface  iface = {
        .RM3100_read    = &ST_SPI_Read,
        .RM3100_write   = &ST_SPI_Write,
        .params = &params,  
    };

    RM3100_Init(&iface);
    
    rm3100_test_single_sample(&iface);
}


PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */

    HAL_UART_Transmit_DMA(&huart2, (uint8_t *) &ch, 1);

    return ch;
}

