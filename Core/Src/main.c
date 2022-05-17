/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "i2s.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/*| 3.3V | 3.3V on Nucleo board  |
| FLT  | Filter select, grounded: select normal latency, grounded  |
| DMP  | De-emphasis control, grounded(OFF)  |
| SCL  | System clock input, in datasheet SCK pin. System clock is internally generated, it should not be wired anywhere  |
| BCK  | Audio data bit clock input. In I2S terminology is equivalent to serial clock(SCK) I've connected it to `PB10 > I2S2_CK` |
| DIN  | Audio data input. This is Serial Data pin, I've connected it to `PC3 > I2S2_SD` |
| LCK  | Is LRCK in PCM5102 datasheet, which is Audio data word clock input. It is used to switch data for left and right channel. In I2S is I2SX_WS pin, means "Word Select" pin. Connect it to `PB12 > I2S2_WS` |
| FMT  | Audio Format Selection, we select I2S, so this should be grounded  |
| XMT  | In dataseet is XSMT, which is mute control, should be HIGH to unmute.  |
*/
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const uint16_t triangle_wave[]  = {
0x400,0x800,0xc00,0x1000,0x1400,0x1800,0x1c00,0x2000,
0x2400,0x2800,0x2c00,0x3000,0x3400,0x3800,0x3c00,0x4000,
0x4400,0x4800,0x4c00,0x5000,0x5400,0x5800,0x5c00,0x6000,
0x6400,0x6800,0x6c00,0x7000,0x7400,0x7800,0x7c00,0x8000,
0x83ff,0x87ff,0x8bff,0x8fff,0x93ff,0x97ff,0x9bff,0x9fff,
0xa3ff,0xa7ff,0xabff,0xafff,0xb3ff,0xb7ff,0xbbff,0xbfff,
0xc3ff,0xc7ff,0xcbff,0xcfff,0xd3ff,0xd7ff,0xdbff,0xdfff,
0xe3ff,0xe7ff,0xebff,0xefff,0xf3ff,0xf7ff,0xfbff,0xffff,
0xfbff,0xf7ff,0xf3ff,0xefff,0xebff,0xe7ff,0xe3ff,0xdfff,
0xdbff,0xd7ff,0xd3ff,0xcfff,0xcbff,0xc7ff,0xc3ff,0xbfff,
0xbbff,0xb7ff,0xb3ff,0xafff,0xabff,0xa7ff,0xa3ff,0x9fff,
0x9bff,0x97ff,0x93ff,0x8fff,0x8bff,0x87ff,0x83ff,0x8000,
0x7c00,0x7800,0x7400,0x7000,0x6c00,0x6800,0x6400,0x6000,
0x5c00,0x5800,0x5400,0x5000,0x4c00,0x4800,0x4400,0x4000,
0x3c00,0x3800,0x3400,0x3000,0x2c00,0x2800,0x2400,0x2000,
0x1c00,0x1800,0x1400,0x1000,0xc00,0x800,0x400,0x0,
};
__IO int16_t                 UpdatePointer = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2S2_Init();
  MX_CRC_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //HAL_I2S_Transmit(&hi2s2, triangle_wave, sizeof(triangle_wave)/sizeof(triangle_wave[0]), 1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
