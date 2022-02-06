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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <time.h>
#include "logger.h"
#include "levmarq.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define N_MEASUREMENTS 395
#define N_PARAMS 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
char print_data[PRINT_DATA_SIZE];
double t_data[N_MEASUREMENTS] = { 21.6,  22. ,  22.4,  22.4,  22.5,  22.4,  22.2,  22.3,  22.4,\
        22.1,  21.8,  21.6,  22.1,  22. ,  21.7,  21.7,  21.5,  21.5,\
        21.4,  21.6,  21.7,  21.8,  21.9,  22. ,  21.9,  22.2,  22.3,\
        22.3,  22.4,  22.4,  22.4,  23. ,  23.6,  24.6,  24.3,  24.3,\
        24.2,  25. ,  25. ,  25.2,  25.4,  26. ,  26.3,  26.3,  26. ,\
        26.1,  25.9,  25.7,  26.1,  26. ,  26.3,  26.4,  26.4,  26.4,\
        26.5,  26.8,  26.8,  26.9,  27.2,  27.8,  27.9,  28.2,  28.1,\
        28.1,  28. ,  28. ,  28.3,  28.4,  28.8,  29.7,  30. ,  30.1,\
        29.8,  30.4,  30.9,  30.6,  30.7,  30.6,  30.3,  30.4,  30.8,\
        31.3,  31.3,  31.4,  31.2,  31.4,  31.5,  31.6,  31.8,  32.3,\
        32.2,  32.7,  32.4,  32.3,  32.7,  33. ,  33. ,  33.1,  33.2,\
        33.3,  33.6,  34. ,  33.9,  34.1,  34.2,  34.4,  34.7,  35.1,\
        35.2,  34.9,  35.2,  35.3,  35.1,  35.3,  36. ,  35.9,  35.9,\
        36.4,  36.9,  36.5,  36.4,  36.9,  36.9,  36.9,  37.4,  37. ,\
        36.9,  36.4,  37. ,  37.1,  37. ,  37.1,  37.1,  37.6,  37.8,\
        38. ,  38.7,  39. ,  39.2,  39.5,  40.1,  40.4,  40.6,  40.2,\
        40.1,  40.3,  40.4,  40.7,  40.9,  40.5,  40.7,  41.5,  41.4,\
        41.3,  40.7,  40.5,  40.8,  41.2,  41.2,  41. ,  41.3,  41.5,\
        41.7,  42.1,  42.1,  42.3,  42.2,  41.8,  42. ,  42.4,  42.6,\
        42.6,  42.3,  42.4,  42.7,  43.1,  42.9,  42.8,  42.8,  43.1,\
        43.4,  44. ,  43.9,  43.9,  43.8,  43.9,  44.4,  44.6,  45.1,\
        45.2,  45.2,  45.2,  45.4,  45.7,  45.3,  45.1,  45.1,  45.6,\
        45.9,  45.9,  45.9,  46. ,  46.2,  46.2,  46.3,  46.6,  46.9,\
        46.9,  47.2,  47.6,  47.5,  47.7,  47.6,  47.7,  48. ,  47.8,\
        47.9,  48. ,  48.2,  48. ,  48. ,  48.1,  48.6,  48.6,  49.2,\
        48.9,  48.8,  49.2,  49.4,  49.2,  49. ,  49.3,  49.8,  50.3,\
        50.7,  50.9,  50.8,  50.6,  50.4,  50.7,  50.9,  51. ,  51.4,\
        51.4,  51.8,  51.8,  51.6,  52.3,  52.7,  53.3,  53.1,  54. ,\
        53.3,  53.3,  53.6,  53.2,  53. ,  53.1,  53.4,  53.5,  53.6,\
        53.9,  53.7,  53.9,  53.9,  52.8,  53.1,  53.1,  53.3,  53.4,\
        53.6,  54. ,  54.3,  54.2,  54.4,  54.7,  54.6,  56.5,  56.4,\
        55.7,  55.8,  55.9,  56.2,  56.2,  56.3,  56.3,  56.5,  56.9,\
        57. ,  57.4,  57.9,  58.2,  57.6,  57.5,  57.9,  58.4,  58.9,\
        59.1,  58.4,  59. ,  62.4,  57.6,  56.6,  58.6,  59. ,  59. ,\
        59. ,  59.3,  59.5,  59. ,  59.4,  59.2,  59.1,  60. ,  59.8,\
        60.3,  60.8,  60.5,  60.4,  61.5,  60.7,  60.7,  61. ,  60.9,\
        61. ,  61.3,  61.7,  61.5,  61.4,  61.8,  61.9,  62.6,  62.3,\
        62.1,  62. ,  62.3,  62.4,  62.4,  61.7,  62.7,  63.2,  62.8,\
        62.9,  63. ,  63.6,  63.9,  63.3,  63.7,  63.5,  63.3,  63.9,\
        64. ,  63.8,  63.9,  64.5,  64.3,  63.8,  63.5,  64. ,  64.3,\
        65.2,  65.7,  65.9,  65.6,  64.7,  65. ,  65.1,  65.8,  66.2,\
        66.6,  66.3,  66. ,  65.5,  66.2,  66.4,  66.8,  67.4,  67.3,\
        67. ,  66.8,  66.4,  67.3,  66.9,  67.6,  72.1,  66.6,  66.2,\
        67.1,  70. ,  68.8,  68.1,  67.8,  67.3,  67.4,  66.2 };

double params[N_PARAMS] = {200, 20, 0.001}; // Initial values of parameters
LMstat lmstat;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
/* @brief   Function, describes Newton law of heating/cooling
 *
 * @usage   par[0] - temperature of heater,
 *          par[1] - initial temperature of water,
 *          par[2] - heat transmission coefficient
 *
 * @par     input of Newton Law:
 * @x       samplenumber
 * @fdata   additional data, not used
 *
 * @return  temperature at the time x
 */
double newton_func(double *par, int x, void *fdata)
{
    return par[0] + (par[1] - par[0]) * exp( -par[2]*x);
}

/*
 * @brief   Gradient function for Newton law of heating
 */
void gradient(double *g, double *par, int x, void *fdata)
{
    g[0] = 1.0 - exp(-par[2] * x);
    g[1] = exp(-par[2] * x);
    g[2] = -x * (par[1] - par[0]) * exp(-par[2] * x);
}

/*
 * @brief  Function for prediction of time, when target temperature will be reached
 *
 * @par    Parameters from Newton equation
 * @temp   Target temperature
 * @return Number of sample
 */
int temp_to_time(double *par, double temp)
{
    return -(1/par[2]) * log((temp - par[0])/(par[1] - par[0]));
}
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
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  int n_iterations;
  levmarq_init(&lmstat);
  n_iterations = levmarq(N_PARAMS, params, N_MEASUREMENTS, t_data, NULL,
          &newton_func, &gradient, NULL, &lmstat);
  log_info("%s","**************** End of calculation ***********************");
  log_info("N iterations: %d", n_iterations);
  log_info("T_heater: %f, T_0: %f, k: %f", params[0], params[1], params[2]);
  log_info("%s","**************** Interpolation test ***********************");
  log_info("%s","Search for temp 70 degrees");
  log_info("Result: %d sample", temp_to_time(params, 50.0));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  log_info("%s","Hello World!");
	  HAL_Delay(1000);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* LPUART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(LPUART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(LPUART1_IRQn);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2010091A;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 112500;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

