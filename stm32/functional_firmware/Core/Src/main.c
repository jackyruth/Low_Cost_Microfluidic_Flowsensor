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
#define FLOW_THRESHOLD 0.4

#define sample_duration 13000
#define N_MEASUREMENTS 13
#define N_PARAMS 5

#define A_CONST 2366.09
#define B_CONST 4150.07
#define C_CONST 0.0320124

#define HEAT_ON 100
#define SIZE 3
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
static const uint8_t TMP117_ADDR = 0x4A << 1;
static const uint8_t BROADCAST_ADDR = 0x00 << 1;
static const uint8_t REG_TEMP = 0x00;
static const uint8_t REG_CONF = 0x01;
static uint8_t buf[SIZE];
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
double analytical_model(double *p, int t, void *fdata)
{
	// p[0] = q
	// p[1] = v
	// p[2] = d
	// p[3] = k
	// p[4] = a
    return p[0]/(4.0*p[3]*3.1415*(t+p[2])) * exp(-pow(3-p[1]*(t+p[2]),2)/(4.0*p[4]*(t+p[2])));
}

/*
 * @brief   Gradient function for Newton law of heating
 */
void gradient(double *g, double *p, int t, void *fdata)
{
	double tmp_rslt = exp(-pow(3-p[1]*(t+p[2]),2)/(4.0*p[4]*(t+p[2])));
    g[0] = tmp_rslt/(4.0*3.1415*p[3]*(t+p[2]));
    g[1] = tmp_rslt*p[0]*(3-p[1]*(t+p[2]))/(8.0*3.1415*p[3]*p[4]*(t+p[2]));
    g[2] = p[0]*tmp_rslt*(-4.0*p[4]*(t+p[2])-(pow(p[2]*p[1],2)+2*t*p[2]*pow(p[1],2)-9+pow(t*p[1],2)))/(16*3.1415*p[3]*p[4]*pow(t+p[2],3));
    g[3] = -p[0]*tmp_rslt/(4.0*3.1415*pow(p[3],2)*(t+p[2]));
    g[4] = tmp_rslt*p[0]*pow(3-p[1]*(t+p[2]),2)/(16.0*3.1415*p[3]*pow(p[4],2)*pow(t+p[2],2));
}

/*
 * @brief  Function for prediction of time, when target temperature will be reached
 *
 * @par    Parameters from Newton equation
 * @temp   Target temperature
 * @return Number of sample
 */
double get_timedelta(double *p)
{
    return 1000.0*((-2*p[4]+sqrt(4*pow(p[4],2)+pow(3*p[1],2)))/pow(p[1],2)-p[2]);
}
/*
 * @brief  Function for transforming timedelta to flowrate, provide 2 decimal places
 *
 * @par    Parameters from Newton equation
 * @temp   Target temperature
 * @return Number of sample
 */
double get_flowrate(double td)
{
    return log(pow(B_CONST/(td-A_CONST),1/C_CONST));
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
  HAL_StatusTypeDef ret;
  uint32_t prev_time;
  int16_t val;
  float temp_c;
  float lowest_c;
  float highest_c;
  float timedelta;
  float flowrate;
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
  buf[0] = REG_CONF;
  buf[1] = 0x00;
  buf[2] = 0x00;
  ret = HAL_I2C_Master_Transmit(&hi2c1, TMP117_ADDR, buf, 3, HAL_MAX_DELAY);
  if (ret != HAL_OK) {
    log_debug("%s","I2C Failed");
  }
  log_debug("%s",(char*)buf);
  levmarq_init(&lmstat);
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
  while (1)
  {
	  double t_data[N_MEASUREMENTS];
	  double params[N_PARAMS] = {30, 0.5, 1, 0.598, 0.143}; // Initial values of parameters

	  // Heater Pulse
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
	    HAL_Delay(HEAT_ON);
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

	  // Collect Data
	    for(int i = 0; i < N_MEASUREMENTS; i++){
			prev_time = HAL_GetTick();
			buf[0] = REG_TEMP;
			ret = HAL_I2C_Master_Transmit(&hi2c1, TMP117_ADDR, buf, 1, HAL_MAX_DELAY);
			if (ret != HAL_OK) {
				log_debug("%s","I2C Failed");
			}
			else{
				ret = HAL_I2C_Master_Receive(&hi2c1, TMP117_ADDR, buf, 2, HAL_MAX_DELAY);
				if (ret != HAL_OK) {
					log_debug("%s","I2C Failed");
				}
				else{
					val = (int16_t)(buf[0] << 8| buf[1]);
					temp_c = val * 0.0078125;
					for(int i=0; i<SIZE; i++){
						buf[i] = 0;
					}
					log_debug("%lu %.2f",(uint32_t)prev_time,temp_c);
					t_data[i] = temp_c;
				}
			}
			while(HAL_GetTick()-prev_time < (sample_duration/N_MEASUREMENTS));
	    }
  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */
	  lowest_c = 100.0;
	  highest_c = 0.0;
	  for(int i = 0; i < N_MEASUREMENTS; i++){
		  if(t_data[i] < lowest_c){
			  lowest_c = t_data[i];
		  }
		  if(t_data[i] > highest_c){
			  highest_c = t_data[i];
		  }
	  }
	  if ((highest_c - lowest_c) > FLOW_THRESHOLD){
		  for(int i = 0; i < N_MEASUREMENTS; i++){
			  t_data[i] -= lowest_c;
		  }

		  prev_time = HAL_GetTick();
		  n_iterations = levmarq(N_PARAMS, params, N_MEASUREMENTS, t_data, NULL,
				  &analytical_model, &gradient, NULL, &lmstat);
		  prev_time = HAL_GetTick()-prev_time;
		  log_debug("%s","\n\n");
		  log_debug("%s","**************** End of calculation ***********************");
		  log_debug("N iterations: %d\t Elapsed Time: %lu ms", n_iterations,prev_time);
		  log_debug("q: %f, v: %f, d: %f, k: %f, a: %f", params[0], params[1], params[2], params[3], params[4]);
		  log_debug("%s","**************** Interpolation test ***********************");

		  timedelta = get_timedelta(params);
		  flowrate  = get_flowrate(get_timedelta(params));
		  log_debug("Result: %.2f uL/min w timedelta %.2f ms",flowrate,timedelta);

		  if((flowrate > 0 ) && (params[0] > 0) && (params[1] > 0) && \
				  (params[2] > 0) && (params[3] > 0) && (params[4] > 0)){
			  log_info("%.2f", flowrate);
		  }
		  else{
			  log_info("%s", "OOR");
		  }
	  }
	  else{
		  log_info("%.2f", 0.);
	  }
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

