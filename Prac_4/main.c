/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f0xx.h"
#include <lcd_stm32f0.c>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TODO: Add values for below variables
#define NS 129          // Number of samples in LUT
#define TIM2CLK 8000000 // STM Clock frequency
#define F_SIGNAL 1      // Frequency of output analog signal
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */
// TODO: Add code for global variables, including LUTs

uint32_t Sin_LUT[NS] = {512.000, 537.123, 562.185, 587.126, 611.886, 636.406, 660.626, 684.488, 707.934, 730.908, 753.355, 775.221, 796.452, 816.998, 836.809, 855.838, 874.039, 891.367, 907.781, 923.242, 937.712, 951.157, 963.544, 974.843, 985.026, 994.071, 1001.953, 1008.656, 1014.162, 1018.458, 1021.535, 1023.383, 1024.000, 1023.383, 1021.535, 1018.458, 1014.162, 1008.656, 1001.953, 994.071, 985.026, 974.843, 963.544, 951.157, 937.712, 923.242, 907.781, 891.367, 874.039, 855.838, 836.809, 816.998, 796.452, 775.221, 753.355, 730.908, 707.934, 684.488, 660.626, 636.406, 611.886, 587.126, 562.185, 537.123, 512.000, 486.877, 461.815, 436.874, 412.114, 387.594, 363.374, 339.512, 316.066, 293.092, 270.645, 248.779, 227.548, 207.002, 187.191, 168.162, 149.961, 132.633, 116.219, 100.758, 86.288, 72.843, 60.456, 49.157, 38.974, 29.929, 22.047, 15.344, 9.838, 5.542, 2.465, 0.617, 0.000, 0.617, 2.465, 5.542, 9.838, 15.344, 22.047, 29.929, 38.974, 49.157, 60.456, 72.843, 86.288, 100.758, 116.219, 132.633, 149.961, 168.162, 187.191, 207.002, 227.548, 248.779, 270.645, 293.092, 316.066, 339.512, 363.374, 387.594, 412.114, 436.874, 461.815, 486.877, 512.000};

uint32_t saw_LUT[NS] = {0.000, 7.992, 15.984, 23.977, 31.969, 39.961, 47.953, 55.945, 63.938, 71.930, 79.922, 87.914, 95.906, 103.898, 111.891, 119.883, 127.875, 135.867, 143.859, 151.852, 159.844, 167.836, 175.828, 183.820, 191.812, 199.805, 207.797, 215.789, 223.781, 231.773, 239.766, 247.758, 255.750, 263.742, 271.734, 279.727, 287.719, 295.711, 303.703, 311.695, 319.688, 327.680, 335.672, 343.664, 351.656, 359.648, 367.641, 375.633, 383.625, 391.617, 399.609, 407.602, 415.594, 423.586, 431.578, 439.570, 447.562, 455.555, 463.547, 471.539, 479.531, 487.523, 495.516, 503.508, 511.500, 519.492, 527.484, 535.477, 543.469, 551.461, 559.453, 567.445, 575.438, 583.430, 591.422, 599.414, 607.406, 615.398, 623.391, 631.383, 639.375, 647.367, 655.359, 663.352, 671.344, 679.336, 687.328, 695.320, 703.312, 711.305, 719.297, 727.289, 735.281, 743.273, 751.266, 759.258, 767.250, 775.242, 783.234, 791.227, 799.219, 807.211, 815.203, 823.195, 831.188, 839.180, 847.172, 855.164, 863.156, 871.148, 879.141, 887.133, 895.125, 903.117, 911.109, 919.102, 927.094, 935.086, 943.078, 951.070, 959.062, 967.055, 975.047, 983.039, 991.031, 999.023, 1007.016, 1015.008, 1023.000};

uint32_t triangle_LUT[NS] = {0.000, 8.000, 16.000, 24.000, 32.000, 40.000, 48.000, 56.000, 64.000, 72.000, 80.000, 88.000, 96.000, 104.000, 112.000, 120.000, 128.000, 136.000, 144.000, 152.000, 160.000, 168.000, 176.000, 184.000, 192.000, 200.000, 208.000, 216.000, 224.000, 232.000, 240.000, 248.000, 256.000, 264.000, 272.000, 280.000, 288.000, 296.000, 304.000, 312.000, 320.000, 328.000, 336.000, 344.000, 352.000, 360.000, 368.000, 376.000, 384.000, 392.000, 400.000, 408.000, 416.000, 424.000, 432.000, 440.000, 448.000, 456.000, 464.000, 472.000, 480.000, 488.000, 496.000, 504.000, 512.000, 504.000, 496.000, 488.000, 480.000, 472.000, 464.000, 456.000, 448.000, 440.000, 432.000, 424.000, 416.000, 408.000, 400.000, 392.000, 384.000, 376.000, 368.000, 360.000, 352.000, 344.000, 336.000, 328.000, 320.000, 312.000, 304.000, 296.000, 288.000, 280.000, 272.000, 264.000, 256.000, 248.000, 240.000, 232.000, 224.000, 216.000, 208.000, 200.000, 192.000, 184.000, 176.000, 168.000, 160.000, 152.000, 144.000, 136.000, 128.000, 120.000, 112.000, 104.000, 96.000, 88.000, 80.000, 72.000, 64.000, 56.000, 48.000, 40.000, 32.000, 24.000, 16.000, 8.000, 0.000};

// TODO: Equation to calculate TIM2_Ticks
uint32_t TIM2_Ticks = TIM2CLK / (NS * F_SIGNAL);  // How often to write new LUT value
uint32_t DestAddress = (uint32_t) & (TIM3->CCR3); // Write LUT TO TIM3->CCR3 to modify PWM duty cycle

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
void EXTI0_1_IRQHandler(void);
void writeLCD(char *char_in);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t prev_millis = 0;
uint32_t curr_millis = 0;
uint32_t delay_t = 1000; // Initialise delay to 1000ms
volatile int wave_type = 0;

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
  init_LCD();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  // TODO: Start TIM3 in PWM mode on channel 3
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  // TODO: Start TIM2 in Output Compare (OC) mode on channel 1.
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);

  // TODO: Start DMA in IT mode on TIM2->CH1; Source is LUT and Dest is TIM3->CCR3; start with Sine LUT
  HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)&Sin_LUT, DestAddress, NS);

  // TODO: Write current waveform to LCD ("Sine")
  delay(3000);
  lcd_command(CLEAR);
  lcd_putstring("Sine");

  // TODO: Enable DMA (start transfer from LUT to CCR)
  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    HAL_Delay(delay_t);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void writeLCD(char *char_in)
{
  // delay(3000);
  // lcd_command(CLEAR);
  lcd_putstring(char_in);
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

  /* Wait till HSI is ready */
  while (LL_RCC_HSI_IsReady() != 1)
  {
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

  /* Wait till System clock is ready */
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
  }
  LL_SetSystemCoreClock(8000000);

  /* Update the time base */
  if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIM2_Ticks - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1023;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

  /**/
  LL_GPIO_SetPinPull(Button0_GPIO_Port, Button0_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(Button0_GPIO_Port, Button0_Pin, LL_GPIO_MODE_INPUT);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EXTI0_1_IRQHandler(void)
{
  // TODO: Debounce using HAL_GetTick()
  curr_millis = HAL_GetTick();

  if (curr_millis - prev_millis >= 1000)
  {
    prev_millis = HAL_GetTick();
  }

  // TODO: Disable DMA transfer and abort IT, then start DMA in IT mode with new LUT and re-enable transfer
  // HINT: Consider using C's "switch" function to handle LUT changes
  __HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_CC1);
  HAL_DMA_Abort_IT(&hdma_tim2_ch1);

  wave_type++;
  delay(3000);
  lcd_command(CLEAR);
  if (wave_type % 3 == 0)
  {
    lcd_putstring("Sine");
    HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)&Sin_LUT, DestAddress, NS);
  }
  else if (wave_type % 3 == 1)
  {
    lcd_putstring("Sawtooth");
    HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)&saw_LUT, DestAddress, NS);
  }
  else
  {
    lcd_putstring("Triangular");
    HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)&triangle_LUT, DestAddress, NS);
    wave_type = -1;
  }
  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);

  HAL_GPIO_EXTI_IRQHandler(Button0_Pin); // Clear interrupt flags
}
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

#ifdef USE_FULL_ASSERT
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
