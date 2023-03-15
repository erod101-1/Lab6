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
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CHARS_PER_LINE 16
#define BUFF 16
#define Ra 1000
#define Rb 2000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void Delay_us(unsigned short us);
void LCD_pulse (void);
void LCD_byte (unsigned char x);
void WriteData (unsigned char x);
void WriteCommand (unsigned char x);
void LCD_4BIT (void);
void LCDprint(char * string, unsigned char line,unsigned char clear);
float get_capacitance(float period);
char get_suffix_and_convert(float* capacitance,float* frequency,char* suffix_f);
float get_period(void);

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

  float capacitance;
  float frequency;
  float period;
  char suffix_f;
  char suffix;
  suffix = 'n';
  suffix_f = 'k';
  char buffer[BUFF];
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  LCD_4BIT();
  start:
  while(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)){
	  LCDprint("Welcome to",1,1);
	  LCDprint("Cap Meter -||-",2,1);
	  while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)){
	  LCDprint("Welcome to",1,1);
	  LCDprint("Cap Meter -||-",2,1);
	  if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)){
		  goto leave_start;
	  }
	  }
  }
  while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)){
  	  LCDprint("Welcome to",1,1);
  	  LCDprint("Cap Meter -||-",2,1);
  }
  leave_start:
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  period = get_period();
	  frequency = (1/get_period())/1000;
	  capacitance = get_capacitance(period);
	  suffix = get_suffix_and_convert(&capacitance,&frequency,&suffix_f);
	  HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port,LED_BUILTIN_Pin);
	  HAL_Delay(1000);
	  LCDprint("CAP. METER", 1, 1);
	  sprintf(buffer,"C:%d%cF ",(int)capacitance,suffix);
	  LCDprint(buffer,2,1);

	  HAL_Delay(100);

	  while((!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) && !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6))|| (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) && !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6))){
		  sprintf(buffer,"Cap:%d%cF ",(int)capacitance,suffix);
		  LCDprint(buffer,1,1);
		  sprintf(buffer,"Freq:%d%cHz ",(int)frequency,suffix_f);
		  LCDprint(buffer,2,1);
	  }

	  while(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)){
		  LCDprint("Thanks for", 1, 1);
		  LCDprint("using :]", 2, 1);
		  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)){
		  LCDprint("Thanks for", 1, 1);
		  LCDprint("using :]", 2, 1);
		  if(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)){
				  goto start;
		  }
		  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65536-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7200-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_E_Pin|LCD_RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_BUILTIN_Pin */
  GPIO_InitStruct.Pin = LED_BUILTIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BUILTIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_D4_Pin LCD_D5_Pin LCD_D6_Pin LCD_D7_Pin */
  GPIO_InitStruct.Pin = LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_E_Pin LCD_RS_Pin */
  GPIO_InitStruct.Pin = LCD_E_Pin|LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim)
{

}

void Delay_us(unsigned short us)
{
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	 HAL_TIM_Base_Start(&htim4);
	  // Wait until the timer counter reaches the calculated value
	  while (__HAL_TIM_GET_COUNTER(&htim4) < us);
	  HAL_TIM_Base_Stop(&htim4);
}

void LCD_pulse (void)
{
	HAL_GPIO_WritePin(LCD_E_GPIO_Port,LCD_E_Pin,1);
	Delay_us(40);
	HAL_GPIO_WritePin(LCD_E_GPIO_Port,LCD_E_Pin,0);
}


void LCD_byte (unsigned char x){
	//Send high nible
	if(x &	0x80) HAL_GPIO_WritePin(LCD_D7_GPIO_Port,LCD_D7_Pin,1); else HAL_GPIO_WritePin(LCD_D7_GPIO_Port,LCD_D7_Pin,0);
	if(x &	0x40) HAL_GPIO_WritePin(LCD_D6_GPIO_Port,LCD_D6_Pin,1); else HAL_GPIO_WritePin(LCD_D6_GPIO_Port,LCD_D6_Pin,0);
	if(x &	0x20) HAL_GPIO_WritePin(LCD_D5_GPIO_Port,LCD_D5_Pin,1); else HAL_GPIO_WritePin(LCD_D5_GPIO_Port,LCD_D5_Pin,0);
	if(x &	0x10) HAL_GPIO_WritePin(LCD_D4_GPIO_Port,LCD_D4_Pin,1); else HAL_GPIO_WritePin(LCD_D4_GPIO_Port,LCD_D4_Pin,0);
	LCD_pulse();
	Delay_us(40);
	//Send low nible
	if(x & 0x08) HAL_GPIO_WritePin(LCD_D7_GPIO_Port,LCD_D7_Pin,1); else HAL_GPIO_WritePin(LCD_D7_GPIO_Port,LCD_D7_Pin,0);
	if(x & 0x04) HAL_GPIO_WritePin(LCD_D6_GPIO_Port,LCD_D6_Pin,1); else HAL_GPIO_WritePin(LCD_D6_GPIO_Port,LCD_D6_Pin,0);
	if(x & 0x02) HAL_GPIO_WritePin(LCD_D5_GPIO_Port,LCD_D5_Pin,1); else HAL_GPIO_WritePin(LCD_D5_GPIO_Port,LCD_D5_Pin,0);
	if(x & 0x01) HAL_GPIO_WritePin(LCD_D4_GPIO_Port,LCD_D4_Pin,1); else HAL_GPIO_WritePin(LCD_D4_GPIO_Port,LCD_D4_Pin,0);
	LCD_pulse();
}
void WriteData (unsigned char x)
{
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port,LCD_RS_Pin,1);
	LCD_byte(x);
	HAL_Delay(2);
}

void WriteCommand (unsigned char x)
{
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port,LCD_RS_Pin,0);
	LCD_byte(x);
	HAL_Delay(5);
}
void LCD_4BIT (void)
{
	HAL_GPIO_WritePin(LCD_E_GPIO_Port,LCD_E_Pin,0); // Resting state of LCD's enable is zero
	// LCD_RW=0; // We are only writing to the LCD in this program
	HAL_Delay(20);
	// First make sure the LCD is in 8-bit mode and then change to 4-bit mode
	WriteCommand(0x33);
	WriteCommand(0x33);
	WriteCommand(0x32); // Change to 4-bit mode

	// Configure the LCD
	WriteCommand(0x28);
	WriteCommand(0x0c);
	WriteCommand(0x01); // Clear screen command (takes some time)
	HAL_Delay(20); // Wait for clear screen command to finsih.
}

void LCDprint(char * string, unsigned char line, unsigned char clear)
{
	int j;

	WriteCommand(line== 2 ? 0xc0: 0x80);
	HAL_Delay(5);
	for(j=0; string[j]!=0; j++)	WriteData(string[j]);// Write the message
	if(clear) for(; j<CHARS_PER_LINE; j++) WriteData(' '); // Clear the rest of the line
}
float get_capacitance(float period)
{
	float capacitance = (1.44*period)/(Ra + 2*Rb);
	return capacitance*1000000000;
}

char get_suffix_and_convert(float* capacitance,float* frequency,char* suffix_f){
	char suffix;
	if(*capacitance > 100){
	if(*capacitance > 1000){
			*capacitance = *capacitance/1000;
			*frequency = *frequency*1000;
			suffix = 'u';
			*suffix_f = ' ';
			goto skip;
	}
	*frequency = *frequency*1000;
	*suffix_f = ' ';
	suffix = 'n';
	}else{
		suffix = 'n';
		*suffix_f = 'k';
	}
	skip:
	return suffix;
}


/*
 __HAL_TIM_SET_COUNTER(&htim4, 0);
	 HAL_TIM_Base_Start(&htim4);
	  // Wait until the timer counter reaches the calculated value
	  while (__HAL_TIM_GET_COUNTER(&htim4) < us);
	  HAL_TIM_Base_Stop(&htim4);
 */

float get_period(void) {

    float period_one = 0;
    float period_two = 0;
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    while ((int)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0));
    while (!(int)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0));
    // Start the timer signal is 1
    HAL_TIM_Base_Start(&htim4);

    // Wait for the falling edge of the incoming signal
    while ((int)(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)));
    // Stop the timer and read the pulse width
    period_one = __HAL_TIM_GET_COUNTER(&htim4);

    while (!(int)(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)));

    period_two =  __HAL_TIM_GET_COUNTER(&htim4) - period_one;


    return (period_two + period_one)/1000000;
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
