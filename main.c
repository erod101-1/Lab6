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
#include "defs.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_TIM4_Init(void);


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
  char buffer[CHARS_PER_LINE];
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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  sprintf(buffer,"Capacitance: ");
	  LCDprint(buffer,1,1);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim4.Init.Prescaler = 0;
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

/* USER CODE BEGIN 4 */
/*
	#define LCD_RS GPIO_Pin_8
	#define LCD_E  GPIO_Pin_15
	#define LCD_D4 GPIO_Pin_14
	#define LCD_D5 GPIO_Pin_12
	#define LCD_D6 GPIO_Pin_3
	#define LCD_D7 GPIO_Pin_4
 */
void delay_us_sys(uint32_t us)
{
    SysTick_Config(SystemCoreClock / 1000000); // Set SysTick timer to interrupt every microsecond
    __disable_irq(); // Disable interrupts
    uint32_t start_tick = SysTick->VAL; // Get current SysTick value
    while ((start_tick - SysTick->VAL) < us); // Wait for the specified number of microseconds
    __enable_irq(); // Re-enable interrupts
}

void LCD_Pulse(void){
	HAL_GPIO_WritePin(LCD_E_PORT,LCD_E, 1);
	delay_us_sys(40);
	HAL_GPIO_WritePin(LCD_E_PORT,LCD_E, 0);
}

void LCD_byte(unsigned char x)
{
	//Send high nible
	if(x&0x80) {
		HAL_GPIO_WritePin(LCD_D7_PORT,LCD_D7,1);
	}else{
		HAL_GPIO_WritePin(LCD_D7_PORT,LCD_D7,0);
	}
	if(x&0x40){
		HAL_GPIO_WritePin(LCD_D6_PORT,LCD_D6,1);
	}else{
		HAL_GPIO_WritePin(LCD_D6_PORT,LCD_D6,0);
	}
	if(x&0x20){
		HAL_GPIO_WritePin(LCD_D5_PORT,LCD_D5,1);
	}else{
		HAL_GPIO_WritePin(LCD_D5_PORT,LCD_D5,0);
	}
	if(x&0x10){
		HAL_GPIO_WritePin(LCD_D4_PORT,LCD_D4,1);
	}else{
		HAL_GPIO_WritePin(LCD_D4_PORT,LCD_D4,0);
	}
	LCD_Pulse();
	delay_us_sys(40);
	//Send low nible
	if(x&0x08){
		HAL_GPIO_WritePin(LCD_D7_PORT,LCD_D7,1);
	}else{
		HAL_GPIO_WritePin(LCD_D7_PORT,LCD_D7,0);
	}
	if(x&0x04){
		HAL_GPIO_WritePin(LCD_D6_PORT,LCD_D6,1);
	}else{
		HAL_GPIO_WritePin(LCD_D6_PORT,LCD_D6,0);
	}
	if(x&0x02){
		HAL_GPIO_WritePin(LCD_D5_PORT,LCD_D5,1);
	}else{
		HAL_GPIO_WritePin(LCD_D5_PORT,LCD_D5,0);
	}
	if(x&0x01){
		HAL_GPIO_WritePin(LCD_D4_PORT,LCD_D4,1);
	}else{
		HAL_GPIO_WritePin(LCD_D4_PORT,LCD_D4,0);
	}
	LCD_Pulse();
}

void WriteData(unsigned char x)
{
	HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS,1);
	LCD_byte(x);
	HAL_Delay(2);
}

void WriteCommand(unsigned char x)
{
	HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS,0);
}

void LCDprint(char * string, unsigned char line, unsigned char clear)
{
	int j;

	WriteCommand(line==2?0xc0:0x80);
	HAL_Delay(5);
	for(j=0; string[j]!=0; j++)	WriteData(string[j]);// Write the message
	if(clear) for(; j<CHARS_PER_LINE; j++) WriteData(' '); // Clear the rest of the line
}

void LCD_4BIT (void)
{
	HAL_GPIO_WritePin(LCD_E_PORT, LCD_E,0); // Resting state of LCD's enable is zero
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

float get_capacitanace(float period)
{
	float capacitance = (1.44*period)/(Ra + 2*Rb);
	capacitance = capacitance*1000000000;
	return capacitance;
}

char get_suffix_and_convert(float* capacitance){
	char suffix;
	if(*capacitance > 1000){
		*capacitance = *capacitance/1000;
		suffix = 'u';
	}else{
		suffix = 'n';
	}
	return suffix;
}
/*
* @brief  Enables or disables the specified TIM peripheral.
* @param  TIMx: where x can be 1 to 17 to select the TIMx peripheral.
* @param  NewState: new state of the TIMx peripheral.
*   This parameter can be: ENABLE or DISABLE.
* @retval None
*/

float get_period(void){
	float period = 0;

	return period;
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
