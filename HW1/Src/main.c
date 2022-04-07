/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

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
int flag;
int count1, dtime;
int *dflag, dflag1;
uint8_t num[10] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Delay_10us (int us)	//10us
{
	flag = 1;
	dtime = us;
	count1 = 0;
	*dflag = 1;
	dflag1 = 1;
	for (; *dflag == 1; *dflag = dflag1);
}

void WriteLcdCom (uint8_t com)
{
	Delay_10us (2);
	GPIOB -> BSRR = 0x00ff0000;
	GPIOB -> BSRR = com;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); // write data to LCD
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // instruction input
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // initial HIGH enable
	Delay_10us (1);
	// negative edge trigger
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
	Delay_10us (1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
	Delay_10us (1);
}


void TriggerDHT11 ()
{
	HAL_GPIO_WritePin (GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	Delay_10us (2);
	HAL_GPIO_WritePin (GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	Delay_10us (1900); // delay 18ms
	HAL_GPIO_WritePin (GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	Delay_10us (3); // delay 30us
}

void WriteLcdData (uint8_t data)
{
	Delay_10us (2);
	GPIOB -> BSRR = 0x00ff0000;
	GPIOB -> BSRR = data;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); // write data to LCD
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); // data input
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // initial HIGH enable
	Delay_10us (1);
	// negative edge trigger
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
	Delay_10us (1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
	Delay_10us (1);
}

/*    Display NTUST    */
void PrintNTUST ()
{
	WriteLcdCom (0x80 + 6);
	WriteLcdData (0x4E); // 'N'
	WriteLcdData (0x54); // 'T'
	WriteLcdData (0x55); // 'U'
	WriteLcdData (0x53); // 'S'
	WriteLcdData (0x54); // 'T'	
}

/*    Display temperature and humidity    */
void PrintData (int temp, int humi)
{
	int temp_tens = temp / 10;
	int temp_digits = temp % 10;
	int humi_tens = humi / 10;
	int humi_digits = humi % 10;
	WriteLcdCom (0xC0 + 3); // write second line
	WriteLcdData (0x54); // 'T'
	WriteLcdData (0x3A); // ':'
	WriteLcdData (num[temp_tens]);
	WriteLcdData (num[temp_digits]);
	WriteLcdData (0x20); // ' '
	WriteLcdData (0x48); // 'H'
	WriteLcdData (0x3A); // ':'
	WriteLcdData (num[humi_tens]);
	WriteLcdData (num[humi_digits]);
	WriteLcdData (0x25); // '%'
}


uint8_t DHT11ReadByte ()
{
	uint8_t dataIn = 0x00;
	int counter, counter1;
	for (counter = 0 ; counter < 8; counter++)
	{
		counter1 = 0;
		while (!HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_0)); // Lo 50us
		while (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_0))
		{
			counter1 ++;
			Delay_10us (1);
		}
		
		if (counter1 >= 4)
		{
			dataIn += 0x01;
			dataIn <<= 1;
		}
		else
		{
			dataIn <<= 1;
		}
	}	
	return dataIn;
}

int DHT11ReadCheck ()
{
	int count0 = 0; // check DHT11 begin signal
	int count1 = 0;
	while (!HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_0))
	{
		count0++;
		Delay_10us (1);
	}
	if(count0 > 10 || count0 < 4)
		return 0;
	while (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_0))
	{
		count1++;
		Delay_10us (1);
	}
	if (count1 > 10 || count1 < 4)
		return 0;
	
	return 1;	
}

void DHT11ReadData ()
{
	uint8_t buff[5]; // temp*2 + humi*2 + checknum*1
	int counter;
	if (DHT11ReadCheck() == 1)
	{
		for (counter = 0; counter < 5; counter ++)		
			buff[counter] = DHT11ReadByte();
		if(buff[0] + buff[1] + buff[2] + buff[3] == buff[4]) // data is correct 		
			PrintData (buff[2]/2, buff[0]/2);
		else // data is not correct
			PrintData (0, 0);
	}
	else // dataReadcheck is not correct
	{
		PrintData (0, 0);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	flag = 0;
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
  /* USER CODE BEGIN 2 */	
	HAL_TIM_Base_Start_IT(&htim2); 	
	
	WriteLcdCom (0x38); //display two line	
	WriteLcdCom (0x06); //display setting
	WriteLcdCom (0x0C); //display on
	
	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	PrintNTUST ();
	HAL_Delay (1000); // booting scene 
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		HAL_GPIO_WritePin (GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // maintain 200ms
		TriggerDHT11 (); // Trigger DHT11
		DHT11ReadData ();
		Delay_10us (18000); // 20ms		
		HAL_GPIO_WritePin (GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // maintain 800ms
		Delay_10us (80000);
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  htim2.Init.Prescaler = 720;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB15 PB3 PB4 PB5 
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) { // 10 us
	switch (flag)
	{
		case 0:			
			break;
		
		case 1:
			if (count1 >= dtime)
			{
				count1 = 0;
				flag = 0;
				dflag1 = 0;
			}
			else
			{
				count1++;
				flag = 1;
			}
			break;
			
			default:
				break;
	}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
