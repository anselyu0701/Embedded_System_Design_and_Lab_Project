/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
/* USER CODE BEGIN Includes */
//#include "gfx.h"
//extern void tft_init(void);
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId Task1_Handle;
osThreadId Task2_Handle;
osThreadId Task3_Handle;
/* USER CODE BEGIN PV */
//static __IO struct DISPLAY_CONTEXT lcd_ctx;
static __IO uint8_t hour;
static __IO uint8_t min;
static __IO uint8_t sec;
static __IO uint8_t msec;
static __IO uint8_t changed;
static __IO uint8_t running;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CRC_Init(void);
static void MX_USART1_UART_Init(void);
void TaskClock(void const * argument);
void CalculatorTask(void* );
void ClockTask(void* );
void ScanKeyPadTask(void *);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */
void WriteLcdCom (uint8_t);
void WriteLcdData (uint8_t);
void PrintClockTitleTitle(void);
//uint8_t num[16] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};
int mode; /* 0:clock  1:calculator */
int time[3] = {12, 0, 0};
int inputQueue[50];
double calculateResult;
int inputPointer;
int clockMode = 1;
int calculatorMode = 1;
/* USER CODE BEGIN 0 */



/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  running = 0;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CRC_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
	
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of Display */
  //osThreadDef(ClockPrinter, TaskClock, osPriorityNormal, 0, 200);
  //ClockPrinterHandle = osThreadCreate(osThread(ClockPrinter), NULL);
	HAL_UART_Transmit(&huart1,"TaskCreat.. \r\n",20,300);
	
	xTaskCreate(CalculatorTask, "Calcul", configMINIMAL_STACK_SIZE, NULL, 3, &Task1_Handle );
	xTaskCreate(ClockTask, "Clock", configMINIMAL_STACK_SIZE, NULL, 3, &Task2_Handle );
	xTaskCreate(ScanKeyPadTask, "ScanKey", configMINIMAL_STACK_SIZE, NULL, 2, &Task3_Handle );
	vTaskStartScheduler();

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  //osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
	
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

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

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC13 PC14 PC15 PC0 
                           PC1 PC2 PC3 PC4 
                           PC5 PC6 PC7 PC8 
                           PC9 PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0 
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 PA4 
                           PA6 PA8 PA11 PA12 
                           PA13 PA14 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	
  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB11 PB12 PB13 PB14 
                           PB15 PB3 PB4 PB5 
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_0
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_PD01_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);


}


/* USER CODE END 4 */
void calShowLCD(int input, int iPointer)
{
	int index1;
	WriteLcdCom(0xC0+15-iPointer);
	for (index1 = 0; index1 <= iPointer+1; index1++)
	{
		switch (inputQueue[index1])
		{
			case 21:
				WriteLcdData('+');	
				break;
			case 22:
				WriteLcdData('-');	
				break;
			case 23:
				WriteLcdData('*');	
				break;
			case 24:
				WriteLcdData('/');	
				break;
			case 30:
				WriteLcdData('=');	
				break;
			case 40:
				WriteLcdData('.');	
				break;
			default:
				WriteLcdData(inputQueue[index1]+'0');	
				break;
		}
	}
}
void WriteLcdCom (uint8_t com)
{
	GPIOB -> BSRR = 0x00ff0000;
	GPIOB -> BSRR = com;
	osDelay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); // write data to LCD
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // instruction input
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // initial HIGH enable
	osDelay(1);
	// negative edge trigger
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
	osDelay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
	osDelay(2);
}

void WriteLcdData (uint8_t data)
{
	GPIOB -> BSRR = 0x00ff0000;
	GPIOB -> BSRR = data;
	osDelay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); // write data to LCD
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); // data input
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // initial HIGH enable
	osDelay(1);
	// negative edge trigger
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
	osDelay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
	osDelay(2);
}
/*     Clock     */
void PrintClockTitleTitle ()
{
	WriteLcdCom (0x01); // clear
	WriteLcdCom (0x06); //display setting
	WriteLcdCom (0x38); //display two line	
	WriteLcdCom (0x0C); //display on
	WriteLcdCom (0x80+3);
	WriteLcdData (0x4E); // 'N'
	WriteLcdData (0x54); // 'T'
	WriteLcdData (0x55); // 'U'
	WriteLcdData (0x53); // 'S'
	WriteLcdData (0x54); // 'T'		
	WriteLcdData (0x20); // ' '		
	WriteLcdData (0x43); // 'C'	
	WriteLcdData (0x4C); // 'L'	
	WriteLcdData (0x4F); // 'O'	
	WriteLcdData (0x43); // 'C'	
	WriteLcdData (0x4B); // 'K'
}

void ShowTime ()
{
	WriteLcdCom (0xC0+4);
	WriteLcdData ((time[0]/10)+'0');
	WriteLcdData ((time[0]%10)+'0');
	WriteLcdData (':');
	WriteLcdData ((time[1]/10)+'0');
	WriteLcdData ((time[1]%10)+'0');
	WriteLcdData (':');
	WriteLcdData ((time[2]/10)+'0');
	WriteLcdData ((time[2]%10)+'0');
}

void TimeCount ()
{
	time[2]++;
	if(time[2]>59)
	{
		time[2] = 0;
		time[1]++;
	}
	if (time[1] > 59)
	{
		time[1] = 0;
		time[0]++;
	}
	if (time[0] > 23)
	{
		time[0] = 0;
	}
}

//int setTimeWhich;
void setTime(int inputNum)
{
	switch (clockMode)
	{
		case 3:	//set hour
			time[0] = time[0] * 10 + inputNum;
			if(time[0]>23)
				time[0] = inputNum;			
			break;
		case 4: // set min
			time[1] = time[1] * 10 + inputNum;
			if(time[1]>59)
				time[1] = inputNum;
			break;
		case 5:	// set sec
			time[2] = time[2] * 10 + inputNum;
			if(time[2]>59)
				time[2] = inputNum;
			break;
		default:
			break;
	}
}
/*    Calculator    */
void PrintCalculatorTitle()
{
	WriteLcdCom (0x01); // clear
	WriteLcdCom (0x06); //display setting
	WriteLcdCom (0x38); //display two line	
	WriteLcdCom (0x0C); //display on
	WriteLcdCom (0x80+3);
	WriteLcdData (0x4E); // 'N'
	WriteLcdData (0x54); // 'T'
	WriteLcdData (0x55); // 'U'
	WriteLcdData (0x53); // 'S'
	WriteLcdData (0x54); // 'T'		
	WriteLcdData (0x20); // ' '		
	WriteLcdData (0x43); // 'C'	
	WriteLcdData ('A'); // 'A'	
	WriteLcdData ('L'); // 'L'	
	WriteLcdData ('C'); // 'C'	
}

void clearInputQueue()
{
	int index1, index2;
	WriteLcdCom(0xC0);
	for (index1 = 0; index1 < 50; index1++)
		inputQueue[index1] = 0;
	for (index2 = 0; index2 < 16; index2++)
		WriteLcdData(' ');
	inputPointer = 0;
}

void showResultatLCD()
{
	int intResult;
	int index4;
	char chResult[20];
	int chPointer = 0;
	for (index4 = 0; index4 < 16; index4++)
		WriteLcdData(' ');
	if (calculateResult < 0)
	{
		chResult[chPointer++] = '-';
		calculateResult = -calculateResult;
	}
	int t = calculateResult;
	int t1 = calculateResult;
	// integer
	int i, q = 1;
	int index1;
	for (i=0; q>0; i++)
	{
		q = t / pow(10.0, i);
	}
	if(i == 1)
	{
		WriteLcdData('0');
		chResult[chPointer] = '0';
		chPointer++;
	}
	else
	{
		i-=2;
		for (index1 = i; index1 >= 0; index1--)
		{
			intResult = (t / (pow(10.0, index1)));
			t = t - (intResult * pow(10.0, index1));
			chResult[chPointer++] = intResult+'0';
			//WriteLcdData(intResult+'0');
		}	
	}
	// float
	double f = calculateResult - t1;
	int index2, x, index3;
	for (index2 = 1;index2 <= (10-i); index2++)
	{
		f = f * 10;
		x = f;
		f = f - x;
		if(index2 == 1)
		{
			chResult[chPointer++] = '.';
		}
		if (index2 <= (9-i))
			chResult[chPointer++] = x+'0';
		if(index2 == (10-i) && x>4)
			chResult[10-i]++;
			
	}
	WriteLcdCom(0xC0+16-chPointer);
	for	(index3 = 0; index3<chPointer; index3++)
	{
		WriteLcdData(chResult[index3]);
	}
	
}
void calculate(double number[], int operation, int inputLength)
{
	bool isError = false;
	bool bignum = false;
	switch (operation)
	{
		case 21: // +
			calculateResult = number[0] + number[1];
			break;
		case 22: // -
			calculateResult = number[0] - number[1];
			break;
		case 23: // *
			if (number[0] == 789456123 || number[1] == 789456123)
			{
				clearInputQueue();
				WriteLcdCom(0xC0+6);
				WriteLcdData(3+'0');
				WriteLcdData(5+'0');
				WriteLcdData(9+'0');
				WriteLcdData(9+'0');
				WriteLcdData(9+'0');
				WriteLcdData(1+'0');
				WriteLcdData(9+'0');
				WriteLcdData(9+'0');
				WriteLcdData(2+'0');
				WriteLcdData(0+'0');
				bignum = true;
			}
			else
				calculateResult = number[0] * number[1];
			break;
		case 24: // /
			if (number[1] == 0)
			{
				if (number[0] == 0)
					calculateResult = 0.0;
				else
					isError = true;
			}
			else 
				calculateResult = number[0] / number[1];
			break;
	}
    if(!isError && !bignum)
       showResultatLCD();
		else if (isError) 
		{
			clearInputQueue();
			WriteLcdCom(0xC0+11);
			WriteLcdData('e');
			WriteLcdData('r');
			WriteLcdData('r');
			WriteLcdData('o');
			WriteLcdData('r');
		}
}

void calculatorFunction(int inputLength)
{
	int index1;
	double number[2] = {0.0, 0.0};
	int numberSelected = 0; // start from number[0]
	int operation = 25; // + - * /, noInput = 25
	bool intOrDeci = true; // int = T, Deci = F
	double powPointer = 0.0; // start from digits
	
	for (index1 = 0; index1 < inputLength; index1++)
	{
		switch (inputQueue[index1])
		{
			case 21: // +
				operation = 21;
				numberSelected = 1;
				intOrDeci = true;
				powPointer = 0.0;
				break;
			case 22: // -
				operation = 22;
				numberSelected = 1;
				intOrDeci = true;
				powPointer = 0.0;
				break;
			case 23: // *
				operation = 23;
				numberSelected = 1;
				intOrDeci = true;
				powPointer = 0.0;
				break; 
			case 24: // /
				operation = 24;
				numberSelected = 1;
				intOrDeci = true;
				powPointer = 0.0;
				break;
			case 30: // =
				calculate(number, operation, inputLength);
				break;
			case 40: // .
				intOrDeci = false;
				powPointer = 1.0;
				break;
			default: // 0~9
				if(intOrDeci) //int
				{
					number[numberSelected] = number[numberSelected] * 10 + inputQueue[index1];
					powPointer += 1.0;
				}
				else //float
				{
					number[numberSelected] = inputQueue[index1] * pow(0.1, powPointer) + number[numberSelected];
					powPointer += 1.0;
				}
				break;
		}
	}
}
/*    scan KeyPad    */
void determineInput(int row, int col)
{
	//int inputPointer;
	if (row == 0 && col == 1)
	{
		if (mode == 0)
		{
			setTime(7);
		}
		else
		{
			inputQueue[inputPointer] = 7;
			calShowLCD(7, inputPointer);
			inputPointer++;
		}		
	}
	else if (row == 0 && col == 2)
	{
		if (mode == 0)
		{
			setTime(8);
		}
		else
		{
			inputQueue[inputPointer] = 8;
			calShowLCD(8, inputPointer);
			inputPointer++;
		}		
	}
	else if (row == 0 && col == 3)
	{
		if (mode == 0)
		{
			setTime(9);
		}
		else
		{
			inputQueue[inputPointer] = 9;
			calShowLCD(9, inputPointer);
			inputPointer++;
		}		
	}
	else if (row == 0 && col == 4)
	{
		inputQueue[inputPointer] = 23;
		calShowLCD(23, inputPointer);
		inputPointer++;
	}
	else if (row == 0 && col == 5)
	{
		inputQueue[inputPointer] = 24;
		calShowLCD(24, inputPointer);
		inputPointer++;
	}
	else if (row == 1 && col == 1)
	{
		if (mode == 0)
		{
			setTime(4);
		}
		else
		{
			inputQueue[inputPointer] = 4;
			calShowLCD(4, inputPointer);
			inputPointer++;
		}		
	}
	else if (row == 1 && col == 2)
	{
		if (mode == 0)
		{
			setTime(5);
		}
		else
		{
			inputQueue[inputPointer] = 5;
			calShowLCD(5, inputPointer);
			inputPointer++;
		}		
	}
	else if (row == 1 && col == 3)
	{
		if (mode == 0)
		{
			setTime(6);
		}
		else
		{
			inputQueue[inputPointer] = 6;
			calShowLCD(6, inputPointer);
			inputPointer++;
		}		
	}
	else if (row == 1 && col == 4)
	{
		inputQueue[inputPointer] = 22;
		calShowLCD(22, inputPointer);
		inputPointer++;
	}
	else if (row == 2 && col == 1)
	{
		if (mode == 0)
		{
			setTime(1);
		}
		else
		{
			inputQueue[inputPointer] = 1;
			calShowLCD(1, inputPointer);
			inputPointer++;
		}		
		
	}
	else if (row == 2 && col == 2)
	{
		if (mode == 0)
		{
			setTime(2);
		}
		else
		{
			inputQueue[inputPointer] = 2;
			calShowLCD(2, inputPointer);
			inputPointer++;
		}		
		
	}
	else if (row == 2 && col == 3)
	{
		if (mode == 0)
		{
			setTime(3);
		}
		else
		{
			inputQueue[inputPointer] = 3;
			calShowLCD(3, inputPointer);
			inputPointer++;
		}		
		
	}
	else if (row == 3 && col == 4)
	{
		inputQueue[inputPointer] = 21;
		calShowLCD(21, inputPointer);
		inputPointer++;
	}
	else if (row == 3 && col == 1)
	{
		if (mode == 0)
		{
			setTime(0);
		}
		else
		{
			inputQueue[inputPointer] = 0;
			calShowLCD(0, inputPointer);
			inputPointer++;
		}		
	}
	else if (row == 3 && col == 2)
	{
		inputQueue[inputPointer] = 40;
		calShowLCD(40, inputPointer);
		inputPointer++;
	}
	else if (row == 3 && col == 3) // =
	{
		if(mode == 1)
		{
			inputQueue[inputPointer] = 30;
			calShowLCD(30, inputPointer);
			inputPointer++;
			calculatorFunction(inputPointer);
		}
	}
	else if (row == 0 && col == 0) // clear
	{
		if (mode == 0)
		{
			
		}
		else
		{
			clearInputQueue();
			inputPointer = 0;
			WriteLcdCom(0xC0+15);
			WriteLcdData('0');
		}
	}
	else if (row == 3 && col == 0)  // ? set time
	{
		if (clockMode == 1 || clockMode == 2)
		{
			clockMode = 3;
		}
		else if (clockMode == 5)
			clockMode = 2;
		else
		{
			clockMode++;
		}
	}
	else if (row == 2 && col == 0)
	{
		if (mode == 0)
		{
			clockMode = 6;
			mode = 1;
			calculatorMode = 1;
			//inputPointer = 0;
		}
		else
		{
			clockMode = 1;
			mode = 0;
		}
	}
}

void whichPress(int row)
{
	int index1;
	bool pressed = false;
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
		determineInput(row, 0);
	else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1))
		determineInput(row, 1);
	else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2))
		determineInput(row, 2);
	else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3))
		determineInput(row, 3);
	else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4))
		determineInput(row, 4);
	else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5))
		determineInput(row, 5);
	
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) || HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) || HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)
		|| HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) || HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) || HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5));
	//osDelay(20);
}

void scanKeyPad()
{
	//int index1;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	whichPress(0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	//
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET);
	whichPress(1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET);
	//
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_SET);
	whichPress(2);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_RESET);
	//
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	whichPress(3);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
}
/*     Task Function *    */
void ScanKeyPadTask(void* t)
{
	while(1)
	{
		scanKeyPad();
		osDelay(20);
		HAL_UART_Transmit(&huart1,"ScanKeyPad.. \r\n",16,300);		
	}
}

void CalculatorTask(void* t)
{
	while(1)
	{
		if (mode == 1)
		{
			switch (calculatorMode)
			{
				case 1: // init
					PrintCalculatorTitle();
					calculatorMode++;
					clearInputQueue();
					break;
				case 2: // waiting for enter
					
					break;
			}
		}
		//calculatorFunction(10);
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
		HAL_UART_Transmit(&huart1,"Calculator.. \r\n",16,300);
		osDelay(20);		
	}
}

/* TaskClock function */

void ClockTask(void* t)
{
  /* USER CODE BEGIN 5 */
	/*   initLCD    */
  /* Infinite loop */
  for(;;)
  {
		switch (clockMode)
		{
			case 1: //Initial show Title
				PrintClockTitleTitle();
				clockMode++;
				break;
			case 2: // time Running
				TimeCount();
				ShowTime();	
				osDelay(100);
				break;
			case 3: // set hour
				WriteLcdCom (0xC0+4);
				WriteLcdData(' ');
				WriteLcdData(' ');
				osDelay(50);
				WriteLcdCom (0xC0+4);
				WriteLcdData((time[0]/10)+'0');
				WriteLcdData((time[0]%10)+'0');
				osDelay(50);
				break;
			case 4: // set minuite
				WriteLcdCom (0xC0+7);
				WriteLcdData(' ');
				WriteLcdData(' ');
				osDelay(50);
				WriteLcdCom (0xC0+7);
				WriteLcdData((time[1]/10)+'0');
				WriteLcdData((time[1]%10)+'0');
				osDelay(50);
				break;
			case 5: // set second
				WriteLcdCom (0xC0+10);
				WriteLcdData(' ');
				WriteLcdData(' ');
				osDelay(50);
				WriteLcdCom (0xC0+10);
				WriteLcdData((time[2]/10)+'0');
				WriteLcdData((time[2]%10)+'0');
				osDelay(50);
				break;
			default: // calculator mode
				TimeCount();
				osDelay(100);
				break;
		}	
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_15);
		HAL_UART_Transmit(&huart1,"Clock.. \r\n",16,300);		
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
   
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
