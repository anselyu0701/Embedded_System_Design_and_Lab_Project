  /**
  ******************************************************************************
  * @file    GUI_App.c
  * @author  MCD Application Team
  * @brief   Simple demo drawing "Hello world"  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright © 2018 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#include "GUI_App.h"
#include "GUI.h"
#include "stm32746g_discovery_ts.h"
#include "stm32746g_discovery.h"
#include "ft5336.h"
#include "DIALOG.h"
extern  WM_HWIN CreateBombSetting(void); 
extern  WM_HWIN CreatePlaying(void);
//extern 	WM_HWIN CreateWindow(void);
extern int enter_press;
extern int set_bump;
int mode = 0;
TS_StateTypeDef TS_State;
GUI_PID_STATE PID_State;

void TS_Input()
{	
	TS_State.touchX[0] = TS_State.touchY[0] = 0;
	TS_State.touchDetected = 0;
	BSP_TS_GetState(&TS_State);
	if(TS_State.touchDetected)
	{
		PID_State.Pressed = 1;
		PID_State.Layer = 0;
		PID_State.x = TS_State.touchX[0];
		PID_State.y = TS_State.touchY[0];
		GUI_PID_StoreState(&PID_State);
	}
	else
	{
		PID_State.Pressed = 0;
		PID_State.x = 0;
		PID_State.y = 0;
		GUI_PID_StoreState(&PID_State);
	}
}

void GRAPHICS_MainTask(void) {

  /* 2- Create a Window using GUIBuilder */
  WM_HWIN hBombSetting = CreateBombSetting();
	WM_HWIN hPlaying;
	//WM_HWIN hWin = CreateWindow();
 
/* USER CODE BEGIN GRAPHICS_MainTask */
	GUI_Init();
	GUI_Clear();
	BSP_TS_Init(480, 272);
   
/* USER CODE END GRAPHICS_MainTask */
  while(1)
{
  switch (mode)
	{
		case 0: // setting scene
			WM_DeleteWindow(hPlaying);
			WM_ShowWindow(hBombSetting);
			TS_Input();
			if(enter_press == 1)
			{
				enter_press = 0;
				mode = 1;
				WM_HideWindow(hBombSetting);
				GUI_SetColor(GUI_BLACK);
				GUI_Clear();
				hPlaying = CreatePlaying();
				WM_ShowWindow(hPlaying);
			}
			break;
			
		case 1: // playing
			WM_HideWindow(hBombSetting);
			TS_Input();
			
			break;
		
		case 2: // wait for press usr to restart
			if (HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_11))
				mode = 0;
			break;
	}    
	GUI_Delay(100);
}
}

/*************************** End of file ****************************/
