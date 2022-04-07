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
#include "fatfs.h"
#include "DIALOG.h"
#include "stdbool.h"

extern  WM_HWIN CreateWindow(void);  
extern int button_tri; // define at "main.c"
extern int enter_press;
extern bool *SwitchPicFlag;
int button_tri_done = 0;
int mode = 0;
int PIC_Seq = 0;
bool pulse = false;

FRESULT result;
TS_StateTypeDef TS_State;
GUI_PID_STATE PID_State;

void Show_PIC( char *sFileName);
void Switch_PIC(void);

static void MPU_Config (void) {
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU attributes for SDRAM */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0xC0000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}


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
		button_tri_done = 1;
	}
	else
	{
		if(button_tri_done)
		{
			button_tri_done = 0;
			PID_State.Pressed = 0;
			PID_State.x = 0;
			PID_State.y = 0;
			GUI_PID_StoreState(&PID_State);
		}
	}
}

void GRAPHICS_MainTask(void) {

  /* 2- Create a Window using GUIBuilder */
  WM_HWIN hWin = CreateWindow();
	//WM_HWIN hWin;
/* USER CODE BEGIN GRAPHICS_MainTask */
 /* User can implement his graphic application here */
  GUI_Init();
	GUI_Clear();
	BSP_TS_Init(480, 272);
  MPU_Config();
	
/* USER CODE END GRAPHICS_MainTask */
  while(1)
{		
	if(button_tri)	//button trigger
	{
		mode = 0;
		button_tri = 0;
		GUI_Clear();		
	}
	
	switch (mode)
	{
		case 0:		
			WM_ShowWindow(hWin);
			TS_Input();
			if(enter_press == 1)
			{
				enter_press = 0;
				mode = 1;
				pulse = false;
				WM_HideWindow(hWin);
				GUI_SetColor(GUI_BLACK);
				GUI_Clear();
			}
			else
				mode = 0;
			break;
			
		case 1:
			BSP_TS_GetState(&TS_State);
			if (TS_State.touchDetected) // touch pulse 
			{
				pulse = !pulse;
				BSP_TS_ResetTouchData(&TS_State);
			}
			Switch_PIC();
			break;
	}
	
  GUI_Delay(5);
}
}

void Switch_PIC(void)
{
	switch (PIC_Seq)
	{
		case 0:
			Show_PIC("DG1.bmp");
			break;
		case 1:
			Show_PIC("DG4.bmp");
			break;
		case 2:
			Show_PIC("DG5.bmp");
			break;
		case 3:
			Show_PIC("logo_1.bmp");
			break;
		case 4:
			Show_PIC("logo_2.bmp");
			break;
	}
	//GUI_Delay(1000);
	if(!pulse)
	{
		if((*SwitchPicFlag) == true)
		{
			(*SwitchPicFlag) = false;
			PIC_Seq++;
			if(PIC_Seq > 4)
				PIC_Seq = 0;
			GUI_Clear();
		}		
	}
	
}

void Show_PIC( char *sFileName)
{
	FATFS F;
	FIL file;
	GUI_HMEM hMem;
	char *acBuffer;
	UINT br;
	FILINFO fno;
  int x,y,scale_nu,scale_de;
  int i;
	int XSize=0,YSize=0;
  GUI_MEMDEV_Handle hMemBMP;
	result=f_mount(&F,"",0);
  if(result!=FR_OK)while(1){};
  result = f_open(&file,sFileName,FA_READ);
  if(result!=FR_OK)while(1){};
		
	hMem  = GUI_ALLOC_AllocZero(1024*1024*2);
  if(!hMem)
		while(1){}
  acBuffer = GUI_ALLOC_h2p(hMem);
	if(!acBuffer)
		while(1){}		
	result = f_read(&file,acBuffer,f_size(&file),&br);
  if(result!=FR_OK)while(1){};
       
  XSize = GUI_BMP_GetXSize(acBuffer);       
  YSize = GUI_BMP_GetYSize(acBuffer);       
        
  if(XSize>480 || YSize>272)
	{
		x=480/XSize;
		y=272/YSize;
		if(x>y)
		{
			scale_nu=272;
			scale_de=YSize;
		}	
		else
		{
			scale_nu=480;
			scale_de=XSize;
		}	
	}
		else
		{
	  	scale_nu=1;
			scale_de=1;
		}
  
	GUI_BMP_DrawScaled(acBuffer, 0, 0,scale_nu,scale_de);
   
  GUI_ALLOC_Free(hMem);
  f_close(&file);      
}
/*************************** End of file ****************************/
