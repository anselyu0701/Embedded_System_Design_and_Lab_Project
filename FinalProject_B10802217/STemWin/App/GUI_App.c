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
extern  WM_HWIN CreatemenuWin(void); 
extern  WM_HWIN CreatestatusWin(void);
extern  WM_HWIN CreateeatWin(void);
extern  WM_HWIN CreatecleanWin(void);
extern  WM_HWIN CreatehealWin(void);
extern  WM_HWIN CreateshopWin(void);
extern  WM_HWIN Createshop_clothWin(void);
extern  WM_HWIN Createshop_foodWin(void);
extern  WM_HWIN Creategame_menu(void);
extern  WM_HWIN CreateGame_bomb(void);
extern  WM_HWIN Creategame_rat(void);
//extern int mode_press;
//extern int set_bump;
int mode = 0;
extern int press_mode;
TS_StateTypeDef TS_State;
GUI_PID_STATE PID_State; 

void chooseWM(int HW_Num, WM_HWIN hmenu, WM_HWIN *hstatus, WM_HWIN *heat, WM_HWIN *hclean, WM_HWIN *hheal, 
	WM_HWIN *hshop, WM_HWIN *hshop_food, WM_HWIN *hshop_cloth, WM_HWIN *hgame_menu, WM_HWIN *hgame_bomb,
	WM_HWIN *hgame_rat)
{
	switch(HW_Num)
	{
		case 1:
			WM_HideWindow(hmenu);
			(*hstatus) = CreatestatusWin();
			WM_ShowWindow(*hstatus);
			mode = 1;
			break;
		case 2:
			WM_HideWindow(hmenu);
			(*heat) = CreateeatWin();
			WM_ShowWindow(*heat);
			mode = 2;
			break;
		case 3:
			WM_HideWindow(hmenu);
			(*hclean) = CreatecleanWin();
			WM_ShowWindow(*hclean);
			mode = 3;
			break;
		case 5:
			WM_HideWindow(hmenu);
			(*hheal) = CreatehealWin();
			WM_ShowWindow(*hheal);
			mode = 5;
			break;
		case 6: // shop
			WM_HideWindow(hmenu);
			(*hshop) = CreateshopWin();
			WM_ShowWindow(*hshop);
			mode = 6;
			break;
		case 4: //game menu
			WM_HideWindow(hmenu);
			(*hgame_menu) = Creategame_menu();
			WM_ShowWindow(*hgame_menu);
			mode = 4;
			break;
		case 11: //show shop_food
			WM_DeleteWindow(*hshop);
			WM_HideWindow(hmenu);
			(*hshop_food) = Createshop_foodWin();
			WM_ShowWindow(*hshop_food);
			mode = 11;
			break;
		case 12: // show shop_cloth
			WM_DeleteWindow(*hshop);
			WM_HideWindow(hmenu);
		 (*hshop_cloth) = Createshop_clothWin();
			WM_ShowWindow(*hshop_cloth);
			mode = 12;
			break;
		case 13: // game_bomb
			WM_DeleteWindow(*hgame_menu);
			WM_HideWindow(hmenu);
		 (*hgame_bomb) = CreateGame_bomb();
			WM_ShowWindow(*hgame_bomb);
			mode = 13;
			break;
		case 14: //game_rat
			WM_DeleteWindow(*hgame_menu);
			WM_HideWindow(hmenu);
		 (*hgame_rat) = Creategame_rat();
			WM_ShowWindow(*hgame_rat);
			mode = 14;
			break;
		default:
			break;
	}
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
  
 
/* USER CODE BEGIN GRAPHICS_MainTask */
	WM_HWIN hmenu = CreatemenuWin();
	WM_HWIN hstatus;
	WM_HWIN heat;
	WM_HWIN hclean;
	WM_HWIN hheal;
	WM_HWIN hshop;
	WM_HWIN hshop_food;
	WM_HWIN hshop_cloth;
	WM_HWIN hgame_menu;
	WM_HWIN hgame_bomb;
	WM_HWIN hgame_rat;
	//
	GUI_Init();
	GUI_Clear();
	BSP_TS_Init(480, 272);
/* USER CODE END GRAPHICS_MainTask */
  while(1)
{
	switch (mode)
	{
		case 0:	//menu
			WM_DeleteWindow(hstatus);
			WM_DeleteWindow(heat);
			WM_DeleteWindow(hclean);
			WM_DeleteWindow(hheal);
			WM_DeleteWindow(hshop);
			WM_DeleteWindow(hshop_food);
			WM_DeleteWindow(hshop_cloth);
			WM_DeleteWindow(hgame_menu);
			WM_DeleteWindow(hgame_bomb);
			WM_DeleteWindow(hgame_rat);
			WM_ShowWindow(hmenu);
			TS_Input();
			chooseWM(press_mode, hmenu, &hstatus, &heat, &hclean, &hheal, &hshop, &hshop_food, &hshop_cloth, 
							 &hgame_menu, &hgame_bomb, &hgame_rat);
			press_mode = 0;
			break;
		case 1:	//status
			TS_Input();
			break;
		case 2: //eat
			
			TS_Input();
			break;
		case 3: //clean
			TS_Input();
			break;
		case 4:	//game
			TS_Input();
			if (press_mode == 13) //game_bomb
			{
				mode = 13;
				chooseWM(press_mode, hmenu, &hstatus, &heat, &hclean, &hheal, &hshop, &hshop_food, &hshop_cloth,
								 &hgame_menu, &hgame_bomb, &hgame_rat);
			}
			else if (press_mode == 14)
			{
				mode = 14;
				chooseWM(press_mode, hmenu, &hstatus, &heat, &hclean, &hheal, &hshop, &hshop_food, &hshop_cloth,
								 &hgame_menu, &hgame_bomb, &hgame_rat);
			}
			break;
		case 5: // heal
			TS_Input();
			break;
		case 6: //shop
			TS_Input();
			if (press_mode == 11)
			{
				mode = 11;
				chooseWM(press_mode, hmenu, &hstatus, &heat, &hclean, &hheal, &hshop, &hshop_food, &hshop_cloth,
								 &hgame_menu, &hgame_bomb, &hgame_rat);
			}
			else if (press_mode == 12)
			{
				mode = 12;
				chooseWM(press_mode, hmenu, &hstatus, &heat, &hclean, &hheal, &hshop, &hshop_food, &hshop_cloth,
								 &hgame_menu, &hgame_bomb, &hgame_rat);
			}
			break;
	
		case 11:
			TS_Input();
			break;
		case 12:
			TS_Input();
			break;
		case 13:	// game_bomb
			TS_Input();
			break;
		case 14: //game_rat
			TS_Input();
			break;
		default:
			break;
	}
      GUI_Delay(100);
}
}

/*************************** End of file ****************************/
