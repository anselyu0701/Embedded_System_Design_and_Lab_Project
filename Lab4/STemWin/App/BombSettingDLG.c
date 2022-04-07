/*********************************************************************
*                                                                    *
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
*                                                                    *
**********************************************************************
*                                                                    *
* C-file generated by:                                               *
*                                                                    *
*        GUI_Builder for emWin version 5.44                          *
*        Compiled Nov 10 2017, 08:53:57                              *
*        (c) 2017 Segger Microcontroller GmbH & Co. KG               *
*                                                                    *
**********************************************************************
*                                                                    *
*        Internet: www.segger.com  Support: support@segger.com       *
*                                                                    *
**********************************************************************
*/

// USER START (Optionally insert additional includes)
// USER END

#include "DIALOG.h"

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
#define ID_WINDOW_0 (GUI_ID_USER + 0x00)
#define ID_BUTTON_0 (GUI_ID_USER + 0x01)
#define ID_BUTTON_1 (GUI_ID_USER + 0x02)
#define ID_BUTTON_2 (GUI_ID_USER + 0x03)
#define ID_BUTTON_3 (GUI_ID_USER + 0x04)
#define ID_BUTTON_4 (GUI_ID_USER + 0x05)
#define ID_BUTTON_5 (GUI_ID_USER + 0x06)
#define ID_BUTTON_6 (GUI_ID_USER + 0x07)
#define ID_BUTTON_7 (GUI_ID_USER + 0x08)
#define ID_BUTTON_8 (GUI_ID_USER + 0x09)
#define ID_BUTTON_9 (GUI_ID_USER + 0x0A)
#define ID_BUTTON_10 (GUI_ID_USER + 0x0B)
#define ID_BUTTON_11 (GUI_ID_USER + 0x0C)
#define ID_EDIT_0 (GUI_ID_USER + 0x0D)
#define ID_TEXT_0 (GUI_ID_USER + 0x0E)


// USER START (Optionally insert additional defines)
// USER END

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/

// USER START (Optionally insert additional static data)
// USER END

/*********************************************************************
*
*       _aDialogCreate
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
  { WINDOW_CreateIndirect, "BombSetting", ID_WINDOW_0, 0, 0, 480, 272, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "button_1", ID_BUTTON_0, 126, 57, 50, 50, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "button_2", ID_BUTTON_1, 182, 57, 50, 50, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "button_3", ID_BUTTON_2, 238, 57, 50, 50, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "button_4", ID_BUTTON_3, 126, 110, 50, 50, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "button_5", ID_BUTTON_4, 182, 110, 50, 50, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "button_6", ID_BUTTON_5, 238, 110, 50, 50, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "button_7", ID_BUTTON_6, 126, 163, 50, 50, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "button_8", ID_BUTTON_7, 182, 163, 50, 50, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "button_9", ID_BUTTON_8, 238, 163, 50, 50, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "button_0", ID_BUTTON_9, 294, 57, 50, 50, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "button_C", ID_BUTTON_10, 295, 110, 50, 50, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "button_E", ID_BUTTON_11, 294, 163, 50, 50, 0, 0x0, 0 },
  { EDIT_CreateIndirect, "Edit_result", ID_EDIT_0, 91, 11, 288, 40, 0, 0x64, 0 },
  { TEXT_CreateIndirect, "NTUST", ID_TEXT_0, 149, 230, 180, 28, 0, 0x64, 0 },
  // USER START (Optionally insert additional widgets)
  // USER END
};

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/

// USER START (Optionally insert additional static code)
int set_bump;
int digi_step = 0;
char str[4] = {' '};
int enter_press;

void PrintValue(char str[4], WM_MESSAGE *pMsg)
{
	WM_HWIN hItem;
	hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_0);
	EDIT_SetText(hItem, str);
	EDIT_SetTextAlign(hItem, GUI_TA_RIGHT | GUI_TA_VCENTER);
  EDIT_SetTextColor(hItem, EDIT_CI_ENABLED, GUI_MAKE_COLOR(0x00000000));
  EDIT_SetFont(hItem, GUI_FONT_32_1);
}

int str2int(char strs[4])
{
	int sum, a, b;
	if (str[1] == ' ')
	{
		sum = strs[2] - '0';
	}
	else
	{
		a = strs[1] - '0';
		b = strs[2] - '0';
		sum = (a*10) + b;
	}
	
	return sum;
}

/*  Determin which button was pressed  */
void DetermineValue(int value, WM_MESSAGE *pMsg)
{
	int count1;	
	switch (value)
	{
		case 11: // C
			str[0] = str[1] = ' ';
		  str[1] = ' ';
			str[2] = '0';
			//str[3] = '\0';
			enter_press = 0;
			break;
		case 12: // E
			set_bump = str2int(str);
			if (set_bump>4)
				enter_press = 1;
			else
				enter_press = 0;
			break;
		default: // 0~9
			if(str[1] == ' ')
			{
				if(str[2]=='0')
				{
					str[1] = ' ';
					str[2] = value + '0';
				}
				else
				{
					str[1] = str[2];
					str[2] = value + '0';
				}
			}
			else 
			{
				str[1] = str [2] = '9';
			}
			enter_press = 0;
			break;
	}
	PrintValue(str, pMsg);
}
// USER END

/*********************************************************************
*
*       _cbDialog
*/
static void _cbDialog(WM_MESSAGE * pMsg) {
  WM_HWIN hItem;
  int     NCode;
  int     Id;
  // USER START (Optionally insert additional variables)
  // USER END

  switch (pMsg->MsgId) {
  case WM_INIT_DIALOG:
    //
    // Initialization of 'button_1'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_0);
    BUTTON_SetText(hItem, "1");
    BUTTON_SetFont(hItem, GUI_FONT_20_1);
    //
    // Initialization of 'button_2'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_1);
    BUTTON_SetFont(hItem, GUI_FONT_20_1);
    BUTTON_SetText(hItem, "2");
    //
    // Initialization of 'button_3'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_2);
    BUTTON_SetFont(hItem, GUI_FONT_20_1);
    BUTTON_SetText(hItem, "3");
    //
    // Initialization of 'button_4'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_3);
    BUTTON_SetFont(hItem, GUI_FONT_20_1);
    BUTTON_SetText(hItem, "4");
    //
    // Initialization of 'button_5'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_4);
    BUTTON_SetFont(hItem, GUI_FONT_20_1);
    BUTTON_SetText(hItem, "5");
    //
    // Initialization of 'button_6'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_5);
    BUTTON_SetFont(hItem, GUI_FONT_20_1);
    BUTTON_SetText(hItem, "6");
    //
    // Initialization of 'button_7'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_6);
    BUTTON_SetFont(hItem, GUI_FONT_20_1);
    BUTTON_SetText(hItem, "7");
    //
    // Initialization of 'button_8'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_7);
    BUTTON_SetFont(hItem, GUI_FONT_20_1);
    BUTTON_SetText(hItem, "8");
    //
    // Initialization of 'button_9'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_8);
    BUTTON_SetFont(hItem, GUI_FONT_20_1);
    BUTTON_SetText(hItem, "9");
    //
    // Initialization of 'button_0'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_9);
    BUTTON_SetFont(hItem, GUI_FONT_20_1);
    BUTTON_SetText(hItem, "0");
    //
    // Initialization of 'button_C'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_10);
    BUTTON_SetFont(hItem, GUI_FONT_20_1);
    BUTTON_SetText(hItem, "C");
    //
    // Initialization of 'button_E'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_11);
    BUTTON_SetFont(hItem, GUI_FONT_20_1);
    BUTTON_SetText(hItem, "E");
    //
    // Initialization of 'Edit_result'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_0);
    EDIT_SetFont(hItem, GUI_FONT_32_1);
    EDIT_SetText(hItem, "10");
    EDIT_SetTextAlign(hItem, GUI_TA_RIGHT | GUI_TA_VCENTER);
		str[0] = ' ';
		str[1] = '1';
		str[2] = '0';
		str[3] = '\0';
    //
    // Initialization of 'NTUST'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_0);
    TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
    TEXT_SetFont(hItem, GUI_FONT_32_1);
    TEXT_SetText(hItem, "Bomb Setting");
    // USER START (Optionally insert additional code for further widget initialization)
    // USER END
    break;
  case WM_NOTIFY_PARENT:
    Id    = WM_GetId(pMsg->hWinSrc);
    NCode = pMsg->Data.v;
    switch(Id) {
    case ID_BUTTON_0: // Notifications sent by 'button_1'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
				DetermineValue(1, pMsg);
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_1: // Notifications sent by 'button_2'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
				DetermineValue(2, pMsg);
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_2: // Notifications sent by 'button_3'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
				DetermineValue(3, pMsg);
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_3: // Notifications sent by 'button_4'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
				DetermineValue(4, pMsg);
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_4: // Notifications sent by 'button_5'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
				DetermineValue(5, pMsg);
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_5: // Notifications sent by 'button_6'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
				DetermineValue(6, pMsg);
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_6: // Notifications sent by 'button_7'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
				DetermineValue(7, pMsg);
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_7: // Notifications sent by 'button_8'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
				DetermineValue(8, pMsg);
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_8: // Notifications sent by 'button_9'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
				DetermineValue(9, pMsg);
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_9: // Notifications sent by 'button_0'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
				DetermineValue(0, pMsg);
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_10: // Notifications sent by 'button_C'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
				DetermineValue(11, pMsg);
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_11: // Notifications sent by 'button_E'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
				DetermineValue(12, pMsg);
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_EDIT_0: // Notifications sent by 'Edit_result'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    // USER START (Optionally insert additional code for further Ids)
    // USER END
    }
    break;
  // USER START (Optionally insert additional message handling)
  // USER END
  default:
    WM_DefaultProc(pMsg);
    break;
  }
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       CreateBombSetting
*/
WM_HWIN CreateBombSetting(void);
WM_HWIN CreateBombSetting(void) {
  WM_HWIN hWin;

  hWin = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), _cbDialog, WM_HBKWIN, 0, 0);
  return hWin;
}

// USER START (Optionally insert additional public code)
// USER END

/*************************** End of file ****************************/
