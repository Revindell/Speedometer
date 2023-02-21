/*********************************************************************
*                                                                    *
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
*                                                                    *
**********************************************************************
*                                                                    *
* C-file generated by:                                               *
*                                                                    *
*        GUI_Builder for emWin version 5.30                          *
*        Compiled Jul  1 2015, 10:50:32                              *
*        (c) 2015 Segger Microcontroller GmbH & Co. KG               *
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
#include "Cover.h"
#include "Main_Window.h"
#include "Record_Window.h"
#include "Details_Window.h"
#include "Control_Panel.h"

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
#define ID_WINDOW_0         (GUI_ID_USER + 0x00)
#define ID_TEXT_0         	(GUI_ID_USER + 0x01)
#define ID_TEXT_1         	(GUI_ID_USER + 0x02)
#define ID_TEXT_2         	(GUI_ID_USER + 0x03)
//#define ID_PROGBAR_0        (GUI_ID_USER + 0x03)
/*********************************************************************
*
*       Static data
*
**********************************************************************
*/
// USER START (Optionally insert additional static data)
uint8_t P_Progress = 0;

TaskHandle_t Main_WinHandle;
TaskHandle_t Record_WinHandle;
TaskHandle_t Control_WinHandle;

SemaphoreHandle_t xMaWinFlag_ON;
SemaphoreHandle_t xMaWinFlag_OFF;
SemaphoreHandle_t xRecWinFlag_ON;
SemaphoreHandle_t xRecWinFlag_OFF;
SemaphoreHandle_t xCtrlWinFlag_ON;
SemaphoreHandle_t xCtrlWinFlag_OFF;

SemaphoreHandle_t xAskRecord;
SemaphoreHandle_t xCatchRecord;

void MainWindow(void);
// USER END
/*********************************************************************
*
*       _aDialogCreate
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
  { WINDOW_CreateIndirect, "COVER", ID_WINDOW_0, 0, 0, 480, 320, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "SPEEDOMETRE", ID_TEXT_0, 130, 65, 220, 60, 0, 0x64, 0 },
  { TEXT_CreateIndirect, "ZEALEC", ID_TEXT_1, 0, 0, 80, 20, 0, 0x64, 0 },
	{ TEXT_CreateIndirect, "Loading", ID_TEXT_2, 115, 140, 260, 160, 0, 0x64, 0 },
//  { PROGBAR_CreateIndirect, "Progbar", ID_PROGBAR_0, 160, 165, 160, 30, 0, 0x0, 0 },
};
/*********************************************************************
*
*       Static code
*
**********************************************************************
*/

// USER START (Optionally insert additional static code)
void Cover_PaintDialog(WM_MESSAGE * pMsg)
{
		GUI_DrawGradientV(0,0,480,320,GUI_MAKE_COLOR(0x00C53858),GUI_DARKGREEN);
}
// USER END

/*********************************************************************
*
*       _cbDialog
*/
static void _cbDialog(WM_MESSAGE * pMsg) {
  WM_HWIN hItem;

  switch (pMsg->MsgId) {
	case WM_PAINT:
		Cover_PaintDialog(pMsg);
		break;
  case WM_INIT_DIALOG:
    //
    // Initialization of 'COVER'
    //
    hItem = pMsg->hWin;
//    WINDOW_SetBkColor(hItem, GUI_MAKE_COLOR(0x005838C5));
    //
    // Initialization of 'TEXT'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_0);
    TEXT_SetText(hItem, "SPEEDOMETRE");
    TEXT_SetFont(hItem, GUI_FONT_32B_ASCII);
    TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
    TEXT_SetTextColor(hItem, GUI_MAKE_COLOR(0x00FFFFFF));
    //
    // Initialization of 'TEXT'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_1);
    TEXT_SetText(hItem, "ZEALEC");
    TEXT_SetTextColor(hItem, GUI_MAKE_COLOR(0x00D5D5D5));
    TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
    TEXT_SetFont(hItem, GUI_FONT_20B_ASCII);
    //
    // Initialization of 'TEXT'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_2);
    TEXT_SetText(hItem, "Loading...");
    TEXT_SetTextColor(hItem, GUI_MAKE_COLOR(0x00D5D5D5));
    TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
    TEXT_SetFont(hItem, GUI_FONT_COMIC24B_1);
//    //
//    // Initialization of 'Progbar'
//    //
//    hItem = WM_GetDialogItem(pMsg->hWin, ID_PROGBAR_0);
//    PROGBAR_SetFont(hItem, GUI_FONT_20F_ASCII);
    break;
//	case WM_TIMER:
//		WM_RestartTimer(pMsg->Data.v,10);
//		P_Progress=P_Progress+1;
//		PROGBAR_SetValue(WM_GetDialogItem(hItem,ID_PROGBAR_0),P_Progress);
//		WM_InvalidateWindow(hItem);
//		break;
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
*       CreateCOVER
*/
void CreateCOVER(void *pvParameters);
void CreateCOVER(void *pvParameters)	
{
//	GUI_CURSOR_Show(); //��ʾ���
  WM_HWIN hWin;
	WM_SetCreateFlags(WM_CF_MEMDEV);  						/* Use memory devices on all windows to avoid flicker */
	WM_SetDesktopColor(GUI_BLACK);      					/* Automacally update desktop window */
  hWin = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), _cbDialog, WM_HBKWIN, 0, 0);
//	WM_CreateTimer(WM_GetClientWindow(hWin),0,10,0);
//	while(P_Progress<99)
//	{	
//		GUI_Delay(20);
//	}
	GUI_Delay(500/portTICK_RATE_MS);
	GUI_EndDialog(hWin, 1);
	
	vSemaphoreCreateBinary( xMaWinFlag_ON );
	vSemaphoreCreateBinary( xMaWinFlag_OFF );
	vSemaphoreCreateBinary( xRecWinFlag_ON );
	vSemaphoreCreateBinary( xRecWinFlag_OFF );
	vSemaphoreCreateBinary( xCtrlWinFlag_ON );
	vSemaphoreCreateBinary( xCtrlWinFlag_OFF );
	
	xSemaphoreTake(xMaWinFlag_ON,0);
	xSemaphoreTake(xMaWinFlag_OFF,0);
	xSemaphoreTake(xRecWinFlag_ON,0);
	xSemaphoreTake(xRecWinFlag_OFF,0);
	xSemaphoreTake(xCtrlWinFlag_ON,0);
	xSemaphoreTake(xCtrlWinFlag_OFF,0);
	
	xSemaphoreGive(xMaWinFlag_ON);
		
	taskENTER_CRITICAL();           //�����ٽ���
	xTaskCreate( CreateMain_Window,"Main_Window", 512, NULL, 4, &Main_WinHandle ); 
	xTaskCreate( CAN_MsgManipulation,"CAN_MsgManipulation",1024, NULL, 4, NULL );
	xTaskCreate( Record_Window,"Record_Window", 1024, NULL, 4, &Record_WinHandle ); 
	vTaskSuspend( Record_WinHandle );
	xTaskCreate( Control_Panel,"Control_Panel", 512, NULL, 4, &Control_WinHandle);
	vTaskSuspend( Control_WinHandle );
	vTaskDelete( NULL );
	taskEXIT_CRITICAL();            //�˳��ٽ���
	GUI_Delay(100);
}
// USER START (Optionally insert additional public code)}
// USER END
/*************************** End of file ****************************/