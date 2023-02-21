#ifndef _RECORD_WINDOEW_H
#define _RECORD_WINDOEW_H
#include "sys.h"
#include "WM.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "can.h"
#include "VelocityForm.h"

/*11-15可自定义*/
#define WM_SUPDATE WM_USER+11 
#define WM_DATAUPDATE WM_USER+12 
void Record_Window(void *pvParameters);
#endif
