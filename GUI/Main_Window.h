#ifndef _MAIN_WINDOW_H
#define _MAIN_WINDOW_H
#include "sys.h"
#include "WM.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "can.h"

void CreateMain_Window(void *pvParameters);

#endif

