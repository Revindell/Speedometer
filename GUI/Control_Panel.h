#ifndef _CONTROL_PANEL_H
#define _CONTROL_PANEL_H
#include "sys.h"
#include "WM.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "can.h"

#define ON	0x01
#define OFF	0x00

void Control_Panel(void *pvParameters);

#endif

/*********************************************************
0xA1 1010 0001
0xC2 1100 0010
0x32 0011 0010
0xF2 1111 0010
0x54 0101 0100
*********************************************************/
