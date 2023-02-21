#ifndef __CAN_H
#define __CAN_H
#include "sys.h"
#include "timer.h"
#include "led.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
typedef struct
{
	double V;
	double V_aver;
	double V_Max;
	double V_Min;
  double distance;
  uint16_t tick;
}Can_VMsg;

typedef struct
{
	double V_Max;
	double V_Min;
	u16 VminTime;
  u16 VmaxTime;	
}Can_VRecord;

#define CAN1_RX0_INT_ENABLE			1		 			//0,不使能;1,使能.								    
										 							 				    
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化
u8 CAN1_Send_Msg(u8* msg,u8 len);						//发送数据
u8 CAN1_Receive_Msg(u8 *buf);							//接收数据
void can_tx(u8 Data1,u8 Data2);


void CAN_Config(void);
void Init_RxMes(CanRxMsg *RxMessage);
void Init_VMsg(Can_VMsg *V_Message);
void CAN_Configration(void);
void CAN_MsgManipulation(void *pvParameters);

#endif
