#include "timer.h"
#include "led.h"
#include "usart.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

QueueHandle_t xTIM_T_TxMessage;
extern uint8_t CAN_RxFlag;
extern SemaphoreHandle_t xCatchRecord;
uint32_t count=0;
Tim_TMsg TimeData;

void Init_TMsg(Tim_TMsg* TimeData)
{
	TimeData->min = 0;
	TimeData->sec = 0;
}
//通用定时器3中断初始化
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟
	
	TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM3,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x06; //抢占优先级6
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	Init_TMsg(&TimeData);
	
	xTIM_T_TxMessage = xQueueCreate( 1, sizeof( Tim_TMsg ) );
	while(xTIM_T_TxMessage==NULL);
}

//定时器3中断服务函数
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{
		if(CAN_RxFlag==1)
			{
				count++;
				if(count==20000)
				{
					count=0;
					TimeData.sec++;
					if(TimeData.sec==60)
					{
						TimeData.sec=0;
						TimeData.min++;
					}
					
					xQueueSendToBackFromISR( xTIM_T_TxMessage, &TimeData, 0 );
				}
			}
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
}
