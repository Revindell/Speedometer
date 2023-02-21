#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"
#include "stm32F4xx_tim.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//��ʱ�� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/6/16
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
typedef struct
{
	uint8_t sec;
	uint8_t min;
}Tim_TMsg;

void TIM3_Int_Init(u16 arr,u16 psc);
void Init_TMsg(Tim_TMsg* TimeData);

#endif
