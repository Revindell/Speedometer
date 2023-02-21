#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "sram.h"
#include "tftlcd.h"
#include "led.h"
#include "key.h"
#include "exti.h"
#include "timer.h"
#include "can.h"
#include "touch.h"
#include "GUI.h"
#include "WM.h"
#include "Cover.h"
#include "DIALOG.h"
#include "FreeRTOS.h"
#include "task.h"
#include "limits.h"
#include "malloc.h"
#include "iwdg.h"

//�������ȼ�
#define START_TASK_PRIO			1
//�����ջ��С	
#define START_STK_SIZE 			64  
//������
TaskHandle_t StartTask_Handler;
//������
void start_task(void *pvParameters);

//TOUCH����
//�����������ȼ�
#define TOUCH_TASK_PRIO			5
//�����ջ��С
#define TOUCH_STK_SIZE			128
//������
TaskHandle_t TouchTask_Handler;
//touch����
void touch_task(void *pvParameters);

//LED0����
//�����������ȼ�
#define WDG_TASK_PRIO 			6
//�����ջ��С
#define WDG_STK_SIZE				64
//������
TaskHandle_t WdgTask_Handler;
//led0����
void Wdg_task(void *pvParameters);

//COVER����
//�����������ȼ�
#define COVER_TASK_PRIO		4
//�����ջ��С
#define COVER_STK_SIZE		256
//������
TaskHandle_t COVERTask_Handler;
//COVER_task����
void CreateCOVER(void *pvParameters);

int main(void)
{
	delay_init(168);       														//��ʱ��ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 	//�жϷ�������
	uart_init(115200);    														//���ڲ���������
	TIM3_Int_Init(50-1,84-1);													//��ʼ����ʱ��3������Ϊ50us
	TFTLCD_Init();																		//��ʼ��LCD
	TP_Init();																				//��ʼ��������
	LED_Init();   																		//LED��ʼ��
	KEY_Init();
	EXTIX_Init();
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_3tq,CAN_BS1_2tq,70,CAN_Mode_Normal); 					//CANӲ����ʼ��
	CAN_Configration();  															//���ݴ���������
	FSMC_SRAM_Init(); 																//SRAM��ʼ��	
	mem_init(SRAMIN); 																//�ڲ�RAM��ʼ��
	mem_init(SRAMEX); 																//�ⲿRAM��ʼ��
	mem_init(SRAMCCM);																//CCM��ʼ��
	IWDG_Init(4,500); 																//���Ƶ��Ϊ64,����ֵΪ250,���ʱ��Ϊ0.5s	
	printf("System Is Running.\r\n");
	//������ʼ����
    xTaskCreate((TaskFunction_t )start_task,            //������
                (const char*    )"start_task",          //��������
                (uint16_t       )START_STK_SIZE,        //�����ջ��С
                (void*          )NULL,                  //���ݸ��������Ĳ���
                (UBaseType_t    )START_TASK_PRIO,       //�������ȼ�
                (TaskHandle_t*  )&StartTask_Handler);   //������                
    vTaskStartScheduler(); 
}

void start_task(void *pvParameters)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC,ENABLE);//����CRCʱ��
	GUI_Init();  					//STemWin��ʼ��
	WM_SetCreateFlags(WM_CF_MEMDEV);
	taskENTER_CRITICAL();           //�����ٽ���
	//����EMWIN Demo����
	xTaskCreate((TaskFunction_t )CreateCOVER,             
							(const char*    )"cover_task",           
							(uint16_t       )COVER_STK_SIZE,        
							(void*          )NULL,                  
							(UBaseType_t    )COVER_TASK_PRIO,        
							(TaskHandle_t*  )&COVERTask_Handler);  
	//������������
	xTaskCreate((TaskFunction_t )touch_task,             
							(const char*    )"touch_task",           
							(uint16_t       )TOUCH_STK_SIZE,        
							(void*          )NULL,                  
							(UBaseType_t    )TOUCH_TASK_PRIO,        
							(TaskHandle_t*  )&TouchTask_Handler);   	
	//����LED0����
	xTaskCreate((TaskFunction_t )Wdg_task,             
							(const char*    )"wdg_task",           
							(uint16_t       )WDG_STK_SIZE,        
							(void*          )NULL,                  
							(UBaseType_t    )WDG_TASK_PRIO,        
							(TaskHandle_t*  )&WdgTask_Handler);  
	vTaskDelete(StartTask_Handler); //ɾ����ʼ����
	taskEXIT_CRITICAL();            //�˳��ٽ���
}

//���������������
void touch_task(void *pvParameters)
{
	while(1)
	{
		GUI_TOUCH_Exec();	
		vTaskDelay(20/portTICK_RATE_MS);		//��ʱ20ms
	}
}

//LED0����
void Wdg_task(void *p_arg)
{
	while(1)
	{
		LED0 = !LED0;
		IWDG_Feed();//ι��
		vTaskDelay(250/portTICK_RATE_MS);		//��ʱ200ms
	}
}
