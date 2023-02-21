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

//任务优先级
#define START_TASK_PRIO			1
//任务堆栈大小	
#define START_STK_SIZE 			64  
//任务句柄
TaskHandle_t StartTask_Handler;
//任务函数
void start_task(void *pvParameters);

//TOUCH任务
//设置任务优先级
#define TOUCH_TASK_PRIO			5
//任务堆栈大小
#define TOUCH_STK_SIZE			128
//任务句柄
TaskHandle_t TouchTask_Handler;
//touch任务
void touch_task(void *pvParameters);

//LED0任务
//设置任务优先级
#define WDG_TASK_PRIO 			6
//任务堆栈大小
#define WDG_STK_SIZE				64
//任务句柄
TaskHandle_t WdgTask_Handler;
//led0任务
void Wdg_task(void *pvParameters);

//COVER任务
//设置任务优先级
#define COVER_TASK_PRIO		4
//任务堆栈大小
#define COVER_STK_SIZE		256
//任务句柄
TaskHandle_t COVERTask_Handler;
//COVER_task任务
void CreateCOVER(void *pvParameters);

int main(void)
{
	delay_init(168);       														//延时初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 	//中断分组配置
	uart_init(115200);    														//串口波特率设置
	TIM3_Int_Init(50-1,84-1);													//初始化定时器3，周期为50us
	TFTLCD_Init();																		//初始化LCD
	TP_Init();																				//初始化触摸屏
	LED_Init();   																		//LED初始化
	KEY_Init();
	EXTIX_Init();
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_3tq,CAN_BS1_2tq,70,CAN_Mode_Normal); 					//CAN硬件初始化
	CAN_Configration();  															//数据处理函数配置
	FSMC_SRAM_Init(); 																//SRAM初始化	
	mem_init(SRAMIN); 																//内部RAM初始化
	mem_init(SRAMEX); 																//外部RAM初始化
	mem_init(SRAMCCM);																//CCM初始化
	IWDG_Init(4,500); 																//与分频数为64,重载值为250,溢出时间为0.5s	
	printf("System Is Running.\r\n");
	//创建开始任务
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄                
    vTaskStartScheduler(); 
}

void start_task(void *pvParameters)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC,ENABLE);//开启CRC时钟
	GUI_Init();  					//STemWin初始化
	WM_SetCreateFlags(WM_CF_MEMDEV);
	taskENTER_CRITICAL();           //进入临界区
	//创建EMWIN Demo任务
	xTaskCreate((TaskFunction_t )CreateCOVER,             
							(const char*    )"cover_task",           
							(uint16_t       )COVER_STK_SIZE,        
							(void*          )NULL,                  
							(UBaseType_t    )COVER_TASK_PRIO,        
							(TaskHandle_t*  )&COVERTask_Handler);  
	//创建触摸任务
	xTaskCreate((TaskFunction_t )touch_task,             
							(const char*    )"touch_task",           
							(uint16_t       )TOUCH_STK_SIZE,        
							(void*          )NULL,                  
							(UBaseType_t    )TOUCH_TASK_PRIO,        
							(TaskHandle_t*  )&TouchTask_Handler);   	
	//创建LED0任务
	xTaskCreate((TaskFunction_t )Wdg_task,             
							(const char*    )"wdg_task",           
							(uint16_t       )WDG_STK_SIZE,        
							(void*          )NULL,                  
							(UBaseType_t    )WDG_TASK_PRIO,        
							(TaskHandle_t*  )&WdgTask_Handler);  
	vTaskDelete(StartTask_Handler); //删除开始任务
	taskEXIT_CRITICAL();            //退出临界区
}

//触摸任务的任务函数
void touch_task(void *pvParameters)
{
	while(1)
	{
		GUI_TOUCH_Exec();	
		vTaskDelay(20/portTICK_RATE_MS);		//延时20ms
	}
}

//LED0任务
void Wdg_task(void *p_arg)
{
	while(1)
	{
		LED0 = !LED0;
		IWDG_Feed();//喂狗
		vTaskDelay(250/portTICK_RATE_MS);		//延时200ms
	}
}
