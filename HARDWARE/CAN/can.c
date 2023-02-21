#include "can.h" 
#include "Control_Panel.h"
//CAN初始化
//tsjw:重新同步跳跃时间单元.范围:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:时间段2的时间单元.   范围:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:时间段1的时间单元.   范围:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :波特率分频器.范围:1~1024; tq=(brp)*tpclk1
//波特率=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_Mode_Normal,普通模式;CAN_Mode_LoopBack,回环模式;
//Fpclk1的时钟在初始化的时候设置为42M,如果设置CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
//则波特率为:42M/((6+7+1)*6)=500Kbps
//返回值:0,初始化OK;
//    其他,初始化失败; 
/*********************************************************************
*
*       Static data
*
**********************************************************************
*/
/*STRUCTURES*/
CanTxMsg TxMessage;
CanRxMsg RxMsg;
CanRxMsg RxCanData;
extern Tim_TMsg MT_Data;
extern uint8_t Light[];
/*VARIABLES*/
double Velocity=0,Steps=0;
uint8_t CAN_RxFlag=0;
uint8_t risingFlag=0;
uint8_t fallingFlag=0;

/*FREERTOS*/
QueueHandle_t xCAN_V_RxMessage;
QueueHandle_t xCAN_V_TxMessage;
QueueHandle_t xCAN_V_RecordMessage;

extern SemaphoreHandle_t xCatchRecord;
extern SemaphoreHandle_t xAskRecord;
/*********************************************************************/
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
  	GPIO_InitTypeDef 			 GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN1_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif
    //使能相关时钟
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能PORTA时钟	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
	
    //初始化GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化PA11,PA12
	
	  //引脚复用映射配置
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11复用为CAN1
	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12复用为CAN1
	  
  	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;										//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=DISABLE;										//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;										//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=ENABLE;										//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;										//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;										//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode=mode;	 										//模式设置 
  	CAN_InitStructure.CAN_SJW=tsjw;												//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; 											//Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;												//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  								//分频系数(Fdiv)为brp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   								// 初始化CAN1 
    
		//配置过滤器
 	  CAN_FilterInitStructure.CAN_FilterNumber=0;	  				//过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList;  //标识符屏蔽位模式 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;			//32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;	//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;  //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);							//滤波器初始化
		
#if CAN1_RX0_INT_ENABLE
	
	  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;     // 主优先级为6
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
}   
 
#if CAN1_RX0_INT_ENABLE	//使能RX0中断
//中断服务函数			    
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxCanData;
  CAN_Receive(CAN1, CAN_FIFO0, &RxCanData);
	xQueueSendToBackFromISR( xCAN_V_RxMessage, &RxCanData, 0 );
	if(RxCanData.Data[0]!=0)
		Steps++;
}
#endif

//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//其他,失败;
u8 CAN1_Send_Msg(u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x00;	 						// 标准标识符为0
  TxMessage.ExtId=0x00;	 						// 设置扩展标示符（29位）
  TxMessage.IDE=CAN_ID_STD;		 		  // 使用扩展标识符
  TxMessage.RTR=CAN_RTR_DATA;		   	// 消息类型为数据帧，一帧8位
  TxMessage.DLC=len;		 						// 发送帧信息长度
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				  // 第一帧信息          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)return 1;
  return 0;		
}

/* 发送两个字节的数据*/
void can_tx(u8 Data1,u8 Data2)
{ 
  CanTxMsg TxMessage;  
	
  TxMessage.StdId=0x00;	//标准标识符为0x00
  TxMessage.ExtId=0x0000; //扩展标识符0x0000
  TxMessage.IDE=CAN_ID_STD;//使用标准标识符
  TxMessage.RTR=CAN_RTR_DATA;//为数据帧
  TxMessage.DLC=2;	//	消息的数据长度为2个字节
  TxMessage.Data[0]=Data1; //第一个字节数据
  TxMessage.Data[1]=Data2; //第二个字节数据 
  CAN_Transmit(CAN1,&TxMessage); //发送数据
}

//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//		 其他,接收的数据长度;
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
	if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据	
	for(i=0;i<RxMessage.DLC;i++)
	buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}
/*********************************************************************
*
*       数据处理
*
**********************************************************************
*/
void Init_RxMes(CanRxMsg *RxMessage)
{
	uint8_t ubCounter = 0;

	RxMessage->StdId = 0x00;
	RxMessage->ExtId = 0x00;
	RxMessage->IDE = CAN_ID_STD;
	RxMessage->DLC = 0;
	RxMessage->FMI = 0;
	for (ubCounter = 0; ubCounter < 8; ubCounter++)
	{
		RxMessage->Data[ubCounter] = 0x00;
	}
}

void Init_VMsg(Can_VMsg *V_Message)
{
	V_Message->V = 0;
	V_Message->V_aver = 0;
	V_Message->V_Max = 0;
	V_Message->V_Min = 0;
  V_Message->distance = 0;
	V_Message->tick = 0;
}

void Init_VRec(Can_VRecord *V_Rec)
{
	uint8_t i;
	for(i=0;i<60;i++)
	{
		V_Rec[i].V_Max=0;
		V_Rec[i].V_Min=99;
		V_Rec[i].VmaxTime=0;
		V_Rec[i].VminTime=0;
	}	
}
void CAN_Configration(void)
{
	Init_RxMes(&RxMsg);
	Init_RxMes(&RxCanData);
	
	vSemaphoreCreateBinary(	xAskRecord);
	xSemaphoreTake(xAskRecord,0);
	vSemaphoreCreateBinary(xCatchRecord);
	xSemaphoreTake(xCatchRecord,0);
	
	xCAN_V_RxMessage     = xQueueCreate( 1 , sizeof( CanRxMsg )    );
	xCAN_V_TxMessage		 = xQueueCreate( 1 , sizeof( Can_VMsg )    );
	xCAN_V_RecordMessage = xQueueCreate( 60, sizeof( Can_VRecord ) );
	
}
/*CAN总线数据处理任务函数*/
void CAN_MsgManipulation(void *pvParameters)
{
	double decimal, vmax=0, vmin=99;
	double distance=0;
	uint8_t Vmintime=0;
	uint8_t Vmaxtime=0;
	uint8_t i,j=0,Rec_Num=0;

	Can_VMsg Velocity_Msg;										//接收CAN中断数据载体，单个速度值
	Can_VRecord Velocity_Rec[60];							//最值数据仓,速度纪录
	Init_VMsg(&Velocity_Msg);									//初始化CAN接收数据缓存
	Init_VRec(Velocity_Rec);									//初始化速度数据最值数组
	
	while(1)
	{
		if(xSemaphoreTake(xCatchRecord,0)==pdPASS)
		{
			if(Rec_Num<60)
			{
				Velocity_Rec[Rec_Num].V_Max = vmax;
				Velocity_Rec[Rec_Num].V_Min = vmin;
				Velocity_Rec[Rec_Num].VmaxTime=Vmaxtime;
			  Velocity_Rec[Rec_Num].VminTime=Vmintime;
				
				Velocity_Msg.V_Max  = Velocity_Rec[Rec_Num].V_Max;
				Velocity_Msg.V_Min  = Velocity_Rec[Rec_Num].V_Min;
				
				vmax=0;	vmin=99;
				Rec_Num++;
			}
		}
		
		if(xSemaphoreTake(xAskRecord,0)==pdPASS)
		{
			for(i=0;i<60;i++)
			{
				if(Velocity_Rec[i].V_Max!=0||Velocity_Rec[i].V_Min!=99)
				{
					xQueueSendToBack( xCAN_V_RecordMessage, &(Velocity_Rec[i]), 0 );
				}
			}
		}
		
		if(uxQueueMessagesWaiting( xCAN_V_RxMessage ) != 0)
		{
/*******************数据开始接收*******************/
			xQueueReceive( xCAN_V_RxMessage, &RxMsg, 0);
//			vPrintString("QueRecie xCAN_RxMessage.\r\n");	
/*******************数据处理开始*******************/
			decimal=((double)RxMsg.Data[1])/10;
      Velocity=(RxMsg.Data[0]+decimal);

			if(Velocity!=0){
				CAN_RxFlag=1;	
				can_tx(Light[4],ON);
				xSemaphoreGive(xCatchRecord);
			}
       else {CAN_RxFlag=0;	can_tx(Light[4],OFF);}
			distance=Steps*1.725;					//UC车轮外缘周长为1.725m

			if(Velocity>=vmax)
			{
				vmax=Velocity;
				risingFlag=1;
			}
			if(Velocity<vmax&&risingFlag==1)
			{
			Vmaxtime=MT_Data.min*60+MT_Data.sec;
			risingFlag=0;
			}
			if((Velocity<vmin)&&(Velocity!=0))
			{
				vmin=Velocity;			
				if(vmin>=vmax)
				vmin=vmax;
				fallingFlag=1;
			}
			if(Velocity>=vmin&&fallingFlag==1)
			{
			Vmintime=MT_Data.min*60+MT_Data.sec;
			fallingFlag=0;
			}	
/*******************数据处理结束，装载数据*******************/
			Velocity_Msg.V      	= Velocity;
//			Velocity_Msg.V_aver 	= distance/(t/3600);
      Velocity_Msg.distance = distance;
      Velocity_Msg.tick 		= Rec_Num;
			xQueueSendToBack( xCAN_V_TxMessage, &Velocity_Msg, 0 );
//			vPrintString("QueSend xCAN_V_TransMessage.\r\n");	
		}
/*******************数据发送完成*******************/			
		
/*******************程序测试代码开始*******************/	
		if(j==1)
		{
		  Velocity_Msg.V=51;
			Velocity_Msg.V_aver = 12;
			Velocity_Msg.V_Max  = 20.0;
			Velocity_Msg.V_Min  = 13.1;
			Velocity_Rec[0].V_Max = 20;			
			Velocity_Rec[0].V_Min = 1;				Velocity_Rec[0].VmaxTime=11;  	Velocity_Rec[0].VminTime=21;			Velocity_Rec[1].V_Max = 22;
			Velocity_Rec[1].V_Min = 5;   	  	Velocity_Rec[1].VmaxTime=32 ; 	Velocity_Rec[1].VminTime=42;		Velocity_Rec[2].V_Max = 23;
			Velocity_Rec[2].V_Min = 11;				Velocity_Rec[2].VmaxTime=53;  	Velocity_Rec[2].VminTime=63;		Velocity_Rec[3].V_Max = 24;
			Velocity_Rec[3].V_Min = 2;				Velocity_Rec[3].VmaxTime=74;		Velocity_Rec[3].VminTime =85;	Velocity_Rec[4].V_Max = 33;
			Velocity_Rec[4].V_Min = 22;				Velocity_Rec[4].VmaxTime=90;			Velocity_Rec[4].VminTime=103;		Velocity_Rec[5].V_Max = 18;
			Velocity_Rec[5].V_Min = 1;				Velocity_Rec[5].VmaxTime=114; 		Velocity_Rec[5].VminTime=121;		Velocity_Rec[6].V_Max = 16;
			Velocity_Rec[6].V_Min = 8;				Velocity_Rec[6].VmaxTime=135; 		Velocity_Rec[6].VminTime=145;  	Velocity_Rec[7].V_Max = 24;
			Velocity_Rec[7].V_Min = 2; 		 		Velocity_Rec[7].VmaxTime=154;		Velocity_Rec[7].VminTime=168;		Velocity_Rec[8].V_Max = 23;
			Velocity_Rec[8].V_Min = 14;				Velocity_Rec[8].VmaxTime=174;		Velocity_Rec[8].VminTime=186;		Velocity_Rec[9].V_Max = 15;
			Velocity_Rec[9].V_Min = 7;  			Velocity_Rec[9].VmaxTime=196 ; 	Velocity_Rec[9].VminTime=208;
			
			Velocity_Rec[10].V_Max = 20;			
			Velocity_Rec[10].V_Min = 1;				Velocity_Rec[10].VmaxTime=211;  		Velocity_Rec[10].VminTime=226;			Velocity_Rec[11].V_Max = 22;
			Velocity_Rec[11].V_Min = 5;   	  Velocity_Rec[11].VmaxTime=234 ; 		Velocity_Rec[11].VminTime=245;			Velocity_Rec[12].V_Max = 23;
			Velocity_Rec[12].V_Min = 11;			Velocity_Rec[12].VmaxTime=252;  		Velocity_Rec[12].VminTime=264;		Velocity_Rec[13].V_Max = 24;
			Velocity_Rec[13].V_Min = 2;				Velocity_Rec[13].VmaxTime=277;			Velocity_Rec[13].VminTime =285;		Velocity_Rec[14].V_Max = 33;
			Velocity_Rec[14].V_Min = 22;			Velocity_Rec[14].VmaxTime=299;			Velocity_Rec[14].VminTime=306;		Velocity_Rec[15].V_Max = 18;
			Velocity_Rec[15].V_Min = 1;				Velocity_Rec[15].VmaxTime=314; 		Velocity_Rec[15].VminTime=324;		Velocity_Rec[16].V_Max = 16;
			Velocity_Rec[16].V_Min = 8;				Velocity_Rec[16].VmaxTime=337; 		Velocity_Rec[16].VminTime=347;  	Velocity_Rec[17].V_Max = 24;
			Velocity_Rec[17].V_Min = 2; 		 	Velocity_Rec[17].VmaxTime=356;			Velocity_Rec[17].VminTime=365;		Velocity_Rec[18].V_Max = 23;
			Velocity_Rec[18].V_Min = 14;			Velocity_Rec[18].VmaxTime=373;			Velocity_Rec[18].VminTime=386;		Velocity_Rec[19].V_Max = 15;
			Velocity_Rec[19].V_Min = 7;  			Velocity_Rec[19].VmaxTime=399 ; 		Velocity_Rec[19].VminTime=406;
				
			xQueueSendToBack( xCAN_V_TxMessage, &Velocity_Msg, 0 );
			j=0;
		}
/*******************程序测试代码结束*******************/	
		vTaskDelay(20/portTICK_RATE_MS);
	}
}
