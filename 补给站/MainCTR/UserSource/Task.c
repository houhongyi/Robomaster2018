#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include <string.h>
#include "iwdg.h"

#include "MyCom.h"
#include "SystemState.h"
#include "bsp_can.h"
#include "bsp_Uart.h"
#include "PN532.h"
#include "bsp_motor.h"

void CommandSetnToCan();//Can命令发送

//LED闪烁
void vLED_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	int Delay_t=500;
	while(1)
	{
		if(SystemState.OutLine_Flag&0x7ff)
			Delay_t=283;
		else
			Delay_t=950;
		vTaskDelayUntil(&xLastWakeTime,Delay_t/ portTICK_RATE_MS);
		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
		vTaskDelayUntil(&xLastWakeTime,50/ portTICK_RATE_MS);
		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
	}

}

//LED2闪烁
void vLED2_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	short Delay=450;
	int canSentn=0;
	while(1)
	{
		if(canSentn==g_CanSent_n)
			Delay=200;
		else
			Delay=950;
		canSentn=g_CanSent_n;
		
		vTaskDelayUntil(&xLastWakeTime,Delay/ portTICK_RATE_MS);
		HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
		vTaskDelayUntil(&xLastWakeTime,50/ portTICK_RATE_MS);
		HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
	}
}

//蜂鸣器任务
void vBEEP_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	int Delay_Being=0;
	int Delay_Silence=200;
	
	HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_SET);
	vTaskDelayUntil(&xLastWakeTime,500/ portTICK_RATE_MS);
	HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_RESET);
		
	while(1)
	{
		switch(SystemState.Beep)
		{
			case Beep_Beeping:
				Delay_Being=200;
				Delay_Silence=0;
				break;
			case Beep_Silence:
				Delay_Being=0;
				Delay_Silence=200;
				break;
		}
		
		if(Delay_Being)
		{
			HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_SET);
			vTaskDelayUntil(&xLastWakeTime,Delay_Being/ portTICK_RATE_MS);
		}
		if(Delay_Silence)
		{
			HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_RESET);
			vTaskDelayUntil(&xLastWakeTime,Delay_Silence/ portTICK_RATE_MS);
		}
	}
}

//================================================================
//****************************************************************
//****************************测试任务****************************
//****************************************************************
//=================================================================
int g_mem_min=0;//最小内存剩余
int g_mem_n=0;//当前内存剩余
void vTest_Task(void *pvParameters)
{
	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
				
		vTaskDelayUntil(&xLastWakeTime,200/ portTICK_RATE_MS);
	}

}



//PN532驱动任务
void vPN532_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		switch(PN532ReaderL.State)
		{
			case PN532_State_Outline:
				PN532_Commen_Sent(&PN532ReaderL,PN532_COM_WEAKUP);//唤醒
			break; 
			
			case PN532_State_Sleep:
				PN532_Commen_Sent(&PN532ReaderL,PN532_COM_WEAKUP);//唤醒
			break;
			
			default:
				PN532_Commen_Sent(&PN532ReaderL,PN532_COM_FindCard);//寻卡
				break;
		}
		
		switch(PN532ReaderR.State)
		{
			case PN532_State_Outline:
				PN532_Commen_Sent(&PN532ReaderR,PN532_COM_WEAKUP);//唤醒
			break; 
			
			case PN532_State_Sleep:
				PN532_Commen_Sent(&PN532ReaderR,PN532_COM_WEAKUP);//唤醒
			break;
			
			default:
				PN532_Commen_Sent(&PN532ReaderR,PN532_COM_FindCard);//寻卡
				break;
		}
			
/*
			
		
		
		if(MyFlagGet(SystemState.OutLine_Flag,NFC_Reader_R))	
			PN532_Commen_Sent(&PN532ReaderR,PN532_COM_WEAKUP);//唤醒
		else
			PN532_Commen_Sent(&PN532ReaderR,PN532_COM_FindCard);//寻卡*/
			
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
	}
}

//气缸控制任务
void vCylinder_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	long time;
	vTaskDelayUntil(&xLastWakeTime,5000/ portTICK_RATE_MS);
	Motor1.Com.En=1;
	while(1)
	{
		time=SystemState.Time;
		SystemState.Cylinde.Cylinder_UL=0;
		SystemState.Cylinde.Cylinder_L=0;
		SystemState.Cylinde.Cylinder_UR=0;
		SystemState.Cylinde.Cylinder_R=0;
		SystemState.Cylinde.Cylinder_M=0;
		
		if(!MyFlagGet(SystemState.OutLine_Flag,NFC_Reader_LCard))//如果左侧有卡
		{
			SystemState.Cylinde.Cylinder_UL=1;
			SystemState.Cylinde.Cylinder_L=1;
		}

		if(!MyFlagGet(SystemState.OutLine_Flag,NFC_Reader_RCard))//如果右侧有卡
		{
			SystemState.Cylinde.Cylinder_UR=1;
			SystemState.Cylinde.Cylinder_R=1;
		}
		
		if(MyFlagGet(SystemState.OutLine_Flag,NFC_Reader_LCard) && MyFlagGet(SystemState.OutLine_Flag,NFC_Reader_RCard))//如果左右两侧都没有卡
		{
			if(time%(20*1000) < 10*1000)
				SystemState.Cylinde.Cylinder_UL=1;
			else
				SystemState.Cylinde.Cylinder_UR=1;	
		}
		
		if(GetSystemTimer()-SystemState.NIR_Commui_t.Hero_communi_t<1000)SystemState.Cylinde.Cylinder_M=1;
		
		if(GetSystemTimer()-SystemState.NIR_Commui_t.Engneer_communi_t<1000)
			Motor1.Com.Loc=-10;
		else 
			Motor1.Com.Loc=-270;
		CommandSetnToCan();
		vTaskDelayUntil(&xLastWakeTime,20/ portTICK_RATE_MS);
	}
}

//================================================================
//****************************************************************
//****************************串口分析****************************
//****************************************************************
//=================================================================
void UARTAnalys(UART_HandleTypeDef *huart,char* s,int len)
{
	if(huart->Instance==huart1.Instance)
	{
		if(s[0]=='Y'&&s[1]=='X'&&s[2]=='C')
			SystemState.NIR_Commui_t.Hero_communi_t=GetSystemTimer();
		if(s[0]=='G'&&s[1]=='C'&&s[2]=='C')
			SystemState.NIR_Commui_t.Engneer_communi_t=GetSystemTimer();
	}
	if(huart->Instance==huart3.Instance)
	{
		RefreshDeviceOutLineTime(NFC_Reader_L);
		PN532_DataDevice(&PN532ReaderL,(unsigned char*) s,50);
	}
	if(huart->Instance==huart4.Instance)
	{
		RefreshDeviceOutLineTime(NFC_Reader_R);
		PN532_DataDevice(&PN532ReaderR,(unsigned char*) s,50);
	}
}


//================================================================
//****************************************************************
//****************************其他函数****************************
//****************************************************************
//=================================================================
void CommandSetnToCan()
{
	unsigned char d[8]={0};
	d[0]=SystemState.Cylinde.Cylinder_UL;
	d[1]=SystemState.Cylinde.Cylinder_UR;
	d[2]=SystemState.Cylinde.Cylinder_L;
	d[3]=SystemState.Cylinde.Cylinder_R;
	d[4]=SystemState.Cylinde.Cylinder_M;
	Can_Sent_msgToQue(1,Mos_CTR_Rev_ID, d);//由can1发出
}