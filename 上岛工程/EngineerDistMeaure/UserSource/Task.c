#include "Task.h"
#include <string.h>

#include "iwdg.h"

#include "MyCom.h"
#include "SystemState.h"
#include "bsp_can.h"
#include "bsp_Uart.h"
#include "GY-53.h"


void GY_53FillCanData(unsigned char* c);

//测量任务
void vDistMesure_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	unsigned char data[8]={0};
	while(1)
	{
		GY_53FillCanData(data);
		Can_Sent_msgToQue(1,CanSentID, data);
		vTaskDelayUntil(&xLastWakeTime,50/ portTICK_RATE_MS);
	}

}

//LED闪烁
void vLED_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	int Delay_t=500;
	while(1)
	{
				
		if(SystemState.OutLine_Flag&0x01)
			Delay_t=150;
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
	int i=0,j=0;
	while(1)
	{
		for(i=Dist1_No;i<=Dist4_No;i++)
		{
			if(MyFlagGet(SystemState.OutLine_Flag,i))//如果断线
			{
				for(j=0;j<i;j++)
				{
					vTaskDelayUntil(&xLastWakeTime,250/ portTICK_RATE_MS);
					HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
					vTaskDelayUntil(&xLastWakeTime,50/ portTICK_RATE_MS);
					HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
				}
				vTaskDelayUntil(&xLastWakeTime,1000/ portTICK_RATE_MS);
			}
		}
		
		vTaskDelayUntil(&xLastWakeTime,500/ portTICK_RATE_MS);
		HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
		vTaskDelayUntil(&xLastWakeTime,500/ portTICK_RATE_MS);
		HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
		vTaskDelayUntil(&xLastWakeTime,500/ portTICK_RATE_MS);
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
//****************************看门狗****************************
//****************************************************************
//=================================================================

void vFEEDDOG_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	vTaskDelayUntil(&xLastWakeTime,2000/ portTICK_RATE_MS);
	__HAL_IWDG_START(&hiwdg);//开启看门狗
	
	while(1)
	{
		//__HAL_IWDG_RELOAD_COUNTER(&hiwdg);//更新看门狗
		
		
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
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

		//g_mem_min=xPortGetMinimumEverFreeHeapSize();
		//g_mem_n=xPortGetFreeHeapSize();
		
		vTaskDelayUntil(&xLastWakeTime,200/ portTICK_RATE_MS);
	}
  //vTaskDelete(NULL);//退出任务
}



//=====================================================================================
//                                 其他函数
//
//
//=====================================================================================
//GY_53can发送字体填充
void GY_53FillCanData(unsigned char* c)
{
	if(MyFlagGet(SystemState.OutLine_Flag,Dist1_No))//如果断线  则发送255
		c[0]=255;
	else
		c[0]=(unsigned char)(GY_53_1.Dis/10);//如果连接则发送实际距离 以cm为单位
	
	if(MyFlagGet(SystemState.OutLine_Flag,Dist2_No))
		c[1]=255;
	else
		c[1]=(unsigned char)(GY_53_2.Dis/10);
	
	if(MyFlagGet(SystemState.OutLine_Flag,Dist3_No))
		c[2]=255;
	else
		c[2]=(unsigned char)(GY_53_3.Dis/10);
	
	if(MyFlagGet(SystemState.OutLine_Flag,Dist4_No))
		c[3]=255;
	else
		c[3]=(unsigned char)(GY_53_4.Dis/10);
	
	if(MyFlagGet(SystemState.OutLine_Flag,Dist5_No))
		c[4]=255;
	else
		c[4]=(unsigned char)(GY_53_5.Dis/10);
	
	if(MyFlagGet(SystemState.OutLine_Flag,Dist6_No))
		c[5]=255;
	else
		c[5]=(unsigned char)(GY_53_6.Dis/10);
	
	c[6]=255;
	c[7]=255;
}



//======================================
//									串口处理函数
//======================================
void UARTAnalys(UART_HandleTypeDef *huart,char* s,int len)
{

	if(huart==&huart1)
	{
		GY_53Analy(&GY_53_1,s);
		RefreshDeviceOutLineTime(Dist1_No);
	}
	if(huart==&huart2)
	{
		GY_53Analy(&GY_53_2,s);
		RefreshDeviceOutLineTime(Dist2_No);
	}
	if(huart==&huart3)
	{
		GY_53Analy(&GY_53_3,s);
		RefreshDeviceOutLineTime(Dist3_No);
	}
	if(huart==&huart4)
	{
		GY_53Analy(&GY_53_4,s);
		RefreshDeviceOutLineTime(Dist4_No);
	}
	
	if(huart==&huart5)
	{
		GY_53Analy(&GY_53_5,s);
		RefreshDeviceOutLineTime(Dist5_No);
	}
	
	if(huart==&huart6)
	{
		int mem_n=0,mem_min=0;
		if(IsStrInc(s,"GetMemoState"))
		{
			mem_min=xPortGetMinimumEverFreeHeapSize();
			mem_n=xPortGetFreeHeapSize();
			sprintf(s,"Memo remain: %d Min %d\r\n",mem_n,mem_min);
			My_print(s);
			return;
		}
		GY_53Analy(&GY_53_6,s);
		RefreshDeviceOutLineTime(Dist6_No);
	}
}

