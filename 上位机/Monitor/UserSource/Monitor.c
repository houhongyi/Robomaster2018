#include "Monitor.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <string.h>

//=============other Lib====
#include "SystemState.h"
#include "bsp_Uart.h"


CanMonitorDef CanMonitor0={0},CanMonitor1={0},CanMonitor2={0};


//===============================================
//
//							需要外部实现的
//
//===============================================


//===============================================
//
//							内部函数
//
//===============================================
void RefreshMonitor(CanMonitorDef* Moni,float timPreio)
{
	Moni->RX_fps=Moni->Rx_n/timPreio*1000.0f;
	//Moni->Rx_n=0;
}

//监视输出
inline void MonitorPrint(CanMonitorDef* Moni,float timPreio)
{
		char s[100]={0};
		
		RefreshMonitor(Moni,timPreio);
		sprintf(s,"Can ID :0x%03x Rec %d in %.2f ms FPS:%.2f LastData<%02x %02x %02x %02x %02x %02x %02x %02x>\r\n",
		Moni->Rx_ID,Moni->Rx_n,timPreio,Moni->RX_fps,
		Moni->Data[0],Moni->Data[1],Moni->Data[2],Moni->Data[3],Moni->Data[4],Moni->Data[5],Moni->Data[6],Moni->Data[7]);
		My_print(s);
		Moni->Rx_n=0;
}

void vMonitor_Task(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	float time_tem,time_prio;
	while(1)
	{
		time_tem=GetSystemTimer();
		vTaskDelayUntil(&xLastWakeTime,100/ portTICK_RATE_MS);
		time_prio=GetSystemTimer()-time_tem;
		
		MonitorPrint(&CanMonitor1,time_prio);
		
	}
}


//===============================================
//
//							外部调用
//
//===============================================

//
void Monitor_Inite()
{
	CanMonitor1.Rx_ID=0x200;
	xTaskCreate( vMonitor_Task, "vMonitor_Task", 200, NULL, 3, NULL );
}

//监视器接收到消息
inline void Monitor_Rev(CanMonitorDef* Moni,unsigned char* data)
{
	Moni->Rx_n++;
	memcpy(Moni->Data,data,7);
}

//监视器接收到消息计数
inline void Monitor_Rev_add(CanMonitorDef* Moni)
{
	Moni->Rx_n++;
}
