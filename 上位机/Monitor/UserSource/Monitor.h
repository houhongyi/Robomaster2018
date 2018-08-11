#ifndef __MONITOR_H
#define __MONITOR_H

typedef struct
{
	short Rx_ID;
	int Rx_n;
	float RX_fps;
	unsigned char Data[8];
}CanMonitorDef;

extern CanMonitorDef CanMonitor0,CanMonitor1;

void Monitor_Inite(void);

void MonitorPrint(CanMonitorDef* Moni,float timPreio);//监视输出
void Monitor_Rev(CanMonitorDef* Moni,unsigned char* data);//监视器接收到消息
void Monitor_Rev_add(CanMonitorDef* Moni);//监视器接收到消息计数
#endif
