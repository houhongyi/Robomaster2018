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

void MonitorPrint(CanMonitorDef* Moni,float timPreio);//�������
void Monitor_Rev(CanMonitorDef* Moni,unsigned char* data);//���������յ���Ϣ
void Monitor_Rev_add(CanMonitorDef* Moni);//���������յ���Ϣ����
#endif
