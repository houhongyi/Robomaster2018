#ifndef __BSP_Task_H__
#define __BSP_Task_H__

#include "cmsis_os.h"
#include "bsp_motor.h"


typedef enum
{
	Motor_3508,
	Mos
}ComDeviceType;

typedef struct{
	ComDeviceType Device_type;
	char Ctr_Type;
	short Data;
}DiviceComDef;

typedef struct{
	DiviceComDef Motor1;
	DiviceComDef Motor2;
	DiviceComDef Motor3;
	DiviceComDef Motor4;
	DiviceComDef MotorA;
	DiviceComDef MotorB;
	DiviceComDef MotorC;
	DiviceComDef MotorD;
	DiviceComDef MotorE;
	DiviceComDef MotorF;
}TaskComDef;

extern TaskComDef TaskCom;
extern xQueueHandle TaskComD_QueHandle;//顺序执行电机任务队列

void bsp_Task_Inite(void);
void InsertTask(short TaskMatrix[][2][11] ,TaskComDef* TaskCom);

#endif
