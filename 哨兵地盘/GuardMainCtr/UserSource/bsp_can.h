#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "stm32f4xx_hal.h"
#include "can.h"
#include "cmsis_os.h"



/** 
  * @brief  CAN发送或是接收的ID
  */
typedef enum
{
	ZERO,
	MOTOR_A,
	MOTOR_B,
	MOTOR_C,
	MOTOR_D,
	MOTOR_CATCH=0x08,
	
	MOTOR_1_BACK=0x205,
	MOTOR_Yaw_BACK=0x205,
	MOTOR_2_BACK=0x206,
	MOTOR_3_BACK=0x207,
	MOTOR_4_BACK=0x208,
	
	MOTOR_A_BACK=0x201,
	MOTOR_B_BACK=0x202,
	MOTOR_C_BACK=0x203,
	MOTOR_D_BACK=0x204,
	MOTOR_E_BACK=0x201,
	MOTOR_F_BACK=0x202,
	MOTOR2006_A_BACK=0x203,
	MOTOR2006_B_BACK=0x204,

	StepMOTOR      = 0x210, //驱动电机控制板
	StepMOTOR_BACK = 0x220, //驱动电机控制板返回ID
	
	CAN_MAIN_CONTROL_RECEIVE_ID   = 0x110,  //主控板接收ID
	CAN_EXTEND_RECEIVE_ID         = 0x101,  //拓展板板接收ID
	CAN_EXTEND_SENT_ID            = 0x111,  //拓展板板发送ID
	
	CAN_YUNTAI_Ctr_RXID           = 0x230, //云台板接收ID
	CAN_YUNTAI_Ctr2_RXID          = 0x232, //云台板2接收ID
	CAN_YUNTAI_Ctr3_RXID          = 0x233, //云台板3接收ID
	
	CAN_YUNTAI_Ctr_BACK           = 0x231, //云台板返回ID
	
	MOTOR_Yaw_Move=0x1ff,
	MOTOR_Move =0x200,
	MOTOR_Lift =0x200

}CAN_Message_ID;

void My_CAN1_FilterConfig(CAN_HandleTypeDef* _hcan);
void My_CAN2_FilterConfig(CAN_HandleTypeDef* _hcan);
HAL_StatusTypeDef CAN_Send_Message(CAN_HandleTypeDef* _hcan, CAN_Message_ID, uint8_t* _message);
BaseType_t Can_Sent_msgToQue(short can_num,short ID, unsigned char* date);//发送数据到Can发送队列

BaseType_t Can_Sent_msgToQueFromISR(short can_num,short ID, unsigned char* date);//发送数据到Can发送队列

int Can_Inite(void);//Can初始化 成功返回1 失败返回0

extern xQueueHandle Can_TXData_QueHandle;//Can发送数据队列
#endif
