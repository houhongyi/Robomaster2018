#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "usart.h"

#define RECEIVELEN 100  
#define Judgment_RECEIVELEN 200

#define UART_SentTask_Priority 3
#define UART_AnalyTask_Priority 3

typedef enum
{
	USART_DMA_SENDOVER,//发送完成
	USART_DMA_SENDING  //正在发送
}DMAStateDef;

typedef struct  
{  
	DMAStateDef dmaSend_flag;//发送完成标志
	uint16_t rx_len;//接收长度
	uint8_t usartDMA_rxBuf[RECEIVELEN];//DMA接收缓存
	
	float Send_n;
	float Send_fps;
}USART_RECEIVETYPE; 

typedef struct  
{  
	DMAStateDef dmaSend_flag;//发送完成标志
	uint16_t rx_len;//接收长度
	uint8_t usartDMA_rxBuf[Judgment_RECEIVELEN];//DMA接收缓存
	
	float Send_n;
	float Send_fps;
}JudgmentUART_RECEIVETYPE;//裁判系统串口接收


//信息打印函数
int My_printFromISR(char* s);
int My_print(char* s);

//信息发送
int UART_SendToQueue(UART_HandleTypeDef *huart,char* data);
int UART_SendToQueueFromISR(UART_HandleTypeDef *huart,char* data);

int UART_Inite(void);//UART初始化并启动任务
#endif
