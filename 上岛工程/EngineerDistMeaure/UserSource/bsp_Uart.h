#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "usart.h"

#define RECEIVELEN 30  

#define UART_SentTask_Priority 3
#define UART_AnalyTask_Priority 3

typedef enum
{
	USART_DMA_SENDOVER,//�������
	USART_DMA_SENDING  //���ڷ���
}DMAStateDef;

typedef struct  
{  
	DMAStateDef dmaSend_flag;//������ɱ�־
	uint16_t rx_len;//���ճ���
	uint8_t usartDMA_rxBuf[RECEIVELEN];//DMA���ջ���
	
	float Send_n;
	float Send_fps;
}USART_RECEIVETYPE; 




//��Ϣ��ӡ����
int My_printFromISR(char* s);
int My_print(char* s);

//��Ϣ����
int UART_SendToQueue(UART_HandleTypeDef *huart,char* data);
int UART_SendToQueueFromISR(UART_HandleTypeDef *huart,char* data);

int UART_Inite(void);//UART��ʼ������������

extern USART_RECEIVETYPE UsartType1,UsartType2,UsartType3,UsartType4,UsartType5,UsartType6; 


#endif
