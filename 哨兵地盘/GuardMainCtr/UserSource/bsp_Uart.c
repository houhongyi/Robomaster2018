#include "bsp_Uart.h"
#include "string.h"

//====== other Lib
#include "Remote.h"
#include "Judgment.h"


USART_RECEIVETYPE UsartType2={0},UsartType3={0},UsartType4={0},UsartType6={0}; 
JudgmentUART_RECEIVETYPE UsartType1={0};

xQueueHandle UART1_TXData_QueHandle;//UART发送队列
xQueueHandle UART1_RXData_QueHandle;//UART接收队列

xQueueHandle UART2_TXData_QueHandle;//UART发送队列
xQueueHandle UART2_RXData_QueHandle;//UART接收队列

xQueueHandle UART3_TXData_QueHandle;//UART发送队列
xQueueHandle UART3_RXData_QueHandle;//UART接收队列

xQueueHandle UART4_TXData_QueHandle;//UART发送队列
xQueueHandle UART4_RXData_QueHandle;//UART接收队列

xQueueHandle UART6_TXData_QueHandle;//UART发送队列
xQueueHandle UART6_RXData_QueHandle;//UART接收队列

#ifdef __GNUC__  
  
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf 
 set to 'Yes') calls __io_putchar() */  
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)  
#else  
  
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)  
#endif /* __GNUC__ */  
      
PUTCHAR_PROTOTYPE  
{  
    HAL_UART_Transmit(&huart1 , (uint8_t *)&ch, 1, 0xFFFF);  
    return ch;  
}  


//=====================================================
//							需要外部实现
//
//=====================================================
//获取当前系统时间 mm
__weak float GetSystemTime()
{
	return 0;//UNUSED
}

//处理UART一帧数据
__weak void UARTAnalys(UART_HandleTypeDef *huart,char* s,int len)
{
	(void)(s);//UNUSED
}

//=====================================================
//							  内部函数
//
//====================================================

//UART1DMA发送
void UsartSendData_DMA(uint8_t *pdata, uint16_t Length)  
{  
    while(UsartType1.dmaSend_flag == USART_DMA_SENDING);  
    UsartType1.dmaSend_flag = USART_DMA_SENDING;  
    HAL_UART_Transmit_DMA(&huart1, pdata, Length);  
}
//UART2DMA发送
void Usart2SendData_DMA(uint8_t *pdata, uint16_t Length)  
{  
    while(UsartType2.dmaSend_flag == USART_DMA_SENDING);  
    UsartType2.dmaSend_flag = USART_DMA_SENDING;  
    HAL_UART_Transmit_DMA(&huart2, pdata, Length);  
}
//UART3DMA发送
void Usart3SendData_DMA(uint8_t *pdata, uint16_t Length)  
{  
    while(UsartType3.dmaSend_flag == USART_DMA_SENDING);  
    UsartType3.dmaSend_flag = USART_DMA_SENDING;  
    HAL_UART_Transmit_DMA(&huart3, pdata, Length);  
}
//UART4DMA发送
void Usart4SendData_DMA(uint8_t *pdata, uint16_t Length)  
{  
    while(UsartType4.dmaSend_flag == USART_DMA_SENDING);  
    UsartType4.dmaSend_flag = USART_DMA_SENDING;  
    HAL_UART_Transmit_DMA(&huart4, pdata, Length);  
}
//UART6DMA发送
void Usart6SendData_DMA(uint8_t *pdata, uint16_t Length)  
{  
    while(UsartType6.dmaSend_flag == USART_DMA_SENDING);  
    UsartType6.dmaSend_flag = USART_DMA_SENDING;  
    HAL_UART_Transmit_DMA(&huart6, pdata, Length);  
}
 

//DMA回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)  
{  
     __HAL_DMA_DISABLE(huart->hdmatx);
		 
		if(huart->Instance==huart1.Instance) 
			UsartType1.dmaSend_flag = USART_DMA_SENDOVER;
		if(huart->Instance==huart2.Instance) 
			UsartType2.dmaSend_flag = USART_DMA_SENDOVER;
		if(huart->Instance==huart3.Instance) 
			UsartType3.dmaSend_flag = USART_DMA_SENDOVER;
		if(huart->Instance==huart4.Instance) 
			UsartType4.dmaSend_flag = USART_DMA_SENDOVER;
		if(huart->Instance==huart6.Instance) 
			UsartType6.dmaSend_flag = USART_DMA_SENDOVER;
}  

//=========UART1

//UART发送任务
void vUARTSent_Task(void *pvParameters)
{
	char s[RECEIVELEN]={0};
	int n=0;
	float time=0;
	float UART_SendPRETime=0;
	while(1)
	{
		xQueueReceive(UART1_TXData_QueHandle,s,portMAX_DELAY);
		n=0;
		while(UsartType1.dmaSend_flag == USART_DMA_SENDING){
			n++;
			if(n>=20)
			{
				UsartType1.dmaSend_flag = USART_DMA_SENDOVER;
				break;
			}
			vTaskDelay(1/ portTICK_RATE_MS);
		}
		UsartSendData_DMA((uint8_t *)s, strlen(s));
		UsartType1.Send_n++;
		time=GetSystemTime();
		if(time-UART_SendPRETime)
			UsartType1.Send_fps=1000.0f/(time-UART_SendPRETime);
		UART_SendPRETime=time;
	}
}
//UART分析任务
void vUARTAnaly_Task(void *pvParameters)
{
	char s[Judgment_RECEIVELEN]={0};
	while(1)
	{
		xQueueReceive(UART1_RXData_QueHandle,s,portMAX_DELAY);
		//UART信息处理
		UARTAnalys(&huart1,s,strlen(s));
	}
}
//=========UART2

//UART发送任务
void vUART2Sent_Task(void *pvParameters)
{
	char s[RECEIVELEN]={0};
	int n=0;
	float time=0;
	float UART_SendPRETime=0;
	while(1)
	{
		xQueueReceive(UART2_TXData_QueHandle,s,portMAX_DELAY);
		n=0;
		while(UsartType2.dmaSend_flag == USART_DMA_SENDING){
			n++;
			if(n>=20)
			{
				UsartType2.dmaSend_flag = USART_DMA_SENDOVER;
				break;
			}
			vTaskDelay(1/ portTICK_RATE_MS);
		}
		Usart2SendData_DMA((uint8_t *)s, strlen(s));
		UsartType2.Send_n++;
		time=GetSystemTime();
		if(time-UART_SendPRETime)
			UsartType2.Send_fps=1000.0f/(time-UART_SendPRETime);
		UART_SendPRETime=time;
	}
}
//UART分析任务
void vUART2Analy_Task(void *pvParameters)
{
	char s[RECEIVELEN]={0};
	while(1)
	{
		xQueueReceive(UART2_RXData_QueHandle,s,portMAX_DELAY);
		//UART信息处理
		UARTAnalys(&huart2,s,strlen(s));
	}
}
//=========UART3

//UART发送任务
void vUART3Sent_Task(void *pvParameters)
{
	char s[RECEIVELEN]={0};
	int n=0;
	float time=0;
	float UART_SendPRETime=0;
	while(1)
	{
		xQueueReceive(UART3_TXData_QueHandle,s,portMAX_DELAY);
		n=0;
		while(UsartType3.dmaSend_flag == USART_DMA_SENDING){
			n++;
			if(n>=20)
			{
				UsartType3.dmaSend_flag = USART_DMA_SENDOVER;
				break;
			}
			vTaskDelay(1/ portTICK_RATE_MS);
		}
		Usart3SendData_DMA((uint8_t *)s, strlen(s));
		UsartType3.Send_n++;
		time=GetSystemTime();
		if(time-UART_SendPRETime)
			UsartType3.Send_fps=1000.0f/(time-UART_SendPRETime);
		UART_SendPRETime=time;
	}
}
//UART分析任务
void vUART3Analy_Task(void *pvParameters)
{
	char s[RECEIVELEN]={0};
	while(1)
	{
		xQueueReceive(UART3_RXData_QueHandle,s,portMAX_DELAY);
		//UART信息处理
		UARTAnalys(&huart3,s,strlen(s));
	}
}
//=========UART4
//UART4发送任务
void vUART4Sent_Task(void *pvParameters)
{
	char s[RECEIVELEN]={0};
	int n=0;
	float time=0;
	float UART_SendPRETime=0;
	while(1)
	{
		xQueueReceive(UART4_TXData_QueHandle,s,portMAX_DELAY);
		n=0;
		while(UsartType4.dmaSend_flag == USART_DMA_SENDING){
			n++;
			if(n>=20)
			{
				UsartType4.dmaSend_flag = USART_DMA_SENDOVER;
				break;
			}
			vTaskDelay(1/ portTICK_RATE_MS);
		}
		Usart4SendData_DMA((uint8_t *)s, strlen(s));
		UsartType4.Send_n++;
		time=GetSystemTime();
		if(time-UART_SendPRETime)
			UsartType4.Send_fps=1000.0f/(time-UART_SendPRETime);
		UART_SendPRETime=time;
	}
}
//UART分析任务
void vUART4Analy_Task(void *pvParameters)
{
	char s[RECEIVELEN]={0};
	while(1)
	{
		xQueueReceive(UART4_RXData_QueHandle,s,portMAX_DELAY);
		//UART信息处理
		UARTAnalys(&huart4,s,strlen(s));
	}
}

//=========UART6
//UART6发送任务
void vUART6Sent_Task(void *pvParameters)
{
	char s[RECEIVELEN]={0};
	int n=0;
	float time=0;
	float UART_SendPRETime=0;
	while(1)
	{
		xQueueReceive(UART6_TXData_QueHandle,s,portMAX_DELAY);
		n=0;
		while(UsartType6.dmaSend_flag == USART_DMA_SENDING){
			n++;
			if(n>=20)
			{
				UsartType6.dmaSend_flag = USART_DMA_SENDOVER;
				break;
			}
			vTaskDelay(1/ portTICK_RATE_MS);
		}
		Usart6SendData_DMA((uint8_t *)s, strlen(s));
		UsartType6.Send_n++;
		time=GetSystemTime();
		if(time-UART_SendPRETime)
			UsartType6.Send_fps=1000.0f/(time-UART_SendPRETime);
		UART_SendPRETime=time;
	}
}
//UART6分析任务
void vUART6Analy_Task(void *pvParameters)
{
	char s[RECEIVELEN]={0};
	while(1)
	{
		xQueueReceive(UART6_RXData_QueHandle,s,portMAX_DELAY);
		//UART信息处理
		UARTAnalys(&huart6,s,strlen(s));
	}
}

//=====================================================
//							  外部调用
//
//=====================================================

//UART初始化并启动任务 成功返回1 失败返回0
int UART_Inite()
{
	HAL_UART_Receive_DMA(&huart1, UsartType1.usartDMA_rxBuf, Judgment_RECEIVELEN); //启动DMA接收
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);//使能串口中断
	
	HAL_UART_Receive_DMA(&huart2, UsartType2.usartDMA_rxBuf, RECEIVELEN); //启动DMA接收
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);//使能串口中断
	
	HAL_UART_Receive_DMA(&huart3, UsartType3.usartDMA_rxBuf, RECEIVELEN); //启动DMA接收
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);//使能串口中断
	
	HAL_UART_Receive_DMA(&huart4, UsartType4.usartDMA_rxBuf, RECEIVELEN); //启动DMA接收
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);//使能串口中断
	
	//HAL_UART_Receive_DMA(&huart6, UsartType6.usartDMA_rxBuf, RECEIVELEN); //启动DMA接收
  //__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);//使能串口中断
	
	UART1_TXData_QueHandle=xQueueCreate(5,Judgment_RECEIVELEN);//信息队列初始化 深度10 长度RECEIVELEN字节
	UART1_RXData_QueHandle=xQueueCreate(5,Judgment_RECEIVELEN);
	UART2_TXData_QueHandle=xQueueCreate(5,RECEIVELEN);//信息队列初始化 深度10 长度RECEIVELEN字节
	UART2_RXData_QueHandle=xQueueCreate(5,RECEIVELEN);
	UART3_TXData_QueHandle=xQueueCreate(5,RECEIVELEN);//信息队列初始化 深度10 长度RECEIVELEN字节
	UART3_RXData_QueHandle=xQueueCreate(5,RECEIVELEN);
	UART4_TXData_QueHandle=xQueueCreate(5,RECEIVELEN);//信息队列初始化 深度10 长度RECEIVELEN字节
	UART4_RXData_QueHandle=xQueueCreate(5,RECEIVELEN);
	//UART6_TXData_QueHandle=xQueueCreate(5,RECEIVELEN);//信息队列初始化 深度10 长度RECEIVELEN字节
	//UART6_RXData_QueHandle=xQueueCreate(5,RECEIVELEN);
	if(NULL==UART1_TXData_QueHandle && NULL==UART1_RXData_QueHandle)return 0;
	if(NULL==UART2_TXData_QueHandle && NULL==UART2_RXData_QueHandle)return 0;
	if(NULL==UART3_TXData_QueHandle && NULL==UART3_RXData_QueHandle)return 0;
	if(NULL==UART4_TXData_QueHandle && NULL==UART4_RXData_QueHandle)return 0;
	//if(NULL==UART6_TXData_QueHandle && NULL==UART6_RXData_QueHandle)return 0;
	
	//启动UART发送任务
	if(!xTaskCreate( vUARTSent_Task,  "UART1Sent_Task" , 200, NULL, UART_SentTask_Priority , NULL ))return 0;
	//启动UART分析任务
	if(!xTaskCreate( vUARTAnaly_Task, "UART1Analy_Task", 400, NULL, UART_AnalyTask_Priority, NULL ))return 0;
	
	//启动UART发送任务
	if(!xTaskCreate( vUART2Sent_Task,  "UART2Sent_Task" , 200, NULL, UART_SentTask_Priority , NULL ))return 0;
	//启动UART分析任务
	if(!xTaskCreate( vUART2Analy_Task, "UART2Analy_Task", 200, NULL, UART_AnalyTask_Priority, NULL ))return 0;
	
	//启动UART发送任务
	if(!xTaskCreate( vUART3Sent_Task,  "UART3Sent_Task" , 200, NULL, UART_SentTask_Priority , NULL ))return 0;
	//启动UART分析任务
	if(!xTaskCreate( vUART3Analy_Task, "UART3Analy_Task", 200, NULL, UART_AnalyTask_Priority, NULL ))return 0;
	
	//启动UART发送任务
	if(!xTaskCreate( vUART4Sent_Task,  "UART4Sent_Task" , 200, NULL, UART_SentTask_Priority , NULL ))return 0;
	//启动UART分析任务
	if(!xTaskCreate( vUART4Analy_Task, "UART4Analy_Task", 200, NULL, UART_AnalyTask_Priority, NULL ))return 0;
	
	//启动UART发送任务
	//if(!xTaskCreate( vUART6Sent_Task,  "UART6Sent_Task" , 200, NULL, UART_SentTask_Priority , NULL ))return 0;
	//启动UART分析任务
	//if(!xTaskCreate( vUART6Analy_Task, "UART6Analy_Task", 200, NULL, UART_AnalyTask_Priority, NULL ))return 0;
	
	printf("UART Inite OK!\r\n");
	return 1;
}


//UART空闲中断
void UsartReceive_IDLE(UART_HandleTypeDef *huart)  
{  
   uint32_t temp; 
	 //==============UART1
   if(huart->Instance==huart1.Instance)
   {
     if((__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET))  
     {   
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);  
        HAL_UART_DMAStop(&huart1);  
        temp = huart1.hdmarx->Instance->NDTR;  
        UsartType1.rx_len =  Judgment_RECEIVELEN - temp; 
				//judgement_data_handler((unsigned char*)UsartType1.usartDMA_rxBuf);//裁判系统解算
				xQueueSendToBackFromISR(UART1_RXData_QueHandle,UsartType1.usartDMA_rxBuf,0);
			  memset(UsartType1.usartDMA_rxBuf,0,Judgment_RECEIVELEN);
        HAL_UART_Receive_DMA(&huart1,UsartType1.usartDMA_rxBuf,Judgment_RECEIVELEN);  
     }
   }
	 //==============UART2
   if(huart->Instance==huart2.Instance)
   {
     if((__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET))  
     {   
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);  
        HAL_UART_DMAStop(&huart2);  
        temp = huart2.hdmarx->Instance->NDTR;  
        UsartType2.rx_len =  RECEIVELEN - temp;   
				GetRemoteCtrMsg(&RC_Data, UsartType2.usartDMA_rxBuf);
				//xQueueSendToBackFromISR(UART2_RXData_QueHandle,UsartType2.usartDMA_rxBuf,0);
			  memset(UsartType2.usartDMA_rxBuf,0,RECEIVELEN);
        HAL_UART_Receive_DMA(&huart2,UsartType2.usartDMA_rxBuf,RECEIVELEN);  
     }
   }
	  //==============UART3
   if(huart->Instance==huart3.Instance)
   {
     if((__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET))  
     {   
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);  
        HAL_UART_DMAStop(&huart3);  
        temp = huart3.hdmarx->Instance->NDTR;  
        UsartType3.rx_len =  RECEIVELEN - temp;   
				GetRemoteCtrMsg(&RC_Data, UsartType3.usartDMA_rxBuf);
				xQueueSendToBackFromISR(UART3_RXData_QueHandle,UsartType3.usartDMA_rxBuf,0);
			  memset(UsartType3.usartDMA_rxBuf,0,RECEIVELEN);
        HAL_UART_Receive_DMA(&huart3,UsartType3.usartDMA_rxBuf,RECEIVELEN);  
     }
   }
	 //==============UART4
   if(huart->Instance==huart4.Instance)
   {
     if((__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET))  
     {   
        __HAL_UART_CLEAR_IDLEFLAG(&huart4);  
        HAL_UART_DMAStop(&huart4);  
        temp = huart4.hdmarx->Instance->NDTR;  
        UsartType4.rx_len =  RECEIVELEN - temp;   
				xQueueSendToBackFromISR(UART4_RXData_QueHandle,UsartType4.usartDMA_rxBuf,0);
			  memset(UsartType4.usartDMA_rxBuf,0,RECEIVELEN);
        HAL_UART_Receive_DMA(&huart4,UsartType4.usartDMA_rxBuf,RECEIVELEN);  
     }
   }
	 //==============UART6
   if(huart->Instance==huart6.Instance)
   {
     if((__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET))  
     {   
        __HAL_UART_CLEAR_IDLEFLAG(&huart6);  
        HAL_UART_DMAStop(&huart6);  
        temp = huart6.hdmarx->Instance->NDTR;  
        UsartType6.rx_len =  RECEIVELEN - temp;   
				xQueueSendToBackFromISR(UART6_RXData_QueHandle,UsartType6.usartDMA_rxBuf,0);
			  memset(UsartType6.usartDMA_rxBuf,0,RECEIVELEN);
        HAL_UART_Receive_DMA(&huart6,UsartType6.usartDMA_rxBuf,RECEIVELEN);  
     }
   }
} 


//将信息放入发送队列
int My_printFromISR(char* s)
{
	return xQueueSendToBackFromISR(UART3_TXData_QueHandle,s,0);
}
int My_print(char* s)
{
	return xQueueSendToBack(UART3_TXData_QueHandle,s,0);
}

int UART_SendToQueue(UART_HandleTypeDef *huart,char* data)
{
	if(huart->Instance==huart2.Instance)
		return xQueueSendToBack(UART2_TXData_QueHandle,data,0);
	if(huart->Instance==huart3.Instance)
		return xQueueSendToBack(UART3_TXData_QueHandle,data,0);
	if(huart->Instance==huart4.Instance)
		return xQueueSendToBack(UART4_TXData_QueHandle,data,0);
	if(huart->Instance==huart6.Instance)
		return xQueueSendToBack(UART6_TXData_QueHandle,data,0);
	return 0;
}
int UART_SendToQueueFromISR(UART_HandleTypeDef *huart,char* data)
{
	if(huart->Instance==huart2.Instance)
		return xQueueSendToBackFromISR(UART2_TXData_QueHandle,data,0);
	if(huart->Instance==huart3.Instance)
		return xQueueSendToBackFromISR(UART3_TXData_QueHandle,data,0);
	if(huart->Instance==huart4.Instance)
		return xQueueSendToBackFromISR(UART4_TXData_QueHandle,data,0);
	if(huart->Instance==huart6.Instance)
		return xQueueSendToBackFromISR(UART6_TXData_QueHandle,data,0);
	return 0;
}


//===============================================
//
//			其他需要模块需要UATRT提供的函数
//
//================================================
//WIFI 发送控制
void Wifi_UARTSent(unsigned char*s,short len)
{
	UART_SendToQueue(&huart4,(char*)s);
}


