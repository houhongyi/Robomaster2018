/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

#include "gpio.h"
#include "iwdg.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_5TQ;
  hcan.Init.BS2 = CAN_BS2_3TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = ENABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    /**CAN GPIO Configuration    
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_CAN1_2();

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */
		static CanTxMsgTypeDef txMsg;
    static CanRxMsgTypeDef rxMsg;
    hcan.pTxMsg = &txMsg;
    hcan.pRxMsg = &rxMsg;
  //can过滤器配置
	CAN_FilterConfTypeDef  can_filter;
	can_filter.FilterNumber = 0; 
	can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
	//can_filter.FilterIdHigh = 0;
	can_filter.FilterIdHigh= (((uint32_t)(BASE_MOTOR_ID)<<21)&0xFFFF0000)>>16;
	can_filter.FilterIdLow = 0;
	//can_filter.FilterMaskIdHigh = 0;
	can_filter.FilterMaskIdHigh = 0xFFE0;//1111 1111 1110 0000
	can_filter.FilterMaskIdLow = 0;
	can_filter.FilterFIFOAssignment = 0;
	can_filter.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan,&can_filter);
	
  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN GPIO Configuration    
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);

  }
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
} 

/* USER CODE BEGIN 1 */
/**
  * @brief  CAN中断的回调函数
  * @param  CAN_HandleTypeDef* _hcan
  * @retval None
  */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
{
//		//用CAN喂狗
 	  HAL_IWDG_Refresh(&hiwdg);

	  if(_hcan->pRxMsg->StdId==BASE_MOTOR_ID)
		{
    //赋值而已
		Control_Command.Relay_Command.Relay1_Data=_hcan->pRxMsg->Data[0];
		Control_Command.Relay_Command.Relay2_Data=_hcan->pRxMsg->Data[1];
		Control_Command.Relay_Command.Relay3_Data=_hcan->pRxMsg->Data[2];
		Control_Command.Relay_Command.Relay4_Data=_hcan->pRxMsg->Data[3];
		Control_Command.Relay_Command.Relay5_Data=_hcan->pRxMsg->Data[4];
	  //提示而已
		HAL_GPIO_TogglePin(Led1_GPIO_Port, Led1_Pin);	
		}
		else
		{
			HAL_GPIO_TogglePin(Led1_GPIO_Port, Led1_Pin);	
		}
	
		__HAL_CAN_ENABLE_IT(_hcan, CAN_IT_FMP0);
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
