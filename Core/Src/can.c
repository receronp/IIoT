/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               TxData[8];
uint8_t               RxData[8];

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;
CAN_FilterTypeDef sFilterConfig;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_LOOPBACK;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_6TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN)
  {
  /* USER CODE BEGIN CAN_MspInit 0 */

  /* USER CODE END CAN_MspInit 0 */
    /* CAN clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN_MspInit 1 */

  /* USER CODE END CAN_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN)
  {
  /* USER CODE BEGIN CAN_MspDeInit 0 */

  /* USER CODE END CAN_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

  /* USER CODE BEGIN CAN_MspDeInit 1 */

  /* USER CODE END CAN_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

HAL_StatusTypeDef CAN_Configurar_Filtrado(){
	  sFilterConfig.FilterBank = 0;
	  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	  sFilterConfig.FilterIdHigh = 0x0000;
	  sFilterConfig.FilterIdLow = 0x0000;
	  sFilterConfig.FilterMaskIdHigh = 0x0000;
	  sFilterConfig.FilterMaskIdLow = 0x0000;
	  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	  sFilterConfig.FilterActivation = ENABLE;
	  sFilterConfig.SlaveStartFilterBank = 14;

	  if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  return HAL_OK;
}

HAL_StatusTypeDef CAN_TX(uint32_t id, uint32_t ide , uint32_t rtr, uint32_t dlc, uint32_t *data)
 {
 	CAN_TxHeaderTypeDef TxHeader;
 	uint32_t TxMailbox;
 	HAL_StatusTypeDef retorno;

 	if(ide == CAN_ID_STD){
 		 TxHeader.StdId = id;
 		 TxHeader.ExtId = 0;
 	} else {
 		 TxHeader.ExtId = id;
 		 TxHeader.StdId = 0;
 	}

 	 TxHeader.RTR = rtr;
 	 TxHeader.IDE = ide;
 	 TxHeader.DLC = dlc;
 	 TxHeader.TransmitGlobalTime = DISABLE;

  	 uint8_t TxData[8] = {0};
  	 for(int i = 0; i < dlc && i < 8;i++){
  		 TxData[i] = data[i];
  	 }

 	 retorno = HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);

 	 if(retorno != HAL_OK){
 		 Error_Handler();
 	 }

 	 while(HAL_CAN_IsTxMessagePending(&hcan, TxMailbox));
 	 return HAL_OK;
 }

 uint32_t CAN_RX()
 {
 	uint32_t NumMensajes = 0;
 	uint32_t i = 0;

 	NumMensajes = HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);

 	while(NumMensajes > 0){
 		if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData)== HAL_OK){
 		    char hexId[50];
 		    char hexString[100];
 		    sprintf(hexId, "RxHeader.ExtId = 0x%lx\r\n", RxHeader.ExtId);
 		    HAL_UART_Transmit(&huart3, (uint8_t*)hexId, 50, HAL_MAX_DELAY);
 		    sprintf(hexString, "Message: %d %d %d\r\n", RxData[0], RxData[1], RxData[2]); // Convert to hex string
 		    HAL_UART_Transmit(&huart3, (uint8_t*)hexString, 100, HAL_MAX_DELAY); // Send hex string
 		}
	NumMensajes = HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);
 	}

 	return (i);
 }

/* USER CODE END 1 */
