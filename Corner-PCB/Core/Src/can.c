/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 10;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN1_MspInit 1 */

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

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
HAL_StatusTypeDef CANTransmitMinion(CAN_HandleTypeDef *canport, CAN_TxHeaderTypeDef *header, uint8_t *dataArray) {
	HAL_StatusTypeDef TXStatusOut = HAL_ERROR;
	//	printf("sending ID ");
	//	printf((uint32_t)(header->StdId));
	int i = 0;
	uint32_t mailbox = 0;
	while (i < CAN_RETRY_LIMIT && TXStatusOut != HAL_OK) {
		while(HAL_CAN_GetTxMailboxesFreeLevel(canport) < 1) {
					//wait until a new mailbox gets freed up
		//			printf("\n\rwaiting\n\r");
				}
		TXStatusOut = HAL_CAN_AddTxMessage(canport, header, dataArray, &mailbox);

		i++;
	}

	if (TXStatusOut != HAL_OK) {
		mailbox = 0;

	}
	//	printf("\n\r");
	return TXStatusOut;
}

void clearEflagsHelper(CORNER_CAN_CONTEXT *CANCONTEXT) {
	CANCONTEXT->misc_dataframe.data.eflags.ADCErrorBit = 0;
	CANCONTEXT->misc_dataframe.data.eflags.BrakeTempErrorBit = 0;
	CANCONTEXT->misc_dataframe.data.eflags.SGMsgErrorBit = 0;
	CANCONTEXT->misc_dataframe.data.eflags.MiscMsgErrorBit = 0;
	CANCONTEXT->misc_dataframe.data.eflags.TTempMsg1ErrorBit = 0;
	CANCONTEXT->misc_dataframe.data.eflags.TTempMsg2ErrorBit = 0;
	CANCONTEXT->misc_dataframe.data.eflags.TTempMsg3ErrorBit = 0;
	CANCONTEXT->misc_dataframe.data.eflags.TTempMsg4ErrorBit = 0;

}

void CANMailman(CAN_HandleTypeDef *canport, CAN_TxHeaderTypeDef *header, CORNER_CAN_CONTEXT *CANCONTEXT) {
	clearEflagsHelper(CANCONTEXT);
	HAL_StatusTypeDef txstatus;

	/*** BEGIN SEND MISC MESSAGE (btemp, whs, board temp, error flags, shock travel) */
	if (HAL_GetTick() - CANCONTEXT->ms_since_miscmsg_broadcast > MISC_DATA_TRANSMISSION_PERIOD) {
		header->StdId = MISC_DATA_ID;
		txstatus = CANTransmitMinion(canport, header, CANCONTEXT->misc_dataframe.array);

		//set error flag
		if (txstatus != HAL_OK) {
			CANCONTEXT->misc_dataframe.data.eflags.MiscMsgErrorBit = true;
		} else {
			CANCONTEXT->misc_dataframe.data.eflags.MiscMsgErrorBit = false;
		}

		CANCONTEXT->ms_since_miscmsg_broadcast = HAL_GetTick();
	}

	/**** BEGIN SEND STRAIN GAUGE DATA ****/
	if (HAL_GetTick() - CANCONTEXT->ms_since_strain_broadcast > STRAIN_GAUGE_TRANSMISSION_PERIOD) {
		header->StdId = STRAIN_GAUGE_ID;
		txstatus = CANTransmitMinion (canport, header, CANCONTEXT->straingauge_dataframe.array);

		//set error flag
		if (txstatus != HAL_OK) {
			CANCONTEXT->misc_dataframe.data.eflags.SGMsgErrorBit = true;
		} else {
			CANCONTEXT->misc_dataframe.data.eflags.SGMsgErrorBit = false;
		}

		CANCONTEXT->ms_since_strain_broadcast = HAL_GetTick();

	}

	/**** BEGIN SEND TIRE TEMP DATA ****/
	if (!TTEMP_DISABLED && HAL_GetTick() - CANCONTEXT->ms_since_ttemp_broadcast > TIRE_TEMP_TRANSMISSION_PERIOD) {

		//		CTXHeader.IDE = CAN_ID_STD;
		//		CTXHeader.RTR = CAN_RTR_DATA;
		//		CTXHeader.DLC = 8;
		//		ms_since_ttemp_broadcast = HAL_GetTick();

		header->StdId = TIRE_TEMP_MSG1_ID;
		txstatus = CANTransmitMinion (canport, header, CANCONTEXT->ttemp_dataframes[0].array);
		if (txstatus != HAL_OK) {
			CANCONTEXT->misc_dataframe.data.eflags.TTempMsg1ErrorBit = true;
		} else {
			CANCONTEXT->misc_dataframe.data.eflags.TTempMsg1ErrorBit = false;
		}

		header->StdId = TIRE_TEMP_MSG2_ID;
		txstatus = CANTransmitMinion (canport, header, CANCONTEXT->ttemp_dataframes[1].array);
		if (txstatus != HAL_OK) {
			CANCONTEXT->misc_dataframe.data.eflags.TTempMsg2ErrorBit = true;
		} else {
			CANCONTEXT->misc_dataframe.data.eflags.TTempMsg2ErrorBit = false;
		}

		header->StdId = TIRE_TEMP_MSG3_ID;
		txstatus = CANTransmitMinion (canport, header, CANCONTEXT->ttemp_dataframes[2].array);
		if (txstatus != HAL_OK) {
			CANCONTEXT->misc_dataframe.data.eflags.TTempMsg3ErrorBit = true;
		} else {
			CANCONTEXT->misc_dataframe.data.eflags.TTempMsg3ErrorBit = false;
		}

		header->StdId = TIRE_TEMP_MSG4_ID;
		txstatus = CANTransmitMinion (canport, header, CANCONTEXT->ttemp_dataframes[3].array);
		if (txstatus != HAL_OK) {
			CANCONTEXT->misc_dataframe.data.eflags.TTempMsg4ErrorBit = true;
		} else {
			CANCONTEXT->misc_dataframe.data.eflags.TTempMsg4ErrorBit = false;
		}

		CANCONTEXT->ms_since_ttemp_broadcast = HAL_GetTick();
	}

	/**** END SEND TIRE TEMP DATA ****/
}
/* USER CODE END 1 */
