/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dataframes.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
#define CORNER_NUMBER 1
#define STRAIN_GAUGE_ID 	0b11100000000 | (CORNER_NUMBER << 4)
#define TIRE_TEMP_MSG1_ID 	0b11100000001 | (CORNER_NUMBER << 4)
#define TIRE_TEMP_MSG2_ID 	0b11100000010 | (CORNER_NUMBER << 4)
#define TIRE_TEMP_MSG3_ID 	0b11100000011 | (CORNER_NUMBER << 4)
#define TIRE_TEMP_MSG4_ID 	0b11100000100 | (CORNER_NUMBER << 4)
#define MISC_DATA_ID 		0b11100000101 | (CORNER_NUMBER << 4)

/* how many times itll retry putting any CAN message in a mailbox before j giving up*/
#define CAN_RETRY_LIMIT 3

/* start transmission rates (ms) */
#define STRAIN_GAUGE_TRANSMISSION_PERIOD 	3
#define TIRE_TEMP_TRANSMISSION_PERIOD 		501
#define MISC_DATA_TRANSMISSION_PERIOD 		21
/* end transmission rates */


/* start sampling rates (ms) */
#define STRAIN_GAUGE_SAMPLE_PERIOD 		3
#define TIRE_TEMP_SAMPLE_PERIOD 		501
#define WHEEL_SPEED_SAMPLE_PERIOD 		11 //this is how long it outputs an actual speed
#define BRAKE_TEMP_SAMPLE_PERIOD		101
#define SHOCK_TRAVEL_SAMPLE_PERIOD		21
#define BOARD_TEMP_SAMPLE_PERIOD		1001
/* end sampling rates */


//these exist to effectively put more decimal points in a byte datafield
/* start scaling factors */
#define STRAIN_GAUGE_SF 	10
#define TIRE_TEMP_SF 		4
#define WHEEL_SPEED_SF 		1 //this is how long it outputs an actual speed
#define BRAKE_TEMP_SF		10
#define SHOCK_TRAVEL_SF		1000
#define BOARD_TEMP_SF		1000
/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
void CANMailman(CAN_HandleTypeDef *canport, CAN_TxHeaderTypeDef *header, CORNER_CAN_CONTEXT *CANCONTEXT);
HAL_StatusTypeDef CANTransmitMinion (CAN_HandleTypeDef *canport, CAN_TxHeaderTypeDef *header, uint8_t *dataArray);
void clearEflagsHelper(CORNER_CAN_CONTEXT *CANCONTEXT);
void CANMailman(CAN_HandleTypeDef *canport, CAN_TxHeaderTypeDef *header, CORNER_CAN_CONTEXT *CANCONTEXT);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

