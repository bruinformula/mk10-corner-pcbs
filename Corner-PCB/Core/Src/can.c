/*
 * can.c
 *
 *  Created on: Apr 9, 2025
 *      Author: antho
 */

#include "can.h"
#include "dataframes.h"



HAL_StatusTypeDef CANTransmitMinion (CAN_HandleTypeDef *canport, CAN_TxHeaderTypeDef *header, uint8_t *dataArray) {

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
		}

		CANCONTEXT->ms_since_miscmsg_broadcast = HAL_GetTick();
	}
	/*** BEGIN SEND MISC MESSAGE (btemp, whs, board temp, error flags, shock travel) */




	/**** BEGIN SEND STRAIN GAUGE DATA ****/
	if (HAL_GetTick() - CANCONTEXT->ms_since_strain_broadcast > STRAIN_GAUGE_TRANSMISSION_PERIOD) {
		header->StdId = STRAIN_GAUGE_ID;
		txstatus = CANTransmitMinion (canport, header, CANCONTEXT->straingauge_dataframe.array);

		//set error flag
		if (txstatus != HAL_OK) {
			CANCONTEXT->misc_dataframe.data.eflags.SGMsgErrorBit = true;
		}

		CANCONTEXT->ms_since_strain_broadcast = HAL_GetTick();

	}
	/**** END SEND STRAIN GAUGE DATA ****/



	/**** BEGIN SEND TIRE TEMP DATA ****/
	if (HAL_GetTick() - CANCONTEXT->ms_since_ttemp_broadcast > TIRE_TEMP_TRANSMISSION_PERIOD) {

		//		CTXHeader.IDE = CAN_ID_STD;
		//		CTXHeader.RTR = CAN_RTR_DATA;
		//		CTXHeader.DLC = 8;
		//		ms_since_ttemp_broadcast = HAL_GetTick();

		header->StdId = TIRE_TEMP_MSG1_ID;
		txstatus = CANTransmitMinion (canport, header, CANCONTEXT->ttemp_dataframes[0].array);
		if (txstatus != HAL_OK) {
			CANCONTEXT->misc_dataframe.data.eflags.TTempMsg1ErrorBit = true;
		}

		header->StdId = TIRE_TEMP_MSG2_ID;
		txstatus = CANTransmitMinion (canport, header, CANCONTEXT->ttemp_dataframes[1].array);
		if (txstatus != HAL_OK) {
			CANCONTEXT->misc_dataframe.data.eflags.TTempMsg2ErrorBit = true;
		}

		header->StdId = TIRE_TEMP_MSG3_ID;
		txstatus = CANTransmitMinion (canport, header, CANCONTEXT->ttemp_dataframes[2].array);
		if (txstatus != HAL_OK) {
			CANCONTEXT->misc_dataframe.data.eflags.TTempMsg3ErrorBit = true;
		}

		header->StdId = TIRE_TEMP_MSG4_ID;
		txstatus = CANTransmitMinion (canport, header, CANCONTEXT->ttemp_dataframes[3].array);
		if (txstatus != HAL_OK) {
			CANCONTEXT->misc_dataframe.data.eflags.TTempMsg4ErrorBit = true;
		}

		CANCONTEXT->ms_since_ttemp_broadcast = HAL_GetTick();
	}

	/**** END SEND TIRE TEMP DATA ****/
}

/*
uint32_t lastCanSpam = 0;
//uint32_t mailbox = 0;
uint8_t penis[] = {0x66, 0x75, 0x63, 0x6B, 0x66, 0x61, 0x63, 0x65};
//CAN_TxHeaderTypeDef CTXHeader;
void spamCan(void) {
	if(HAL_GetTick() - lastCanSpam >= 100 ){ //scuffed but itll work for testing
		CTXHeader.StdId = 0x600;
		//		CTXHeader.ExtId = 0x1AAAAAA;
		CTXHeader.IDE = CAN_ID_STD;
		CTXHeader.RTR = CAN_RTR_DATA;
		CTXHeader.DLC = 8;
		//		CTXHeader.TransmitGlobalTime = DISABLE;

		//printf("SPAM CAN\n\r");
		//
		uint32_t mailbox = 0;
		HAL_StatusTypeDef status = HAL_TIMEOUT;
		int i = 0;
		while (i < CAN_RETRY_LIMIT && status != HAL_OK) {
			status = HAL_CAN_AddTxMessage(&hcan1, &CTXHeader, penis, &mailbox);

			i ++;
		}

		lastCanSpam = HAL_GetTick();
	}
}
 */
