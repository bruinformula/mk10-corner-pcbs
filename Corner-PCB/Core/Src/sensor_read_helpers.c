/*
 * sensor_read_helpers.c
 *
 *  Created on: Apr 7, 2025
 *      Author: antho
 */

#include "sensor_read_helpers.h"
#include <stdio.h>


void readLinearPotentiometer(ADC_HandleTypeDef *hadc, uint32_t *lastReadMS, MISC_DATAFRAME *dataframe) {
	if (HAL_GetTick() - *lastReadMS > SHOCK_TRAVEL_SAMPLE_PERIOD) {

		HAL_ADC_Start(hadc);
		if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK) {
			uint32_t adc_val = HAL_ADC_GetValue(hadc);
			HAL_ADC_Stop(hadc);

			uint16_t scaled_travel = (uint16_t)((adc_val * 65535UL) / 4095);
			dataframe->data.shockTravel = scaled_travel;

			*lastReadMS = HAL_GetTick();
		} else {
			HAL_ADC_Stop(hadc);
		}
	}
}

void readBrakeTemp(uint32_t *lastReadMS, MISC_DATAFRAME *dataframe, UART_HandleTypeDef *uartPort) {
	uint8_t txData[8];

	if(HAL_GetTick() - *lastReadMS > BRAKE_TEMP_SAMPLE_PERIOD){
		//send data
//		for (int i = 1; i < 200; i++) {
			txData[0] = 1;
			txData[1] = 0x03;
			txData[2] = 0;
			txData[3] = 0x02;
			txData[4] = 0;
			txData[5] = 0x01;
			uint16_t crc = computeCRC16(txData, 6);
			txData[6] = crc & 0xFF;
			txData[7] = (crc >> 8) & 0xFF;



			dataframe->data.brakeTemp = sendBrakeTempData(txData, uartPort);
			//todo: actual brake temp sensor read code
			*lastReadMS = HAL_GetTick();
//		}
	}

	//todo: convert to deg C
}

void readTireTemp(uint32_t *lastReadMS, TTEMP_DATAFRAME *dataframes) {

	if(HAL_GetTick() - *lastReadMS > TIRE_TEMP_SAMPLE_PERIOD){
		for(int i = 0; i < 4; i++) {
			dataframes[i].data.pix0 = 1 + (4*i);
			dataframes[i].data.pix1 = 2 + (4*i);
			dataframes[i].data.pix2 = 3 + (4*i);
			dataframes[i].data.pix3 = 4 + (4*i);
			dataframes[i].data.pix4 = 5 + (4*i);
			dataframes[i].data.pix5 = 6 + (4*i);
			dataframes[i].data.pix6 = 7 + (4*i);
			dataframes[i].data.pix7 = 8 + (4*i);
		}


		//todo: actual tire temp sensor read code
		*lastReadMS = HAL_GetTick();
	}
}
void readStrainGauges(SPI_HandleTypeDef *hspi, uint32_t *lastReadMS, SG_DATAFRAME *dataframe) {

	if(HAL_GetTick() - *lastReadMS > STRAIN_GAUGE_SAMPLE_PERIOD){
		dataframe->data.SG0 = 0;
		dataframe->data.SG1 = 0;
		dataframe->data.SG2 = 0;
		dataframe->data.SG3 = 0;

		//todo: actual strain gauge sensor read code
		//todo: convert counts to newtons
		*lastReadMS = HAL_GetTick();
	}
}

void readWheelSpeed(uint32_t *lastReadMS, MISC_DATAFRAME *dataframe) {
    static uint32_t edgeCount = 0;
    static uint32_t sampleStartTime = 0;
    static uint8_t prevLevel = 0;
    const uint32_t sampleWindowMS = 200;
    const uint32_t edgesPerRev = 24;

    uint32_t now = HAL_GetTick();

    if ((now - sampleStartTime) >= sampleWindowMS) {
        float rotations = (float)edgeCount / (float)edgesPerRev;
        float rpm = rotations * (60000.0f / sampleWindowMS);

        dataframe->data.wheelRPM = (uint16_t)rpm;

        edgeCount = 0;
        sampleStartTime = now;
    }

    uint8_t currentLevel = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
    if (currentLevel != prevLevel) {
        edgeCount++;
        prevLevel = currentLevel;
    }

    *lastReadMS = now;
}




void readBoardTemp(SPI_HandleTypeDef *hspi, uint32_t *lastReadMS, MISC_DATAFRAME *dataframe) {

	if(HAL_GetTick() - *lastReadMS > STRAIN_GAUGE_SAMPLE_PERIOD){
		dataframe->data.boardTemp = 0;


		//todo: use ads1118, same chip as the shits, to read board temp
		//todo: convert counts to deg.C
		*lastReadMS = HAL_GetTick();
	}
}
