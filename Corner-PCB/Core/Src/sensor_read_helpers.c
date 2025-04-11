/*
 * sensor_read_helpers.c
 *
 *  Created on: Apr 7, 2025
 *      Author: antho
 */

#include "sensor_read_helpers.h"
#include "stdbool.h"
#include "can.h"
#include "stdio.h"

uint16_t MLX_eeData[832] = {0};
paramsMLX90640 MLX_params;
uint16_t MLX_dataFrame[834] = {0};
float MLX_to[768] = {0};
uint8_t MLX_sample[32] = {0};

bool initializeMLX() {
	unsigned int status;
	MLX90640_SetRefreshRate(MLX_ADDR, REFRESH_RATE);
	MLX90640_SetChessMode(MLX_ADDR);
	status = MLX90640_DumpEE(MLX_ADDR, MLX_eeData);
	printf("\r\nLOADING EEPROM PARAMETERS:%d\r\n", status);
	status = MLX90640_ExtractParameters(MLX_eeData, &MLX_params);
	printf("\r\nEXTRACTING PARAMETERS:%d\r\n", status);

	if (status == -1) return 0;
	return 1;
}

void computeMLXSample() {
	int startIndex = 384;
	for (int k = 0; k < 32; k++) {
		MLX_sample[k] = (uint8_t)(255.0f * (MLX_to[startIndex+k] + 40.0f) / 340.0f);
	}
}

void readLinearPotentiometer(ADC_HandleTypeDef *hadc, uint32_t *lastReadMS,  MISC_DATAFRAME *dataframe) {
	uint32_t ADC_Read[1];
	uint32_t ADC_BUFFER = 1;

	HAL_ADC_PollForConversion(hadc, 100);
	ADC_Read[0] = HAL_ADC_GetValue(hadc);

	HAL_ADC_Start_DMA(hadc, ADC_Read, ADC_BUFFER);
	if( HAL_GetTick() - *lastReadMS > SHOCK_TRAVEL_SAMPLE_PERIOD){
		dataframe->data.shockTravel = ADC_Read[0];

		*lastReadMS = HAL_GetTick();
	}

	//todo: convert counts to travel
}

void readBrakeTemp(uint32_t *lastReadMS, MISC_DATAFRAME *dataframe) {

	if(HAL_GetTick() - *lastReadMS > BRAKE_TEMP_SAMPLE_PERIOD){
		dataframe->data.brakeTemp = 0;
		//todo: actual brake temp sensor read code
		*lastReadMS = HAL_GetTick();
	}

	//todo: convert to deg C
}

void readTireTemp(uint32_t *lastReadMS, TTEMP_DATAFRAME *dataframes) {
	if (HAL_GetTick() - *lastReadMS > TIRE_TEMP_SAMPLE_PERIOD) {

		MLX90640_GetFrameData(MLX_ADDR, MLX_dataFrame);
		float tr = MLX90640_GetTa(MLX_dataFrame, &MLX_params) - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
		MLX90640_CalculateTo(MLX_dataFrame, &MLX_params, EMISSIVITY, tr, MLX_to);
		computeMLXSample();

		for (int i = 0; i < 4; i++) {
			dataframes[i].data.pix0 = MLX_sample[0 + 8*i];
			dataframes[i].data.pix0 = MLX_sample[1 + 8*i];
			dataframes[i].data.pix1 = MLX_sample[2 + 8*i];
			dataframes[i].data.pix2 = MLX_sample[3 + 8*i];
			dataframes[i].data.pix3 = MLX_sample[4 + 8*i];
			dataframes[i].data.pix4 = MLX_sample[5 + 8*i];
			dataframes[i].data.pix5 = MLX_sample[6 + 8*i];
			dataframes[i].data.pix6 = MLX_sample[7 + 8*i];
		}

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

	if(HAL_GetTick() - *lastReadMS > WHEEL_SPEED_SAMPLE_PERIOD){

		uint8_t prevWHSLogicLevel = GPIO_PIN_RESET;




		uint8_t edges = 0;
		uint8_t readBeginMS = HAL_GetTick(); //possilbly a good idea to lower tick period to like 10us or sth
		for (int i = 0; i < 1000; i++) {//burst read 100 values real quick, find how many times polarity switches

			/* if whs pin is logic high and prev_whs_logic_level is opposite, add one to edges */

			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) != prevWHSLogicLevel) {
				edges++;
				prevWHSLogicLevel = !prevWHSLogicLevel;
			}
		}

		uint8_t readEndMS = HAL_GetTick();


		//convert to rpm
		/*
		 * edges/msec * 1/(edges/rotation) * msec/sec = rotations/msec
		 * 1/(edges/rotation) * msec/sec = 1/24 * 1/1000 =
		 */
		dataframe->data.wheelRPM = ( ((float)(edges)) / ((float)(readEndMS)-(float)(readBeginMS)) ) * (float)(1/24000);
		printf("%d", (int) (dataframe->data.wheelRPM*1000.0));


		*lastReadMS = HAL_GetTick();
	}
}

void readBoardTemp(SPI_HandleTypeDef *hspi, uint32_t *lastReadMS, MISC_DATAFRAME *dataframe) {

	if(HAL_GetTick() - *lastReadMS > STRAIN_GAUGE_SAMPLE_PERIOD){
		dataframe->data.boardTemp = 0;


		//todo: use ads1118, same chip as the shits, to read board temp
		//todo: convert counts to deg.C
		*lastReadMS = HAL_GetTick();
	}
}
