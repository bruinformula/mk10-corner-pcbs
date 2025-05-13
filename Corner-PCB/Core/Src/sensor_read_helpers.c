/*
 * sensor_read_helpers.c
 *
 *  Created on: Apr 7, 2025
 *      Author: antho
 */

#include "sensor_read_helpers.h"

ADS StrainGaugeADS;
float crossSectionalArea = (2.19)*(0.0001);

uint16_t MLX_eeData[832];
paramsMLX90640 MLX_params;
uint16_t MLX_dataFrame[834];
float MLX_to[768];
uint8_t MLX_sample[32];

uint16_t adcBuffer[1];
float linpot_reading;

volatile int hall_effect_edges;

void initializeWheelSpeed() {
	HAL_NVIC_DisableIRQ(EXTI4_IRQn);
}

void initializeBrakeTemp() {
	// Literally broken lol
}

bool initializeTireTemp() {
	int status_1, status_2, status_3;
	status_1 = MLX90640_SetChessMode(MLX_ADDR);
	status_2 = MLX90640_DumpEE(MLX_ADDR, MLX_eeData);
	status_3 = MLX90640_ExtractParameters(MLX_eeData, &MLX_params);
	if (status_1 == -1 || status_2 == -1 || status_3 == -1) return 0;
	return 1;
}

bool initializeLinPot(ADC_HandleTypeDef* adcInstance) {
	if (HAL_ADC_Start_DMA(adcInstance, (uint32_t*) adcBuffer, 1) != HAL_OK) {
		return 0;
	}
	return 1;
}

bool initializeStrainGauge(SPI_HandleTypeDef *spiInstance) {
	bool status_1, status_2;
	initADS(&StrainGaugeADS, spiInstance, ADS_EN_PORT, ADS_EN_PIN);
	status_1 = enableContinuousConversion(&StrainGaugeADS);
	status_2 = enableFSR_6144(&StrainGaugeADS);
	return (status_1 & status_2);
}

// ------- ALL THE READ FUNCTIONS -----------
void readLinearPotentiometer(ADC_HandleTypeDef *hadc, uint32_t *lastReadMS,  MISC_DATAFRAME *dataframe) {
	if( HAL_GetTick() - *lastReadMS > SHOCK_TRAVEL_SAMPLE_PERIOD) {
		HAL_ADC_Start_DMA(hadc, (uint32_t*) adcBuffer, 1);
		linpot_reading = getLinPotTravel();
		dataframe->data.shockTravel = (uint16_t)(linpot_reading*100);
		*lastReadMS = HAL_GetTick();
	}
	//todo: convert counts to travel
}

void readBrakeTemp(uint32_t *lastReadMS, MISC_DATAFRAME *dataframe) {
	// LITERALLY DOES NOT WORK
	if(HAL_GetTick() - *lastReadMS > BRAKE_TEMP_SAMPLE_PERIOD){
		dataframe->data.brakeTemp = 0;
		//todo: actual brake temp sensor read code --> this shit is broken
		*lastReadMS = HAL_GetTick();
	}

	//todo: convert to deg C
}

void readTireTemp(uint32_t *lastReadMS, TTEMP_DATAFRAME *dataframes) {
	if (HAL_GetTick() - *lastReadMS > TIRE_TEMP_SAMPLE_PERIOD) {
		MLX90640_GetFrameData(MLX_ADDR, MLX_dataFrame);
		float tr = MLX90640_GetTa(MLX_dataFrame, &MLX_params) - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
		MLX90640_CalculateTo(MLX_dataFrame, &MLX_params, EMISSIVITY, tr, MLX_to);
		getMLXSample();

		for (int i = 0; i < 4; i++) {
			dataframes[i].data.pix0 = MLX_sample[0 + 8*i];
			dataframes[i].data.pix1 = MLX_sample[1 + 8*i];
			dataframes[i].data.pix2 = MLX_sample[2 + 8*i];
			dataframes[i].data.pix3 = MLX_sample[3 + 8*i];
			dataframes[i].data.pix4 = MLX_sample[4 + 8*i];
			dataframes[i].data.pix5 = MLX_sample[5 + 8*i];
			dataframes[i].data.pix6 = MLX_sample[6 + 8*i];
			dataframes[i].data.pix7 = MLX_sample[7 + 8*i];
		}

		*lastReadMS = HAL_GetTick();
	}
}

void readStrainGauges(SPI_HandleTypeDef *hspi, uint32_t *lastReadMS, SG_DATAFRAME *dataframe) {
	if(HAL_GetTick() - *lastReadMS > STRAIN_GAUGE_SAMPLE_PERIOD){
		enableADCSensor(&StrainGaugeADS);

		enableAINPN_0_G(&StrainGaugeADS);
		continuousRead(&StrainGaugeADS);
		dataframe->data.SG0 = (uint16_t)(getStrainGaugeForce(StrainGaugeADS.voltage)*100);

		enableAINPN_1_G(&StrainGaugeADS);
		continuousRead(&StrainGaugeADS);
		dataframe->data.SG1 = (uint16_t)(getStrainGaugeForce(StrainGaugeADS.voltage)*100);

		enableAINPN_2_G(&StrainGaugeADS);
		continuousRead(&StrainGaugeADS);
		dataframe->data.SG2 = (uint16_t)(getStrainGaugeForce(StrainGaugeADS.voltage)*100);

		enableAINPN_3_G(&StrainGaugeADS);
		continuousRead(&StrainGaugeADS);
		dataframe->data.SG3 = (uint16_t)(getStrainGaugeForce(StrainGaugeADS.voltage)*100);

		*lastReadMS = HAL_GetTick();
	}
}

// Special interrupt callback function
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == WHS_IN_Pin) {
		hall_effect_edges++;
		// Everytime there is a rising edge, we increment the number of edges
	}
}

void readWheelSpeed(uint32_t *lastReadMS, MISC_DATAFRAME *dataframe) {
	if (HAL_GetTick() - *lastReadMS > WHEEL_SPEED_SAMPLE_PERIOD) {

		hall_effect_edges = 0; // Reset the count..
		HAL_NVIC_EnableIRQ(EXTI4_IRQn); // Turn on the interrupt
		HAL_Delay(HALL_EFFECT_SAMPLE_INTERVAL); // Burst read over 50 ms
		HAL_NVIC_DisableIRQ(EXTI4_IRQn);
		dataframe->data.wheelRPM = (uint16_t)(getRPM()); // Compute the RPM

		*lastReadMS = HAL_GetTick();
	}
}

// Old Wheel Speed Function

//void readWheelSpeed(uint32_t *lastReadMS, MISC_DATAFRAME *dataframe) {
//
//	if(HAL_GetTick() - *lastReadMS > WHEEL_SPEED_SAMPLE_PERIOD){
//
//		uint8_t prevWHSLogicLevel = GPIO_PIN_RESET;
//		uint8_t edges = 0;
//		uint8_t readBeginMS = HAL_GetTick(); // possilbly a good idea to lower tick period to like 10us or sth
//		for (int i = 0; i < 1000; i++) {//burst read 100 values real quick, find how many times polarity switches
//			/* if whs pin is logic high and prev_whs_logic_level is opposite, add one to edges */
//			if (HAL_GPIO_ReadPin(WHS_IN_PORT, WHS_IN_PIN) != prevWHSLogicLevel) {
//				edges++;
//				prevWHSLogicLevel = !prevWHSLogicLevel;
//			}
//		}
//		uint8_t readEndMS = HAL_GetTick();
//		//convert to rpm
//		/*
//		 * edges/msec * 1/(edges/rotation) * msec/sec = rotations/msec
//		 * 1/(edges/rotation) * msec/sec = 1/24 * 1/1000 =
//		 */
//		dataframe->data.wheelRPM = ( ((float)(edges)) / ((float)(readEndMS)-(float)(readBeginMS)) ) * (float)(1/24000);
//		printf("%d", (int) (dataframe->data.wheelRPM*1000.0));
//
//
//		*lastReadMS = HAL_GetTick();
//	}
//}

void readBoardTemp(SPI_HandleTypeDef *hspi, uint32_t *lastReadMS, MISC_DATAFRAME *dataframe) {
	if(HAL_GetTick() - *lastReadMS > STRAIN_GAUGE_SAMPLE_PERIOD){
		enableTempSensor(&StrainGaugeADS);
		continuousRead(&StrainGaugeADS);
		dataframe->data.boardTemp = StrainGaugeADS.temp;
		*lastReadMS = HAL_GetTick();
	}
}

// Other helper functions
void getMLXSample() {
	for (int i = 384; i < 416; i++) {
		MLX_sample[i - 384] = (uint8_t)MLX_to[i];
	}
}

float getStrainGaugeForce(float voltageReading) {
	float strain = (voltageReading/3.3*SG_GF);
	float stress = strain*YG_MODULUS;
	return (stress*crossSectionalArea);
}

float getLinPotTravel() {
	float position_mm = ((float)adcBuffer[0] / 4095.0) * LINPOT_STROKE_LENGTH;
	return position_mm;
}

float getRPM() {
	float edgesPerSecond = (hall_effect_edges*1000.0)/HALL_EFFECT_SAMPLE_INTERVAL;
	float rps = edgesPerSecond/PULSES_PER_REVOLUTION;
	return rps*60.0;
}

