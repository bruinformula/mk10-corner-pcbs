/*
 * sensor_read_helpers.h
 *
 *  Created on: Apr 7, 2025
 *      Author: antho
 */

#ifndef INC_SENSOR_READ_HELPERS_H_
#define INC_SENSOR_READ_HELPERS_H_


#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_can.h"
#include "stm32l4xx_hal_conf.h"
#include "main.h"
#include "ct1000n.h"

void readLinearPotentiometer(ADC_HandleTypeDef *hadc, uint32_t *lastReadMS, MISC_DATAFRAME *dataframe);
void readBrakeTemp(uint32_t *lastReadMS, MISC_DATAFRAME *dataframe, UART_HandleTypeDef *uartPort);
void readTireTemp(uint32_t *lastReadMS, TTEMP_DATAFRAME *dataframes);
void readStrainGauges(SPI_HandleTypeDef *hspi, uint32_t *lastReadMS, SG_DATAFRAME *dataframe);
void readWheelSpeed(uint32_t *lastReadMS, MISC_DATAFRAME *dataframe);
void readBoardTemp(SPI_HandleTypeDef *hspi, uint32_t *lastReadMS, MISC_DATAFRAME *dataframe);


#endif /* INC_SENSOR_READ_HELPERS_H_ */
