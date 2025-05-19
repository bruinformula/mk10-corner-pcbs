/*
 * sensor_read_helpers.h
 *
 *  Created on: Apr 7, 2025
 *      Author: antho
 */

#ifndef INC_SENSOR_READ_HELPERS_H_
#define INC_SENSOR_READ_HELPERS_H_

// Include all our driver/peripheral libraries
#include "stdio.h"
#include "dataframes.h"
#include "can.h"
#include "i2c.h"
#include "spi.h"
#include "adc.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "ads1118.h"

// Necessary defines for all the pins + other parameters
#define ADS_EN_PORT GPIOA
#define ADS_EN_PIN GPIO_PIN_4

#define RS485_EN_PORT GPIOB
#define RS485_EN_PIN GPIO_PIN_3

#define SG_GF 2.08 // According to Amazon
#define YG_MODULUS 205 // in gPa

#define MLX_ADDR 0x33
#define TA_SHIFT 8
#define EMISSIVITY 0.95

#define LINPOT_PORT GPIOA
#define LINPOT_PIN GPIO_PIN_1
#define LINPOT_STROKE_LENGTH 80 // in mm; NEED TO CALIBRATE

#define EDGES_PER_REVOLUTION 24.0 // NEED TO CALIBRATE
#define HALL_EFFECT_SAMPLE_INTERVAL 250.0

// Any other necessary variables
extern ADS StrainGaugeADS; // Can also read board temp btw
extern float crossSectionalArea;

extern uint16_t MLX_eeData[832];
extern paramsMLX90640 MLX_params;
extern uint16_t MLX_dataFrame[834];
extern float MLX_to[768];
extern uint8_t MLX_sample[32];

extern uint16_t adcBuffer[1];
extern float linpot_reading;
#define LINPOT_CALIB_MAX_ANACOUNTS 1166//analog counts corresponding to 0% sensor travel; max load
#define LINPOT_CALIB_MIN_ANACOUNTS 0//analog counts corresponding to 100% sensor travel; no load



extern volatile int hall_effect_edges;

// Perform any necessary initializations of our hardware
bool initializeLinPot(ADC_HandleTypeDef* adcInstance);
bool initializeTireTemp();
bool initializeStrainGauge(SPI_HandleTypeDef* spiInstance);
void initializeBrakeTemp();
void initializeWheelSpeed();

// All our read functions called in main
void readLinearPotentiometer(ADC_HandleTypeDef *hadc, uint32_t *lastReadMS, MISC_DATAFRAME *dataframe);
void readBrakeTemp(uint32_t *lastReadMS, MISC_DATAFRAME *dataframe);
void readTireTemp(uint32_t *lastReadMS, TTEMP_DATAFRAME *dataframes);
void readStrainGauges(SPI_HandleTypeDef *hspi, uint32_t *lastReadMS, SG_DATAFRAME *dataframe);
void readWheelSpeed(uint32_t *lastReadMS, MISC_DATAFRAME *dataframe);
void readBoardTemp(SPI_HandleTypeDef *hspi, uint32_t *lastReadMS, MISC_DATAFRAME *dataframe);

// Helper computation functions
void getMLXSample();
float getStrainGaugeForce(float voltageReading);
float getLinPotTravel();
float getRPM();

#endif /* INC_SENSOR_READ_HELPERS_H_ */
