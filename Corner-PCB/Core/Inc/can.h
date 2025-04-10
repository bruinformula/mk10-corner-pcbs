/*
 * constants.h
 *
 *  Created on: April 5, 2025
 *      Author: wony tang
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_can.h"
#include "stm32l4xx_hal_conf.h"
#include "dataframes.h"


/* CAN ID shit*/
#define CORNER_NUMBER 1 // n
#define STRAIN_GAUGE_ID 	0b11100000000 | (CORNER_NUMBER << 4) //7n0
#define TIRE_TEMP_MSG1_ID 	0b11100000001 | (CORNER_NUMBER << 4) //7n1
#define TIRE_TEMP_MSG2_ID 	0b11100000010 | (CORNER_NUMBER << 4) //7n2
#define TIRE_TEMP_MSG3_ID 	0b11100000011 | (CORNER_NUMBER << 4) //7n3
#define TIRE_TEMP_MSG4_ID 	0b11100000100 | (CORNER_NUMBER << 4) //7n4
#define MISC_DATA_ID 		0b11100000101 | (CORNER_NUMBER << 4) //7n5

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
#define BRAKE_TEMP_SAMPLE_PERIOD		200
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
/* end scaling factors */

/* method declarations */
void CANMailman(CAN_HandleTypeDef *canport, CAN_TxHeaderTypeDef *header, CORNER_CAN_CONTEXT *CANCONTEXT);

#endif /* INC_CAN_H_*/
