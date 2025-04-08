/*
 * constants.h
 *
 *  Created on: April 5, 2025
 *      Author: wony tang
 */

#ifndef INC_CONSTANTS_H_
#define INC_CONSTANTS_H_

/* CAN ID shit*/
#define CORNER_NUMBER 1
#define STRAIN_GAUGE_IDENTIFIER_TEMPLATE 	0b11100000000 | (CORNER_NUMBER << 4)
#define TIRE_TEMP_IDENTIFIER_TEMPLATE_MSG1 	0b11100000001 | (CORNER_NUMBER << 4)
#define TIRE_TEMP_IDENTIFIER_TEMPLATE_MSG2 	0b11100000010 | (CORNER_NUMBER << 4)
#define TIRE_TEMP_IDENTIFIER_TEMPLATE_MSG3 	0b11100000011 | (CORNER_NUMBER << 4)
#define TIRE_TEMP_IDENTIFIER_TEMPLATE_MSG4 	0b11100000100 | (CORNER_NUMBER << 4)
#define MISC_DATA_IDENTIFIER_TEMPLATE 		0b11100000101 | (CORNER_NUMBER << 4)

/* how many times itll retry putting any CAN message in a mailbox before j giving up*/
#define CAN_RETRY_LIMIT 3

/* start transmission rates (ms) */
#define STRAIN_GAUGE_TRANSMISSION_PERIOD 	5
#define TIRE_TEMP_TRANSMISSION_PERIOD 		500
#define MISC_DATA_TRANSMISSION_PERIOD 		10
/* end transmission rates */


/* start sampling rates (ms) */
#define STRAIN_GAUGE_SAMPLE_PERIOD 		5
#define TIRE_TEMP_SAMPLE_PERIOD 		500
#define WHEEL_SPEED_SAMPLE_PERIOD 		10 //this is how long it outputs an actual speed
#define BRAKE_TEMP_SAMPLE_PERIOD		100
#define SHOCK_TRAVEL_SAMPLE_PERIOD		20
#define BOARD_TEMP_SAMPLE_PERIOD		1000
/* start sampling rates */


//these exist to effectively put more decimal points in a byte datafield
/* start scaling factors */
#define STRAIN_GAUGE_SF 	10
#define TIRE_TEMP_SF 		4
#define WHEEL_SPEED_SF 		1 //this is how long it outputs an actual speed
#define BRAKE_TEMP_SF		10
#define SHOCK_TRAVEL_SF		1000
#define BOARD_TEMP_SF		1000
/* start sampling rates */




#endif /* INC_CONSTANTS_H_ */
