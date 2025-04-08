/*
 * dataframes.h
 *
 *  Created on: Apr 7, 2025
 *      Author: antho
 */

#ifndef INC_DATAFRAMES_H_
#define INC_DATAFRAMES_H_

#include <stdbool.h>

/* start unions for dataframes */

typedef union SG_DATAFRAME{
	struct {
		uint16_t SG0; //strain gauge 0 force (N) * 10
		uint16_t SG1; //strain gauge 1 force (N) * 10
		uint16_t SG2; //strain gauge 2 force (N) * 10
		uint16_t SG3; //strain gauge 3 force (N) * 10
	} data;
	uint8_t array[8];
} SG_DATAFRAME;

typedef union TTEMP_DATAFRAME {
	struct {
		uint8_t pix0;
		uint8_t pix1;
		uint8_t pix2;
		uint8_t pix3;
		uint8_t pix4;
		uint8_t pix5;
		uint8_t pix6;
		uint8_t pix7;

	} data;
	uint8_t array[8];
} TTEMP_DATAFRAME;

typedef union MISC_DATAFRAME {
	struct {
		uint16_t wheelRPM; //RPMs * 60
		uint16_t brakeTemp; //brake temp * 1000
		uint16_t shockTravel; //shock travel * 1000
		uint8_t boardTemp; //board temp * 2
		struct {
			bool ADCErrorBit			: 1;
			bool BrakeTempErrorBit		: 1;
			bool MiscMsgErrorBit 		: 1;
			bool SGMsgErrorBit	 		: 1;
			bool TTempMsg1ErrorBit 		: 1;
			bool TTempMsg2ErrorBit 		: 1;
			bool TTempMsg3ErrorBit		: 1;
			bool TTempMsg4ErrorBit		: 1;
		} eflags;
	} data;
	uint8_t array[8];
} MISC_DATAFRAME;

/* end unions for dataframes */

#endif /* INC_DATAFRAMES_H_ */
