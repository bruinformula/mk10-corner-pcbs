/*
 * ads1118.h
 *
 *  Created on: Feb 20, 2025
 *      Author: antho
 *      i really do not know what i am doing but i feel it is probably smart
 *      to have a dedicated driver file for this adc
 */

#ifndef SRC_ADS1118_H_
#define SRC_ADS1118_H_
#include "stm32l4xx.h"


class ads1118 {
public:
	ads1118(uint_8 SPIBusPins[], bool debug);
	ads1118(bool debug);
	bool InitADS1118(uint_8 fsr, uint_8 datarate, bool ADCReadMode);
	int* BurstReadSingleEndedValues(bool converted);
	int* BurstReadDifferentialValues(bool converted);
	//bool StartContinuousRead();
	int* PullADCValues(bool converted, bool* newData);
	int ReadValue(uint_8 muxCode);
	virtual ~ads1118();


	#define CONFIG_BIT_RESV 1;
	union ADS_InitStruct {
	    struct params{//16 bits total, 4 bytes
	        volatile unsigned char    RESV			:1;   //low
	        volatile unsigned char    NOP			:2;
	        volatile unsigned char    PULLUP		:1;
	        volatile unsigned char    TS_MODE		:1;
	        volatile unsigned char    DATARATE		:3;
	        volatile unsigned char    OP_MODE		:1;
	        volatile unsigned char    PGA			:3;
	        volatile unsigned char    MUX			:3;
	        volatile unsigned char    SS			:1;   //high
	    };
	    volatile unsigned int  word; //4 bytes
	    volatile unsigned char byte[2]; //array of 2 chars, so 2 bytes
	};

	enum NOP {
		DATA_VALID      = 0x1,
		DATA_INVALID    = 0x2
	};
	enum PULLUP {
		PUP_DISABLED	=	0x0,
		PUP_ENABLED		=	0x1
	};
	enum ADSMODE {
		CONTINUOUS	=	0x0,
		SINGLESHOT	=	0x1
	};

	enum DATARATE {
		DR_8_SPS	=   0x0,
		DR_16_SPS	=   0x1,
		DR_32_SPS	=   0x2,
		DR_64_SPS 	=   0x3,
		DR_128_SPS	=   0x4,
		DR_250_SPS	=   0x5,
		DR_475_SPS	=   0x6,
		DR_860_SPS	=   0x7
	};

	enum PGA {
	    PGA_6144 	=	0x0,
	    PGA_4096 	=	0x1,
	    PGA_2048 	=	0x2,
	    PGA_1024 	=	0x3,
	    PGA_512 	=	0x4,
	    PGA_256 	=	0x5
	};

	enum MUX {
	    AINPN_0_1 	= 	0x0,
	    AINPN_0_3 	=   0x1,
	    AINPN_1_3 	=   0x2,
	    AINPN_2_3 	=   0x3,
	    AINPN_0_GND	=  	0x4,
	    AINPN_1_GND	=  	0x5,
	    AINPN_2_GND	=  	0x6,
	    AINPN_3_GND	=  	0x7
	};

	enum ADS_MODE {
	    ADC_MODE    =   0x0,
	    TEMP_MODE 	=  	0x1
	};

	enum DATAREADY {
		DATAREADY	=	0x0,
		DATANREADY	=	0x1
	};
private:
	void SPI_RCC_Config();
	void SPI_GPIO_Config();
	void SPI_NVIC_Config();
	bool SPIWrite();
	bool Convert(uint_32 values[]);
	void PrintDebugStatus(uint_8 status);
	bool debug = false;

	ADS_InitStruct configReg;

	enum PGA_CONV {
		CONV_PGA_6144 	=	5333,
		CONV_PGA_4096 	=	8000,
		CONV_PGA_2048 	=	16000,
		CONV_PGA_1024 	=	32000,
		CONV_PGA_512 	=	64000,
		CONV_PGA_256 	=	128000 ///ts gonna overflow....................................
	};

	//-------CONSTANTS-------//
	#define DEFAULT_PGA			PGA_2048
	#define ADS1118_DISABLE		GPIO_SetBits(SPI_MASTER_GPIO, SPI_MASTER_PIN_NSS) //send CS high
	#define ADS1118_ENABLE		GPIO_ResetBits(SPI_MASTER_GPIO, SPI_MASTER_PIN_NSS)//send CS low
	/***************SPI*******************/
	#define SPI_TIMEOUT			10		//10ms
	#define SPI_MAX_TRIES		3		//the max amount of tries any read or write command will do
	#define SPI					SPI2
	#define SPI_CLK				RCC_APB1Periph_SPI2
	#define SPI_GPIO            GPIOB
	#define SPI_GPIO_CLK        RCC_APB2Periph_GPIOB
	#define	SPI_PIN_NSS			GPIO_Pin_10
	#define SPI_PIN_SCK         GPIO_Pin_11
	#define SPI_PIN_MISO        GPIO_Pin_12
	#define SPI_PIN_MOSI        GPIO_Pin_13
	#define SPI_IRQn            SPI2_IRQn
	/*************END SPI****************/

	/*************GPIO*******************/
};
#endif /* SRC_ADS1118_H_ */

