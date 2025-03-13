/*
 * ads1118.cpp
 *
 *  Created on: Feb 20, 2025
 *      Author: antho
 */

#include "ads1118.h"
#include "stm32l4xx.h"


ads1118::ads1118(uint_8 SPIBusPins[], bool debug) {
	// TODO Auto-generated constructor stub
}

bool InitADS1118(uint_8 fsr, uint_8 datarate, boolean ADCReadMode) {
	//*****INITIALIZE SPI*****//
	SPI_InitTypeDef SPI_InitStructure;

	//**RCC CONFIGURATION**//
	SPI_RCC_Config();
	SPI_GPIO_Config();
	SPI_NVIC_Config(); //not rly sure if we need this.............. though time accuracy might be important to sync w other corners?



	/* Disable SPI_MASTER */
	SPI_Cmd(SPI_MASTER, DISABLE);
	/* SPI_MASTER configuration ------------------------------------------------*/
	SPI_InitStructure.SPI_Direction 			=	SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode 					=	SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize 				=	SPI_DataSize_16b; //does this need to be 8 bits lmao
	SPI_InitStructure.SPI_CPOL 					=	SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA 					=	SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS 					= 	SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler 	=	SPI_BaudRatePrescaler_128;
	SPI_InitStructure.SPI_FirstBit 				=	SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial 		=	7;

	HAL_SPI_Init(SPI_MASTER, &SPI_InitStructure); //add a while loop; while this doesn't return the OK status (0x1), try 3 times w/ delay, if not then just exit



	/* Enable SPI_MASTER TXE interrupt */
	//SPI_I2S_ITConfig(SPI_MASTER, SPI_I2S_IT_TXE, ENABLE);
	/* Enable SPI_SLAVE RXNE interrupt */
	// SPI_I2S_ITConfig(SPI_MASTER, SPI_I2S_IT_RXNE, ENABLE);


	/* Enable SPI_MASTER */
	SPI_Cmd(SPI_MASTER, ENABLE);
	//*****END INITALIZE SPI*****//


	//*****INITIALIZE ADC*****//


	configReg.params.NOP		=	DATA_VALID;
	configReg.params.PULLUP		=	PUP_DISABLED;
	configReg.params.TS_MODE	=	ADC_MODE;
	configReg.params.DR			=	DR_128_SPS;
	configReg.params.MODE		=	ADCReadMode;
	configReg.params.PGA		=	fsr;
	configReg.params.MUX		=	AINPN_0_GND;
	configReg.params.SS			=	0x1;   //high
	configReg.params.RESV		=	CONFIG_BIT_RESV;

	//*****END INITIALIZE ADC*****//

}

void SPI_GPIO_Config() {
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure SPI_MASTER pins-*/

	// Pin PB13 (SCLK) must be configured as as 50MHz push pull
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.Pin = SPI_PIN_SCK;
	GPIO_InitStructure.Mode = GPIO_Mode_AF_PP;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);

	// Pin PB14 (MISO) must be configured as as input pull-up
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.Pin = SPI_PIN_MISO;
	GPIO_InitStructure.Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);

	// Pin PB15 (MOSI) must be configured as as 50MHz push pull
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = SPI_PIN_MOSI;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);

	//SPI1 NSS
	//GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = SPI_PIN_NSS;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);

	GPIO_SetBits(SPI_GPIO, SPI_PIN_NSS);

}

void SPI_NVIC_Config() {
  NVIC_InitTypeDef NVIC_InitStructure;


  /* 1 bit for pre-emption priority, 3 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  /* Configure and enable SPI_MASTER interrupt -------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = SPI_MASTER_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void SPI_RCC_Config() {

  /* Enable GPIO clock for SPI_MASTER */
  RCC_APB2PeriphClockCmd(SPI_MASTER_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);

  /* Enable SPI_MASTER Periph clock */
  RCC_APB1PeriphClockCmd(SPI_MASTER_CLK, ENABLE);

}
/**
 * Burst read all single ended values.
 *
 * @param converted	burstReadSingleEndedValues will output converted voltage values if true
 * @return pointer to array of readings, either converted or not
 */
uint_16* BurstReadDifferentialValues(bool converted) {
	uint_16 readings[4];
	HAL_StatusTypeDef status;
	configReg.stru.NOP			=	DATA_VALID;
	configReg.stru.TS_MODE		=	ADC_MODE;
	configReg.stru.MODE			=	SINGLE_SHOT;
	//keep pga the same


	//write first settings so we can do less rereads
	configReg.stru.SS		=	SINGLE_CONVER_START;   //high to start a conversion
	configReg.stru.MUX		=	0x4;

	//write the new config params to the ads1118
	ADS1118_ENABLE;
	delay_us((uint_32)1);

	readerHelper(readings, 0x0, 0x3);

	if (converted) return Convert(readings);

	return readings;
}

/**
 * Burst read all single ended values.
 *
 * @param converted	burstReadSingleEndedValues will output converted voltage values if true
 * @return pointer to array of readings, either converted or not
 */
uint_16* BurstReadSingleEndedValues(bool converted) {
	uint_16 readings[4];
	HAL_StatusTypeDef status;
	configReg.stru.NOP			=	DATA_VALID;
	configReg.stru.TS_MODE		=	ADC_MODE;
	configReg.stru.MODE			=	SINGLE_SHOT;
	//keep pga the same


	//write first settings so we can do less rereads
	configReg.stru.SS		=	SINGLE_CONVER_START;   //high to start a conversion
	configReg.stru.MUX		=	0x4;

	//write the new config params to the ads1118
	ADS1118_ENABLE;
	delay_us((uint_32)1);

	readerHelper(readings, 0x4, 0x7);

	if (converted) return Convert(readings);

	return readings;
}

void ReaderHelper(uint_16* readings, byte startType, byte endType) {
	status = SPITradeData(configReg.word, readings[0]); //itll put junk data in index 0 but its okay because i overwrite it first loop thru
	ADS1118_DISABLE;
	delay_ms(1);
	for (int i = startType; i < endType; i++) {
		//write the new config params to the ads1118
		ADS1118_ENABLE;
		delay_us((uint_32)1);
		status = SPITradeData(configReg.word, readings[i-startType]);
		ADS1118_DISABLE;
		delay_ms(1);
		configReg.stru.SS		=	SINGLE_CONVER_START;   //high to start a conversion
		configReg.stru.MUX		=	i;
		//think i have to reread what the fuck am i havae to treread i went to the bathroom and forgot
		//reread data because data extracted from last SPITradeData() is actually outdated and from the previous read
	}

	//gotta check if the loop actually reads to the end of hte 4

	//get ddata from last read
	ADS1118_ENABLE;
	delay_us((uint_32)1);
	status = SPITradeData(configReg.word, readings[3]);
	ADS1118_DISABLE;
	delay_ms(1);
}

HAL_StatusTypeDef SPITradeData(const uint_16* TXData, uint_16* RXData) {

	uint_8 byte1 = 0;
	uint_8 byte2 = 0;

	uint_8 tries = 0;

	uint_8 status1 = HAL_SPI_TransmitReceive(SPI, (uint_8)(TXData >> 8), byte1, 6, SPI_TIMEOUT);
	uint_8 status2 = HAL_SPI_TransmitReceive(SPI, (uint_8), byte2, 6, SPI_TIMEOUT);

	while (tries < SPI_MAX_TRIES) {
		//abs no idea bout length, google says to add the amt of txdata with amt of rxdata in bytes...
		//	each register is 2 bytes, total 4 bytes sent. starts to send useful data after 2 bytes, so 2+4=6???
		//	https://electronics.stackexchange.com/questions/413899/what-is-hal-spi-transmitreceive-purpose-and-how-it-works
		while (status1 != HAL_OK) {
			if (status1 == HAL_TIMEOUT) return status; //exit if timeout

			status1 = HAL_SPI_TransmitReceive(SPI, (uint_8)(TXData >> 8), byte1, 6, SPI_TIMEOUT);
		}

		status2 = HAL_SPI_TransmitReceive(SPI, (uint_8)(TXData), byte2, 6, SPI_TIMEOUT);
		while (status2 != HAL_OK) {
			if (status2 == HAL_TIMEOUT) return status; //exit if timeout

			status2 = HAL_SPI_TransmitReceive(SPI, (uint_8)(TXData), byte2, 6, SPI_TIMEOUT);
		}


		tries++;
	}

	if (status1 != HAL_OK) return status1;
	if (status2 != HAL_OK) return status2;
	*RXData = (uint16_t)byte1 || ((uint16_t)byte2)<<8; //return the two output bytes, combined into one uint_16

	return HAL_OK;
}

bool Convert(uint_16* shits[]) {
	if (sizeof(arr) / sizeof(arr[0]) != 4) return false;

	uint_16 convFactor;
	switch (configReg.PGA) {
	case PGA_6144:
		convFactor = LSB_PGA_6144;
	case PGA_4096:
		convFactor = LSB_PGA_4096;
	case PGA_2048:
		convFactor = LSB_PGA_2048;
	case PGA_1024:
		convFactor = LSB_PGA_1024;
	case PGA_512:
		convFactor = LSB_PGA_512;
	case PGA_256:
		return false; //##########not currently supported because im lazy
		//convFactor = LSB_PGA_256;
	}

	for (int i = 0; i < 4; i++) {
		shits[i] = shits[i] / convFactor;
	}

	return true;
}

/*
bool StartContinuousRead(uint_8 fsr, uint_8 datarate) {
	configReg.params.NOP		=	DATA_VALID;
	configReg.params.PULLUP		=	PUP_DISABLED;
	configReg.params.TS_MODE	=	ADC_MODE;
	configReg.params.DR			=	DR_128_SPS;
	configReg.params.MODE		=	CONTINUOUS;
	configReg.params.PGA		=	fsr;
	configReg.params.MUX		=	AINPN_0_GND;
	configReg.params.SS			=	0x1;   //high
	configReg.params.RESV		=	CONFIG_BIT_RESV;


}
*/

ads1118::~ads1118() {
	// TODO Auto-generated destructor stub
	delete configReg;
}

