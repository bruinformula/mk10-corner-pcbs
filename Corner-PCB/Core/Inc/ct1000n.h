/*
 * ct1000n.h
 *
 *  Created on: Mar 31, 2025
 *      Author: ishanchitale
 */

#ifndef INC_CT1000N_H_
#define INC_CT1000N_H_

#include "main.h"

#define MAX485_EN_PORT GPIOB
#define MAX485_EN_PIN GPIO_PIN_3



HAL_StatusTypeDef sendBrakeTempData(uint8_t* data, UART_HandleTypeDef *uartPort);
HAL_StatusTypeDef receiveBrakeTempData(uint8_t* rxdata, UART_HandleTypeDef *uartPort);
uint16_t computeCRC16(uint8_t* buf, uint16_t len);
// Write out general functions for getting Ambient Temp, Brake Temp later
// Additional functions for the other registers

#endif /* INC_CT1000N_H_ */
