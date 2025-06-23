/*
 * GPX2.h
 *
 *  Created on: Feb 13, 2024
 *      Author: alexi
 */

#ifndef INC_GPX2_H_
#define INC_GPX2_H_

// Includes
#include <stdint.h>
#include "GPX2_reg.h"
#include "stm32f4xx_hal.h"

// Struct Declaration
typedef struct {
	// SPI Interface
	SPI_HandleTypeDef *GPX2_SPI;
	GPIO_TypeDef *NSS_Bank;
	uint16_t NSS_Pin;

	// Interrupt pin
	GPIO_TypeDef *INT_Bank;
	uint16_t INT_Pin;

	// Receive array
	uint8_t rec[19];
	uint32_t prev_rec[2][4];
	uint32_t curr_res[4];
	uint8_t count_consec;

	// Averaging counter
	uint16_t count_to;
} GPX2;

// Unit Declaration
GPX2 GPX2x;

// Helper functions

void init_GPX2(GPX2 *Unit, SPI_HandleTypeDef *SPI_interface,
		GPIO_TypeDef *NSS_Bank, uint16_t NSS_Pin, GPIO_TypeDef *INT_Bank, uint16_t INT_Pin);

int read_results_GPX2(GPX2 *Unit);

#endif /* INC_GPX2_H_ */
