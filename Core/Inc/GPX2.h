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
	uint8_t GPX2_tx[25];
	uint8_t GPX2_rx[25];
	uint32_t prev_rec[2][4];
	uint32_t curr_res[4];
	uint8_t count_consec;

	// Transmit array
	uint8_t *USB_tx;
	uint8_t USB_tx_1[1024];
	uint8_t USB_tx_2[1024];
	uint8_t USB_tx_track;
	uint16_t USB_tx_index;
	uint8_t USB_busy_flag;

	// uSD array
	uint8_t *uSD_tx;
	uint8_t uSD_tx_1[4096];
	uint8_t uSD_tx_2[4096];
	uint8_t uSD_tx_track;
	uint16_t uSD_tx_index;
	uint16_t uSD_file_counter;
	uint32_t uSD_file_index;

	// Averaging counter
	uint16_t count_to;

	// Print flag
	uint8_t USB_flag;
	uint8_t uSD_flag;

	// Init flag - interrupts don't trigger unless this flag is set to a non-0 value, automatically
	// handled during the init_GPX2 function
	uint8_t is_init;

	// Random number
	uint16_t random_number;
} GPX2;

// Unit Declaration
GPX2 GPX2x;

// Helper functions

void init_GPX2(GPX2 *Unit, SPI_HandleTypeDef *SPI_interface,
		GPIO_TypeDef *NSS_Bank, uint16_t NSS_Pin, GPIO_TypeDef *INT_Bank,
		uint16_t INT_Pin, uint16_t random_number);

int read_results_GPX2(GPX2 *Unit);

#endif /* INC_GPX2_H_ */
