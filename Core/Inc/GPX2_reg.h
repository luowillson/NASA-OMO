/*
 * GPX2_REG.h
 *
 *  Created on: Feb 13, 2024
 *      Author: alexi
 */

#ifndef INC_GPX2_REG_H_
#define INC_GPX2_REG_H_

// CONFIGURATION REGISTERS

#define CONFIG_LENGTH 20

// 1 channel with independent FIFO for each channel
//static

// 3 channels with independent FIFO for each channel - FIX
static uint8_t GPX2_config_reg_vals_1[CONFIG_LENGTH] = { 0x02, 0x82, 0xDF, 0xA0, 0x86, 0x01,
			0xC0, 0xD3, 0xA1, 0x13, 0x00, 0x0A, 0xCC, 0xCC, 0xF1, 0x7D, 0x04,
			0x00, 0x00, 0x00 };

// OPCODES

#define spiopc_power 		0x30
#define spiopc_init			0x18
#define spiopc_write_config	0x80	// Add register address to 0x80 to address a specific address
#define spiopc_read_results	0x60	//
#define spiopc_read_config	0x40

#endif /* INC_GPX2_REG_H_ */
