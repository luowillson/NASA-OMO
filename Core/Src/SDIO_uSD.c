/*
 * SPI_uSD.c
 *
 *  Created on: Jul 28, 2024
 *      Author: alexiy
 */

#include "SDIO_uSD.h"
#include "fatfs.h"
#include "string.h"
#include "stdio.h"
#include "GPX2.h"

FRESULT uSD_record_configuration(GPX2 *Unit) {
	uint8_t buffer[512] = { "\0" };
	uint8_t *pos = buffer;

	memset(Unit->GPX2_tx, 0, 25);
	memset(Unit->GPX2_rx, 0, 25);

	// Read configuration
	Unit->GPX2_tx[0] = (0x40);
	// Pulse SS pin to initiate transmission
	HAL_GPIO_WritePin(Unit->NSS_Bank, Unit->NSS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Unit->NSS_Bank, Unit->NSS_Pin, GPIO_PIN_RESET);
	// Complete transmission of read command and values
	HAL_SPI_TransmitReceive(Unit->GPX2_SPI, Unit->GPX2_tx, Unit->GPX2_rx, 21,
	HAL_MAX_DELAY);

	uint8_t config_check = 1;

	pos += sprintf(pos, "GPX2 Config Registers: ");

	for (uint8_t i = 1; i < 21; i++) {
		pos += sprintf(pos, "0x%02x, ", Unit->GPX2_rx[i]);
	}

	pos += sprintf(pos, "Number: %u\r\n", Unit->random_number);

	memset(Unit->GPX2_tx, 0, 25);
	memset(Unit->GPX2_rx, 0, 25);

	uSD_open("CONFIG.TXT");
	uSD_write("CONFIG.TXT", buffer, strlen(buffer));
	uSD_close();
}

FRESULT uSD_mount() {
	// Mount uSD card FS
	fresult = f_mount(&fs, "/", 1);
	if (fresult == FR_OK) {
		// Get free space on uSD card
		f_getfree("", &fre_clust, &pfs);
		uSD_close();
	} else {

		// In the event that it is not successful, loop repeatedly remounting the uSD card until it is mounted
		for (int i = 0; i < 10; i++) {
			FATFS_UnLinkDriver("/");
			MX_FATFS_Init();
			fresult = f_mount(&fs, "/", 1);
			// Get free space on uSD card
			f_getfree("", &fre_clust, &pfs);
		}

		if (fresult != FR_OK) {
			NVIC_SystemReset();
		}

		// Add entry to error buffer
		sprintf(error_buffer, "mount %luf%lu.dat, %lu\r\n", GPX2x.random_number, GPX2x.uSD_file_index,
				GPX2x.uSD_file_counter);
		uSD_open("ERROR.TXT");
		uSD_write("ERROR.TXT", error_buffer, strlen(error_buffer));
		uSD_close();
	}

	// Return result
	return fresult;
}

FRESULT uSD_open(uint8_t *file_name) {
	// Open file to write/ create a file if it doesn't exist
	fresult = f_open(&fil, file_name, FA_WRITE | FA_OPEN_APPEND);

	if (fresult != FR_OK) {
		uSD_mount();

		sprintf(error_buffer, "open f%lu.dat, %lu\r\n", GPX2x.uSD_file_index,
				GPX2x.uSD_file_counter);
		uSD_open("ERROR.TXT");
		uSD_write("ERROR.TXT", error_buffer, strlen(error_buffer));
		uSD_close();

		uSD_open(file_name);
	}

	return fresult;
}

FRESULT uSD_close() {
	// Close file
	fresult = f_close(&fil);

	return fresult;
}

// uSD write is a DMA function - you can not write too quickly (hence the 20 ksps rate) otherwise it will try to access
// a busy resource

FRESULT uSD_write(uint8_t *file_name, uint8_t *contents, uint16_t length) {
	// Obtain result from writing
	fresult = f_write(&fil, contents, length, NULL);

	// If the result is not FR_OK, remount the uSD card and reopen the file for writing
	if (fresult != FR_OK) {
		uSD_mount();

		sprintf(error_buffer, "write f%lu.dat, %lu\r\n", GPX2x.uSD_file_index,
				GPX2x.uSD_file_counter);
		uSD_open("ERROR.TXT");
		uSD_write("ERROR.TXT", error_buffer, strlen(error_buffer));
		uSD_close();

		uSD_open(file_name);
		uSD_write(file_name, contents, length);
	}

	return fresult;
}

