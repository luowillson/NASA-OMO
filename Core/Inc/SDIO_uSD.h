/*
 * SPI_uSD.h
 *
 *  Created on: Jul 28, 2024
 *      Author: alexiy
 */

#ifndef INC_SDIO_USD_H_
#define INC_SDIO_USD_H_

#include "fatfs.h"
#include "string.h"
#include "stdio.h"
#include "GPX2.h"

FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

uint8_t file_name_buffer[64];
uint8_t error_buffer[64];

FRESULT uSD_mount();
FRESULT uSD_open(uint8_t *file_name) ;
FRESULT uSD_close();
FRESULT uSD_write(uint8_t *file_name, uint8_t *contents, uint16_t length);

extern TIM_HandleTypeDef htim11;

#endif /* INC_SPI_USD_H_ */
