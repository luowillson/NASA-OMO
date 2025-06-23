#include "GPX2.h"
#include "GPX2_reg.h"

void init_GPX2(GPX2 *Unit, SPI_HandleTypeDef *SPI_interface,
		GPIO_TypeDef *NSS_Bank, uint16_t NSS_Pin, GPIO_TypeDef *INT_Bank,
		uint16_t INT_Pin, uint16_t random_number) {

	Unit->random_number = random_number;

	memset(Unit->GPX2_tx, 0, 25);
	memset(Unit->GPX2_rx, 0, 25);

	// Initialize GPX2 struct
	Unit->GPX2_SPI = SPI_interface;
	Unit->NSS_Bank = NSS_Bank;
	Unit->NSS_Pin = NSS_Pin;

	// Reset counter
	Unit->count_consec = 0;
	Unit->count_to = 0;

	// Reset flag
	Unit->USB_flag = 0;
	Unit->USB_busy_flag = 0;
	Unit->uSD_flag = 0;

	// Initialize USB Transmit Array
	Unit->USB_tx_1[0] = 0xAB;
	Unit->USB_tx_1[1] = 0xCD;
	Unit->USB_tx_1[1022] = 0xDC;
	Unit->USB_tx_1[1023] = 0xBA;

	Unit->USB_tx_2[0] = 0xAB;
	Unit->USB_tx_2[1] = 0xCD;
	Unit->USB_tx_2[1022] = 0xDC;
	Unit->USB_tx_2[1023] = 0xBA;

	Unit->USB_tx = Unit->USB_tx_1;
	Unit->USB_tx_track = 1;

	Unit->USB_tx_index = 0;

	// Initialize uSD Transmit Array
	Unit->uSD_tx = Unit->uSD_tx_1;
	Unit->uSD_tx_track = 1;

	Unit->uSD_tx_index = 0;
	Unit->uSD_file_counter = 0;
	Unit->uSD_file_index = 0;

	// Initialize the GPX2
	uint8_t testing[20] = { 0 };
	uint8_t testing2[20] = { 0 };

	// Power cycle GPX2
	Unit->GPX2_tx[0] = spiopc_power;
	// Pulse SS pin to initiate transmission
	HAL_GPIO_WritePin(Unit->NSS_Bank, Unit->NSS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Unit->NSS_Bank, Unit->NSS_Pin, GPIO_PIN_RESET);
	// Complete transmission of power command
	HAL_SPI_Transmit(Unit->GPX2_SPI, Unit->GPX2_tx, 1, HAL_MAX_DELAY);

	// Delay (required by datasheet supposedly)
	HAL_Delay(10);

	memset(Unit->GPX2_tx, 0, 25);
	memset(Unit->GPX2_rx, 0, 25);

	uint8_t default_configuration[20] = { 0x03, 0x83, 0xDF, 0xA0, 0x86, 0x01,
			0xC0, 0xD3, 0xA1, 0x13, 0x00, 0x0A, 0xCC, 0xCC, 0xF1, 0x7D, 0x04,
			0x00, 0x00, 0x00 };

	// Write configuration
	Unit->GPX2_tx[0] = (0x80);
	memcpy(&Unit->GPX2_tx[1], default_configuration, 20);
	// Pulse SS pin to initiate transmission
	HAL_GPIO_WritePin(Unit->NSS_Bank, Unit->NSS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Unit->NSS_Bank, Unit->NSS_Pin, GPIO_PIN_RESET);
	// Complete transmission of write command and values
	HAL_SPI_Transmit(Unit->GPX2_SPI, Unit->GPX2_tx, 21, HAL_MAX_DELAY);

	// Delay (required by datasheet supposedly)
	HAL_Delay(10);

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

	for (uint8_t i = 0; i < 21; i++) {
		if (default_configuration[i] != Unit->GPX2_rx[i + 1]) {
			config_check = 0;
		}
	}

	// Delay (required by datasheet supposedly)
	HAL_Delay(10);

	memset(Unit->GPX2_tx, 0, 25);
	memset(Unit->GPX2_rx, 0, 25);

	// Initialization
	Unit->GPX2_tx[0] = spiopc_init;
	// Pulse SS pin to initiate transmission
	HAL_GPIO_WritePin(Unit->NSS_Bank, Unit->NSS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Unit->NSS_Bank, Unit->NSS_Pin, GPIO_PIN_RESET);
	// Complete transmission of initialization command
	HAL_SPI_Transmit(Unit->GPX2_SPI, Unit->GPX2_tx, 1, HAL_MAX_DELAY);

	memset(Unit->GPX2_tx, 0, 25);
	memset(Unit->GPX2_rx, 0, 25);
}

int read_results_GPX2(GPX2 *Unit) {

	Unit->GPX2_tx[0] = (spiopc_read_results + 8);

	// Pulse SS pin to initiate transmission
	HAL_GPIO_WritePin(Unit->NSS_Bank, Unit->NSS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Unit->NSS_Bank, Unit->NSS_Pin, GPIO_PIN_RESET);
	// Transmit / Receive in DMA
	if (HAL_SPI_TransmitReceive_DMA(Unit->GPX2_SPI, Unit->GPX2_tx, Unit->GPX2_rx,
			19) != HAL_OK) {
		return 1;
	}

	return 0;
}
