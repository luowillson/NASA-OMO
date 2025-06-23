#include "GPX2.h"
#include "GPX2_reg.h"

void init_GPX2(GPX2 *Unit, SPI_HandleTypeDef *SPI_interface,
		GPIO_TypeDef *NSS_Bank, uint16_t NSS_Pin, GPIO_TypeDef *INT_Bank,
		uint16_t INT_Pin) {

	// Initialize GPX2 struct
	Unit->GPX2_SPI = SPI_interface;
	Unit->NSS_Bank = NSS_Bank;
	Unit->NSS_Pin = NSS_Pin;

	// FOR US IT SHOULD BE C7
//	Unit->NSS_Bank = INT_Bank;
//	Unit->NSS_Pin = INT_Pin;

	// Reset counter
	Unit->count_consec = 0;
	Unit->count_to = 0;

	uint8_t testing[20] = { 0 };
	uint8_t testing2[20] = { 0 };

	// Power cycle GPX2
	uint8_t temp_8[1] = { spiopc_power };
	// Pulse SS pin to initiate transmission
	HAL_GPIO_WritePin(Unit->NSS_Bank, Unit->NSS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Unit->NSS_Bank, Unit->NSS_Pin, GPIO_PIN_RESET);
	// Complete transmission of power command
	HAL_SPI_Transmit(Unit->GPX2_SPI, (uint8_t*) temp_8, 1, HAL_MAX_DELAY);

	// Delay (required by datasheet supposedly)
	HAL_Delay(10);

	// Write configuration
	temp_8[0] = (0x80);
	uint8_t GPX2_config_reg_vals_0[20] = { 0x01, 0x81, 0xDF, 0xA0, 0x86, 0x01,
			0xC0, 0xD3, 0xA1, 0x13, 0x00, 0x0A, 0xCC, 0xCC, 0xF1, 0x7D, 0x04,
			0x00, 0x00, 0x00 };
	// Pulse SS pin to initiate transmission
	HAL_GPIO_WritePin(Unit->NSS_Bank, Unit->NSS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Unit->NSS_Bank, Unit->NSS_Pin, GPIO_PIN_RESET);
	// Complete transmission of write command and values
	HAL_SPI_Transmit(Unit->GPX2_SPI, (uint8_t*) temp_8, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(Unit->GPX2_SPI, (uint8_t*) GPX2_config_reg_vals_0, 20,
	HAL_MAX_DELAY);

	// Delay (required by datasheet supposedly)
	HAL_Delay(10);

	// Read configuration
	testing[0] = (0x40);
	// Pulse SS pin to initiate transmission
	HAL_GPIO_WritePin(Unit->NSS_Bank, Unit->NSS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Unit->NSS_Bank, Unit->NSS_Pin, GPIO_PIN_RESET);
	// Complete transmission of read command and values
	HAL_SPI_TransmitReceive(Unit->GPX2_SPI, (uint8_t*) testing,
			(uint8_t*) testing2, 20, HAL_MAX_DELAY);

	// Delay (required by datasheet supposedly)
	HAL_Delay(10);

	// Initialization
	temp_8[0] = spiopc_init;
	// Pulse SS pin to initiate transmission
	HAL_GPIO_WritePin(Unit->NSS_Bank, Unit->NSS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Unit->NSS_Bank, Unit->NSS_Pin, GPIO_PIN_RESET);
	// Complete transmission of initialization command
	HAL_SPI_Transmit(Unit->GPX2_SPI, (uint8_t*) temp_8, 1, HAL_MAX_DELAY);
}

int read_results_GPX2(GPX2 *Unit) {

	// Prepare array for transmission
	uint8_t testing_empty[19] = { 0 };
	testing_empty[0] = (spiopc_read_results + 8);

	// Pulse SS pin to initiate transmission
	HAL_GPIO_WritePin(Unit->NSS_Bank, Unit->NSS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Unit->NSS_Bank, Unit->NSS_Pin, GPIO_PIN_RESET);
	// Transmit / Receive in DMA
	HAL_SPI_TransmitReceive_DMA(Unit->GPX2_SPI, testing_empty, Unit->rec, 19);

	return 0;
}
