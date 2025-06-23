/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SDIO_uSD.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>

#include "GPX2.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */
char TxBuffer[250];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM11_Init(void);
static void MX_RNG_Init(void);
static void MX_TIM2_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void USB_CDC_Print(uint8_t *TxStr) {
	if (GPX2x.USB_busy_flag == 0){
		GPX2x.USB_busy_flag = 1;
		CDC_Transmit_FS((uint8_t*) TxStr, 1024);
		GPX2x.USB_busy_flag = 0;
	}
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	GPX2x.is_init = 0;
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_SDIO_SD_Init();
	MX_SPI2_Init();
	MX_FATFS_Init();
	MX_USB_DEVICE_Init();
	MX_TIM11_Init();
	MX_RNG_Init();
	MX_TIM2_Init();
	MX_RTC_Init();
	/* USER CODE BEGIN 2 */

	uint16_t random_number = HAL_RNG_GetRandomNumber(&hrng);

	// SDIO SD Card Initialization Code
		uSD_mount();

	// GPX2 Initialization Code
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_Delay(100);
	init_GPX2(&GPX2x, &hspi2, GPIOC, GPIO_PIN_6, GPIOC, GPIO_PIN_7,
			random_number);

	// MOVE THIS TO THE STRUCT / FUNCTION
	HAL_TIM_Base_Start_IT(&htim11);

	// Initialization recording
	uSD_record_configuration(&GPX2x);

	// Open first file to write to by setting name buffer
	sprintf(file_name_buffer, "%luf%05lu.dat", GPX2x.random_number,
			GPX2x.uSD_file_index);
	uSD_open(file_name_buffer);

	// Timestamp generation
	HAL_TIM_Base_Start(&htim2);

	GPX2x.is_init = 1;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		// Ping pong buffer for the USB transmission
		if (GPX2x.USB_flag == 1) {
			if (GPX2x.USB_tx_track == 1) {
				USB_CDC_Print((uint8_t*) GPX2x.USB_tx_2);
			} else {
				USB_CDC_Print((uint8_t*) GPX2x.USB_tx_1);
			}
			GPX2x.USB_flag = 0;
		}

		// Ping pong buffer for the uSD storage, including creating new files at regular sample interg
		if (GPX2x.uSD_flag == 1) {
			GPX2x.uSD_flag = 2;
			if (GPX2x.uSD_tx_track == 1) {
				uSD_write(file_name_buffer, GPX2x.uSD_tx_2, 4096);
			} else {
				uSD_write(file_name_buffer, GPX2x.uSD_tx_1, 4096);
			}
			GPX2x.uSD_file_counter++;
			if (GPX2x.uSD_file_counter == 1024) {
				uSD_close();
				GPX2x.uSD_file_counter = 0;
				GPX2x.uSD_file_index++;
				sprintf(file_name_buffer, "%luf%05lu.dat", GPX2x.random_number,
						GPX2x.uSD_file_index);
				uSD_open(file_name_buffer);
			}
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 5;
	RCC_OscInitStruct.PLL.PLLN = 120;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 5;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief RNG Initialization Function
 * @param None
 * @retval None
 */
static void MX_RNG_Init(void) {

	/* USER CODE BEGIN RNG_Init 0 */

	/* USER CODE END RNG_Init 0 */

	/* USER CODE BEGIN RNG_Init 1 */

	/* USER CODE END RNG_Init 1 */
	hrng.Instance = RNG;
	if (HAL_RNG_Init(&hrng) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN RNG_Init 2 */

	/* USER CODE END RNG_Init 2 */

}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void) {

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */
	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

}

/**
 * @brief SDIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_SDIO_SD_Init(void) {

	/* USER CODE BEGIN SDIO_Init 0 */

	/* USER CODE END SDIO_Init 0 */

	/* USER CODE BEGIN SDIO_Init 1 */

	/* USER CODE END SDIO_Init 1 */
	hsd.Instance = SDIO;
	hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
	hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
	hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
	hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
	hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
	hsd.Init.ClockDiv = 0;
	/* USER CODE BEGIN SDIO_Init 2 */

	/* USER CODE END SDIO_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 29 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM11 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM11_Init(void) {

	/* USER CODE BEGIN TIM11_Init 0 */

	/* USER CODE END TIM11_Init 0 */

	/* USER CODE BEGIN TIM11_Init 1 */

	/* USER CODE END TIM11_Init 1 */
	htim11.Instance = TIM11;
	htim11.Init.Prescaler = 120 - 1;
	htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim11.Init.Period = 50 - 1;
	htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim11) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM11_Init 2 */
	/* USER CODE END TIM11_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	/* DMA1_Stream4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
	/* DMA2_Stream3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
	/* DMA2_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : CARD_DETECT_Pin */
	GPIO_InitStruct.Pin = CARD_DETECT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(CARD_DETECT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI2_CS_Pin */
	GPIO_InitStruct.Pin = SPI2_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PC7 */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {

	// In an FPGA, this entire process would be per-channel and fully parallelized
	// use LVDS, not SPI for FPGA (SPI needed for initial register configuration)

	// Load / calculate current result into GPX2 struct
	GPX2x.curr_res[0] = 100000
			* ((GPX2x.GPX2_rx[1] << 16 | GPX2x.GPX2_rx[2] << 8
					| GPX2x.GPX2_rx[3]) - GPX2x.prev_rec[0][0])
			+ ((GPX2x.GPX2_rx[4] << 16 | GPX2x.GPX2_rx[5] << 8
					| GPX2x.GPX2_rx[6]) - GPX2x.prev_rec[1][0]);

	GPX2x.curr_res[1] = 100000
			* ((GPX2x.GPX2_rx[7] << 16 | GPX2x.GPX2_rx[8] << 8
					| GPX2x.GPX2_rx[9]) - GPX2x.prev_rec[0][1])
			+ ((GPX2x.GPX2_rx[10] << 16 | GPX2x.GPX2_rx[11] << 8
					| GPX2x.GPX2_rx[12]) - GPX2x.prev_rec[1][1]);

	GPX2x.curr_res[2] = 100000
			* ((GPX2x.GPX2_rx[13] << 16 | GPX2x.GPX2_rx[14] << 8
					| GPX2x.GPX2_rx[15]) - GPX2x.prev_rec[0][2])
			+ ((GPX2x.GPX2_rx[16] << 16 | GPX2x.GPX2_rx[17] << 8
					| GPX2x.GPX2_rx[18]) - GPX2x.prev_rec[1][2]);

	// Increment the amount of counts we count to

	GPX2x.count_to++;

	// Always calculate previous results (STOP + Count)
	GPX2x.prev_rec[0][0] = (GPX2x.GPX2_rx[1] << 16 | GPX2x.GPX2_rx[2] << 8
			| GPX2x.GPX2_rx[3]);
	GPX2x.prev_rec[1][0] = (GPX2x.GPX2_rx[4] << 16 | GPX2x.GPX2_rx[5] << 8
			| GPX2x.GPX2_rx[6]);

	GPX2x.prev_rec[0][1] = (GPX2x.GPX2_rx[7] << 16 | GPX2x.GPX2_rx[8] << 8
			| GPX2x.GPX2_rx[9]);
	GPX2x.prev_rec[1][1] = (GPX2x.GPX2_rx[10] << 16 | GPX2x.GPX2_rx[11] << 8
			| GPX2x.GPX2_rx[12]);

	GPX2x.prev_rec[0][2] = (GPX2x.GPX2_rx[13] << 16 | GPX2x.GPX2_rx[14] << 8
			| GPX2x.GPX2_rx[15]);
	GPX2x.prev_rec[1][2] = (GPX2x.GPX2_rx[16] << 16 | GPX2x.GPX2_rx[17] << 8
			| GPX2x.GPX2_rx[18]);

	// Always record data to uSD card
	// Record via uSD - NO AVERAGING
	GPX2x.uSD_tx[0 + GPX2x.uSD_tx_index * 12] = GPX2x.curr_res[0] >> 24;
	GPX2x.uSD_tx[1 + GPX2x.uSD_tx_index * 12] = GPX2x.curr_res[0] >> 16;
	GPX2x.uSD_tx[2 + GPX2x.uSD_tx_index * 12] = GPX2x.curr_res[0] >> 8;
	GPX2x.uSD_tx[3 + GPX2x.uSD_tx_index * 12] = GPX2x.curr_res[0];

	GPX2x.uSD_tx[4 + GPX2x.uSD_tx_index * 12] = GPX2x.curr_res[1] >> 24;
	GPX2x.uSD_tx[5 + GPX2x.uSD_tx_index * 12] = GPX2x.curr_res[1] >> 16;
	GPX2x.uSD_tx[6 + GPX2x.uSD_tx_index * 12] = GPX2x.curr_res[1] >> 8;
	GPX2x.uSD_tx[7 + GPX2x.uSD_tx_index * 12] = GPX2x.curr_res[1];

	GPX2x.uSD_tx[8 + GPX2x.uSD_tx_index * 12] = GPX2x.curr_res[2] >> 24;
	GPX2x.uSD_tx[9 + GPX2x.uSD_tx_index * 12] = GPX2x.curr_res[2] >> 16;
	GPX2x.uSD_tx[10 + GPX2x.uSD_tx_index * 12] = GPX2x.curr_res[2] >> 8;
	GPX2x.uSD_tx[11 + GPX2x.uSD_tx_index * 12] = GPX2x.curr_res[2];

	// Add time stamp to data at index 4080-4083, if we are currently writing a new "page"?
	if (GPX2x.uSD_tx_index == 0){
		GPX2x.uSD_tx[4080] = (TIM2->CNT) >> 24;
		GPX2x.uSD_tx[4081] = (TIM2->CNT) >> 16;
		GPX2x.uSD_tx[4082] = (TIM2->CNT) >> 8;
		GPX2x.uSD_tx[4083] = (TIM2->CNT);
	}

	GPX2x.uSD_tx_index++;

	// Ping pong buffer swap - one is filled, now it is ready to be written to the uSD card
	// The other one is now the one to be filled
	if (GPX2x.uSD_tx_index == 340) {
		if (GPX2x.uSD_tx_track == 1) {
			GPX2x.uSD_tx_track = 2;
			GPX2x.uSD_tx = GPX2x.uSD_tx_2;
		} else {
			GPX2x.uSD_tx_track = 1;
			GPX2x.uSD_tx = GPX2x.uSD_tx_1;
		}
		GPX2x.uSD_tx_index = 0;
		GPX2x.uSD_flag = 1;
	}

	// This if statement ensures that the results that are transmitted are consecutive results - values in excess of
	// the value here can be safely discarded because they will be non-consecutive

	// No averaging in this section - afaik? this should be added eventually, but not super necessary - divide
	// reduces overall bandwidth

	if (GPX2x.curr_res[0] < 20000000) {

		GPX2x.count_consec++;

		// If 100 valid samples have been received, set flag to print results via USB
		if (GPX2x.count_to == 200) {
			GPX2x.USB_tx[0 + 2 + GPX2x.USB_tx_index * 12] = GPX2x.curr_res[0] >> 24;
			GPX2x.USB_tx[1 + 2 + GPX2x.USB_tx_index * 12] = GPX2x.curr_res[0] >> 16;
			GPX2x.USB_tx[2 + 2 + GPX2x.USB_tx_index * 12] = GPX2x.curr_res[0] >> 8;
			GPX2x.USB_tx[3 + 2 + GPX2x.USB_tx_index * 12] = GPX2x.curr_res[0];

			GPX2x.USB_tx[4 + 2 + GPX2x.USB_tx_index * 12] = GPX2x.curr_res[1] >> 24;
			GPX2x.USB_tx[5 + 2 + GPX2x.USB_tx_index * 12] = GPX2x.curr_res[1] >> 16;
			GPX2x.USB_tx[6 + 2 + GPX2x.USB_tx_index * 12] = GPX2x.curr_res[1] >> 8;
			GPX2x.USB_tx[7 + 2 + GPX2x.USB_tx_index * 12] = GPX2x.curr_res[1];

			GPX2x.USB_tx[8 + 2 + GPX2x.USB_tx_index * 12] = GPX2x.curr_res[2] >> 24;
			GPX2x.USB_tx[9 + 2 + GPX2x.USB_tx_index * 12] = GPX2x.curr_res[2] >> 16;
			GPX2x.USB_tx[10 + 2 + GPX2x.USB_tx_index * 12] = GPX2x.curr_res[2] >> 8;
			GPX2x.USB_tx[11 + 2 + GPX2x.USB_tx_index * 12] = GPX2x.curr_res[2];

			GPX2x.USB_tx_index++;

			GPX2x.count_to = 0;

			if (GPX2x.USB_tx_index == 85) {
				if (GPX2x.USB_tx_track == 1) {
					GPX2x.USB_tx_track = 2;
					GPX2x.USB_tx = GPX2x.USB_tx_2;
				} else {
					GPX2x.USB_tx_track = 1;
					GPX2x.USB_tx = GPX2x.USB_tx_1;
				}
				GPX2x.USB_flag = 1;
				GPX2x.USB_tx_index = 0;
			}
		}
	}
}

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */
	if (GPX2x.is_init ==  1 && htim->Instance == TIM11) {
		read_results_GPX2(&GPX2x);
	}
	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
