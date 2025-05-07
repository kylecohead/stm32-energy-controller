/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <ssd1306.h>
#include <ssd1306_fonts.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 400
#define OP_AMP_GAIN 1.33
#define DEBOUNCE_DELAY 20

// Font and color definitions
#define FONT Font_6x8
#define TEXT_COLOR  White
#define BG_COLOR    Black

// Display constants - adjusted for 128x32 OLED
#define DISPLAY_WIDTH  128
#define DISPLAY_HEIGHT 32
#define LINE_HEIGHT    10
#define LEFT_MARGIN    2
#define VALUE_COLUMN   60

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint8_t rx_buffer[50];
volatile uint8_t rx_index = 0;
volatile uint8_t dma_complete = 0;
uint32_t adc_buffer[BUFFER_SIZE];
uint32_t v_max;
uint32_t v_min;
uint32_t i_max;
uint32_t i_min;
float voltage = 0.0;
float voltage_buf[BUFFER_SIZE / 2];
float current_buf[BUFFER_SIZE / 2];

float current = 0.0;
float apparent = 0.0;
float power_factor = 0.0;
float units_left = 10.010;
float phase_diff = 0.0;
float real_power = 0.0;
float reactive_power = 0.0;
float energy = 0.0;
float max_power = 3000.0;
float min_units = 10.0;
int unit_add = 0;
int count_units = 1;

// Define states for the load button
int loadButton = 0;
typedef enum {
	LOAD_BTN_IDLE, LOAD_BTN_DEBOUNCE, LOAD_BTN_HELD, LOAD_BTN_WAIT_RELEASE
} LoadButtonState;
static LoadButtonState loadBtnState = LOAD_BTN_IDLE;
static uint32_t loadDebounceStartTime = 0;
static const uint32_t LOAD_DEBOUNCE_DELAY = DEBOUNCE_DELAY; // Use the same delay constant

typedef enum {
	KEYPAD_IDLE, KEYPAD_DEBOUNCE, KEYPAD_HELD, KEYPAD_WAIT_RELEASE
} KeypadState;

KeypadState keypadState = KEYPAD_IDLE;

static uint32_t debounceStartTime = 0;     // When the debounce timing started
char detectedKey = 0;               // The key currently being debounced
volatile uint8_t keypadButton = 0; // Flag set by interrupt when any key is pressed
char currentKeypadInput = 0;       // The most recently confirmed pressed key
int keyHandled = 0;

// State machine states
typedef enum {
	STATE_DEFAULT_PAGE,
	STATE_DISPLAY_PAGE_1,
	STATE_DISPLAY_PAGE_2,
	STATE_DISPLAY_PAGE_3,
	STATE_MENU_LEVEL_1,
	STATE_LOAD_SWITCH_MENU,
	STATE_LOAD_SWITCH_MENU_ON,
	STATE_LOAD_SWITCH_MENU_OFF,
	STATE_MENU_LEVEL_2,
	STATE_UNIT_COUNT_MENU,
	STATE_UNIT_COUNT_MENU_ON,
	STATE_UNIT_COUNT_MENU_OFF,
	STATE_UNITS_ADD_MENU
} SystemState;

SystemState currentState = STATE_DEFAULT_PAGE;

uint32_t lastToggleTimeLEDUnitsLow = 0;
uint32_t lastToggleTimeLEDhighUsage = 0;
float wHTicker = 10.010;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */
void send_stat_response(void);
void send_student_num(void);
void start_listening_for_commands(void);
void set_RMS(int v_or_i);
void convert_ADC(void);
void loadButtonPressed(void);
void keypadButtonPressed(void);

void clearDisplay(void);
void writeTextLine(uint8_t row, const char *text);
void writeValueAtRow(uint8_t row, const char *text);
void updateDisplay(void);
void updatePowerMonitorState(void);
void initPowerMonitor(void);

void power_calcs(void);

void debugLEDWhTicker(void);
void debugLEDUnitsLow(void);
void debugLEDhighUsage(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

	send_student_num();

	initPowerMonitor();
	int lastUpdate = 0;

	start_listening_for_commands();

	HAL_TIM_Base_Start(&htim2); // Start timer with interrupts
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buffer, 200 * 2); // Start ADC

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		if (dma_complete) {
			convert_ADC();
			dma_complete = 0;
		}

		if (loadButton) {
			loadButtonPressed();
		}

		if (keypadButton == 1) {
			keypadButtonPressed();

			if (currentKeypadInput != 0){
				updatePowerMonitorState();
				currentKeypadInput = 0;
			}
		}

		if (HAL_GetTick() - lastUpdate > 500) {
			lastUpdate = HAL_GetTick();
			updateDisplay();
		}

		if (units_left < min_units){
			debugLEDUnitsLow();
		} else {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
			lastToggleTimeLEDUnitsLow = 0;
		}

		if (real_power > max_power){
			debugLEDhighUsage();
		} else {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
			lastToggleTimeLEDhighUsage = 0;
		}

		if (wHTicker - units_left >= 0.001){
			debugLEDWhTicker();
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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_12CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_12CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10B17DB5;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x22;
  sTime.Minutes = 0x12;
  sTime.Seconds = 0x42;
  sTime.SubSeconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  sDate.Month = RTC_MONTH_FEBRUARY;
  sDate.Date = 0x26;
  sDate.Year = 0x25;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 12799;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_9|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA9 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_9|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB10 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void send_student_num(void) {
	// Student number to be transmitted on power up
	HAL_Delay(150);
	uint8_t stu_num[] = "*25964917#\n";
	// Transmit student number
	HAL_UART_Transmit(&huart2, stu_num, 11, 100);
}

void start_listening_for_commands(void) {
	// Start listening for first byte
	HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_index], 1);

}

//UART receive interrupt callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		// Check for full command
		if (rx_index == 0 && rx_buffer[0] == '*') {
			rx_index++;
		} else if (rx_index > 0) {
			if (rx_buffer[rx_index] == '\n') {
				rx_buffer[rx_index] = '\0'; // Null terminate the string
				if (strcmp((char*) rx_buffer, "*Load#") == 0) {
					// Command to toggle load switch ON/OFF
					if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 1) {
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
					} else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 0
							&& units_left > 0) {
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
					}
				} else if (strcmp((char*) rx_buffer, "*Stat#") == 0) {
					// Command to send SB status info
					send_stat_response();
				}
				rx_index = 0; // Reset for next command
			} else if (rx_index < 49) {
				rx_index++; // increase
			} else {
				rx_index = 0; // Buffer overflow, reset for safety
			}
		}
		// Restart listening for next byte
		HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_index], 1);
	}

}

void send_stat_response(void) {

	// Get current date and time
	RTC_TimeTypeDef sTime = { 0 };
	RTC_DateTypeDef sDate = { 0 };
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	char SB_status_info[300];
	if (phase_diff < 0) {
		sprintf(SB_status_info, "20%02d/%02d/%02d %02d:%02d:%02d\n"
				"Voltage:     %05.1fV\n"
				"Current:    %05.0fmA\n"
				"Phase:    %05.3frad\n"
				"Apparent:    %04.0fVA\n"
				"Real:         %04.0fW\n"
				"Reactive:   %04.0fVAR\n"
				"PowerFact:    %05.3f\n"
				"Energy/d:   %05.0fWh\n"
				"MaxPower:     %04.0fW\n"
				"Units:   %07.3fkWh\n", sDate.Year, sDate.Month, sDate.Date,
				sTime.Hours, sTime.Minutes, sTime.Seconds, voltage, current,
				phase_diff, apparent, real_power, reactive_power, power_factor,
				energy, max_power, units_left);
	} else {
		sprintf(SB_status_info, "20%02d/%02d/%02d %02d:%02d:%02d\n"
				"Voltage:     %05.1fV\n"
				"Current:    %05.0fmA\n"
				"Phase:     %05.3frad\n"
				"Apparent:    %04.0fVA\n"
				"Real:         %04.0fW\n"
				"Reactive:   %04.0fVAR\n"
				"PowerFact:    %05.3f\n"
				"Energy/d:   %05.0fWh\n"
				"MaxPower:     %04.0fW\n"
				"Units:   %07.3fkWh\n", sDate.Year, sDate.Month, sDate.Date,
				sTime.Hours, sTime.Minutes, sTime.Seconds, voltage, current,
				phase_diff, apparent, real_power, reactive_power, power_factor,
				energy, max_power, units_left);
	}

	// Transmit SB stat info
	HAL_UART_Transmit(&huart2, SB_status_info, strlen(SB_status_info), 500);
}

void convert_ADC(void) {
	HAL_ADC_Stop_DMA(&hadc1);
	uint32_t adc_buffer_copy[BUFFER_SIZE];
	memcpy(adc_buffer_copy, (uint32_t*) adc_buffer,
			sizeof(uint32_t) * BUFFER_SIZE);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buffer, BUFFER_SIZE);

	for (int i = 0; i < BUFFER_SIZE / 2; i++) {
		voltage_buf[i] = (float) adc_buffer_copy[i * 2];
		current_buf[i] = (float) adc_buffer_copy[i * 2 + 1];
	}

	v_max = 0;
	v_min = 4095;
	i_max = 0;
	i_min = 4095;

	for (uint16_t i = 0; i < BUFFER_SIZE / 2; i++) {
		if (adc_buffer_copy[i * 2] > v_max) {
			v_max = adc_buffer_copy[i * 2];
		}
		if (adc_buffer_copy[i * 2] < v_min) {
			v_min = adc_buffer_copy[i * 2];
		}
		if (adc_buffer_copy[i * 2 + 1] > i_max) {
			i_max = adc_buffer_copy[i * 2 + 1];
		}
		if (adc_buffer_copy[i * 2 + 1] < i_min) {
			i_min = adc_buffer_copy[i * 2 + 1];
		}
	}

	set_RMS(0); // flag 0 for voltage
	set_RMS(1); // flag 1 for current
	power_calcs();

}

void set_RMS(int v_or_i) {
	if (v_or_i == 0) // voltage
			{
		float vpp = v_max - v_min;
		vpp = vpp * (3.3 / 4096);
		voltage = (vpp / OP_AMP_GAIN) * 115;

		//if (voltage < 2) {
		//	voltage = 0;
		//}
	} else if (v_or_i == 1) // current
			{
		float ipp = i_max - i_min;
		ipp = ipp * (3.3 / 4096);
		current = ((ipp / OP_AMP_GAIN) * 7.5) * 1000;
		//if (current < 100) {
		//	current = 0;
		//}
	}
	apparent = voltage * (current / 1000);
}

void power_calcs(void) {
	const float sr = 5000.0;
	float v_cross = 0;
	float c_cross = 0;

	float v_mid = v_min + (v_max - v_min) / 2;
	float i_mid = i_min + (i_max - i_min) / 2;

	for (int i = 1; i < BUFFER_SIZE; i++) {
		if (voltage_buf[i - 1] < v_mid && voltage_buf[i] >= v_mid) {
			v_cross = (i - 1)
					+ ((v_mid - voltage_buf[i - 1])
							/ (voltage_buf[i] - voltage_buf[i - 1]));
			break;
		}
	}

	for (int i = 1; i < BUFFER_SIZE; i++) {
		if (current_buf[i - 1] < i_mid && current_buf[i] >= i_mid) {
			c_cross = (i - 1)
					+ ((i_mid - current_buf[i - 1])
							/ (current_buf[i] - current_buf[i - 1]));
			break;
		}
	}

	float time_diff_phase = (c_cross - v_cross) / (sr);
	float phase_diff_unnorm = time_diff_phase * 50 * 2 * M_PI; //pos: current lags voltage, neg: voltage lags current

	if (phase_diff_unnorm > M_PI) {
		phase_diff_unnorm = phase_diff_unnorm - 2 * M_PI;
	} else if (phase_diff_unnorm < -M_PI) {
		phase_diff_unnorm = phase_diff_unnorm + 2 * M_PI;
	}

	phase_diff = phase_diff_unnorm;

	real_power = voltage * (current / 1000) * cos(phase_diff);
	reactive_power = voltage * (current / 1000) * sin(phase_diff);
	power_factor = cos(phase_diff);

	if (count_units == 1){
		units_left = units_left - (real_power * 0.04)/3600000;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		dma_complete = 1;
	}
}

void loadButtonPressed(void) {
	uint32_t currentTime = HAL_GetTick();
	uint8_t btnState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4); // Current state of the button (1 for pressed)

	switch (loadBtnState) {
	case LOAD_BTN_IDLE:
		if (btnState == 1) {
			loadDebounceStartTime = currentTime;
			loadBtnState = LOAD_BTN_DEBOUNCE;
		}
		break;
	case LOAD_BTN_DEBOUNCE:
		if ((currentTime - loadDebounceStartTime) > LOAD_DEBOUNCE_DELAY) {
			if (btnState == 1) {
				// Button is still pressed after debounce period
				loadBtnState = LOAD_BTN_HELD;
				// Execute the load toggle action
				if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 1) {
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
				} else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 0
						&& units_left > 0) {
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
				}
			} else {
				// Button was released during debounce period
				loadBtnState = LOAD_BTN_IDLE;
			}
		}
		break;
	case LOAD_BTN_HELD:

		if (btnState == 0) {
			loadBtnState = LOAD_BTN_WAIT_RELEASE;
			break;
			case LOAD_BTN_WAIT_RELEASE:
			// Make sure button is fully released before going back to idle
			if (btnState == 0) {
				loadBtnState = LOAD_BTN_IDLE;
				loadButton = 0; // Reset the button flag if you're using it
			}
			break;
		}
	}
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_4) {
		loadButton = 1;
	} else if (GPIO_Pin == GPIO_PIN_1 || GPIO_Pin == GPIO_PIN_2
			|| GPIO_Pin == GPIO_PIN_12) {
		keypadButton = 1;

	}
}

void keypadButtonPressed(void) {
	uint32_t currentTime = HAL_GetTick();
	char key = 0;

	// Set all rows low initially
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);  // Row 1
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);  // Row 2
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);  // Row 3
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);   // Row 4

	// Small delay to let pins settle
	HAL_Delay(1);

	// Scan Row 1
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 1) {
		key = '1';
	} else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == 1) {
		key = '2';
	} else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 1) {
		key = '3';
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);

	// Scan Row 2
	if (key == 0) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 1) {
			key = '4';
		} else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == 1) {
			key = '5';
		} else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 1) {
			key = '6';
		}
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
	}

	// Scan Row 3
	if (key == 0) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 1) {
			key = '7';
		} else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == 1) {
			key = '8';
		} else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 1) {
			key = '9';
		}
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	}

	// Scan Row 4
	if (key == 0) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 1) {
			key = '*';
		} else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == 1) {
			key = '0';
		} else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 1) {
			key = '#';
		}
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
	}

	// Debounce logic
	switch (keypadState) {
	case KEYPAD_IDLE:
		if (key != 0) {
			detectedKey = key;
			debounceStartTime = currentTime;
			keypadState = KEYPAD_DEBOUNCE;
		}
		break;

	case KEYPAD_DEBOUNCE:
		if ((currentTime - debounceStartTime) > DEBOUNCE_DELAY) {
			if (key == detectedKey) {
				currentKeypadInput = key;
				keypadState = KEYPAD_HELD;
			} else {
				keypadState = KEYPAD_IDLE;
			}
		}
		break;

	case KEYPAD_HELD:
		if (key != detectedKey) {
			keypadState = KEYPAD_WAIT_RELEASE;
		}
		break;

	case KEYPAD_WAIT_RELEASE:
		if (key == 0) {
			keypadState = KEYPAD_IDLE;
			keypadButton = 0;
		}
		break;
	}

	// Re-enable interrupt capability by setting all rows high
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);

}

void clearDisplay(void) {
	ssd1306_Fill(BG_COLOR);
}

// write a line of text at a specific row
void writeTextLine(uint8_t row, const char *text) {
	ssd1306_SetCursor(LEFT_MARGIN, row * LINE_HEIGHT);
	ssd1306_WriteString((char*) text, FONT, TEXT_COLOR);
}

// write a value at a specific row
void writeValueAtRow(uint8_t row, const char *text) {
	ssd1306_SetCursor(VALUE_COLUMN, row * LINE_HEIGHT);
	ssd1306_WriteString((char*) text, FONT, TEXT_COLOR);
}

// Update display based on current state
void updateDisplay(void) {
	char str[32];
	clearDisplay();

	// Get current date and
	RTC_TimeTypeDef sTime = { 0 };
	RTC_DateTypeDef sDate = { 0 };
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	switch (currentState) {
	case STATE_DEFAULT_PAGE:
		writeTextLine(0, "Apparent:");
		sprintf(str, "    %04.0fVA", apparent);
		writeValueAtRow(0, str);

		writeTextLine(1, "Energy/d:");
		sprintf(str, "   %05.0fWh", energy);
		writeValueAtRow(1, str);

		writeTextLine(2, "Units:");
		sprintf(str, "%07.3fkWh", units_left);
		writeValueAtRow(2, str);
		break;

	case STATE_DISPLAY_PAGE_1:
		writeTextLine(0, "Voltage:");
		sprintf(str, "  %05.1fV", voltage);
		writeValueAtRow(0, str);

		writeTextLine(1, "Current:");
		sprintf(str, " %05.0fmA", current);
		writeValueAtRow(1, str);

		writeTextLine(2, "Phase:");
		sprintf(str, "%05.3fRad", phase_diff);
		writeValueAtRow(2, str);
		break;

	case STATE_DISPLAY_PAGE_2:
		writeTextLine(0, "Real:");
		sprintf(str, "  %04.0fW", real_power);
		writeValueAtRow(0, str);

		writeTextLine(1, "Reactive:");
		sprintf(str, "%04.0fVAR", reactive_power);
		writeValueAtRow(1, str);

		writeTextLine(2, "PowerF:");
		sprintf(str, "  %04.3f", power_factor);
		writeValueAtRow(2, str);
		break;

	case STATE_DISPLAY_PAGE_3:
		sprintf(str, "%02d:%02d:%02d %2d/%02d/%02d", sTime.Hours, sTime.Minutes,
				sTime.Seconds, sDate.Year, sDate.Month, sDate.Date);
		writeTextLine(0, str);

		writeTextLine(1, "Max power:  ");
		sprintf(str, "%04.0fW", max_power);
		writeValueAtRow(1, str);

		writeTextLine(2, "Min Units:");
		sprintf(str, "%04.0fkWh", min_units);
		writeValueAtRow(2, str);
		break;

	case STATE_MENU_LEVEL_1:
		writeTextLine(0, "Menu(Lv1 1):");
		writeTextLine(1, "1: Load On/Off");
		writeTextLine(2, "2: Manage Units");
		break;

	case STATE_LOAD_SWITCH_MENU:
		writeTextLine(0, "Load on/off");
		writeTextLine(1, "1=ON    2=OFF");
		writeTextLine(2, "'*' to confirm");
		break;

	case STATE_LOAD_SWITCH_MENU_ON:
		writeTextLine(0, "Load on/off");
		writeTextLine(1, "1=ON#    2=OFF");
		writeTextLine(2, "'*' to confirm");
		break;

	case STATE_LOAD_SWITCH_MENU_OFF:
		writeTextLine(0, "Load on/off");
		writeTextLine(1, "1=ON    2=OFF#");
		writeTextLine(2, "'*' to confirm");
		break;

	case STATE_MENU_LEVEL_2:
		writeTextLine(0, "Menu(Lv1 2):");
		writeTextLine(1, "1: Count On/Off");
		writeTextLine(2, "2: Add Units");
		break;

	case STATE_UNIT_COUNT_MENU:
		writeTextLine(0, "Unit Count");
		writeTextLine(1, "1=ON    2=OFF");
		writeTextLine(2, "'*' to confirm");
		break;

	case STATE_UNIT_COUNT_MENU_ON:
		writeTextLine(0, "Unit Count");
		writeTextLine(1, "1=ON#    2=OFF");
		writeTextLine(2, "'*' to confirm");
		break;

	case STATE_UNIT_COUNT_MENU_OFF:
		writeTextLine(0, "Unit Count");
		writeTextLine(1, "1=ON    2=OFF#");
		writeTextLine(2, "'*' to confirm");
		break;

	case STATE_UNITS_ADD_MENU:
		writeTextLine(0, "Units to add:");
		char unitsStr[12];
		sprintf(unitsStr, "%d", unit_add);
		writeTextLine(1, unitsStr);  // Value would be updated as user enters
		writeTextLine(2, "Press '*' to add");
		break;
	}

	// Update the physical display
	ssd1306_UpdateScreen();
}

// Update the state based on keypad input
void updatePowerMonitorState(void) {
	// Process state changes based on current state and input
	switch (currentState) {
	case STATE_DEFAULT_PAGE:
	case STATE_DISPLAY_PAGE_1:
	case STATE_DISPLAY_PAGE_2:
	case STATE_DISPLAY_PAGE_3:
		if (currentKeypadInput == '0') {
			currentState = STATE_DEFAULT_PAGE;
		} else if (currentKeypadInput == '1') {
			currentState = STATE_DISPLAY_PAGE_1;
		} else if (currentKeypadInput == '2') {
			currentState = STATE_DISPLAY_PAGE_2;
		} else if (currentKeypadInput == '3') {
			currentState = STATE_DISPLAY_PAGE_3;
		} else if (currentKeypadInput == '*') {
			currentState = STATE_MENU_LEVEL_1;
		}
		break;

	case STATE_MENU_LEVEL_1:
		if (currentKeypadInput == '1') {
			currentState = STATE_LOAD_SWITCH_MENU;
		} else if (currentKeypadInput == '2') {
			currentState = STATE_MENU_LEVEL_2;
		} else if (currentKeypadInput == '#') {
			currentState = STATE_DEFAULT_PAGE;
		}
		break;

	case STATE_LOAD_SWITCH_MENU:
		if (currentKeypadInput == '1') {
			currentState = STATE_LOAD_SWITCH_MENU_ON;
		} else if (currentKeypadInput == '2') {
			// User selected OFF
			currentState = STATE_LOAD_SWITCH_MENU_OFF;
		} else if (currentKeypadInput == '*') {
			// Confirm setting and return to default page
			currentState = STATE_DEFAULT_PAGE;
		} else if (currentKeypadInput == '#') {
			// Cancel and go back to previous menu
			currentState = STATE_MENU_LEVEL_1;
		}
		break;

	case STATE_LOAD_SWITCH_MENU_ON:
		//user selected ON
		if (currentKeypadInput == '2') {
			currentState = STATE_LOAD_SWITCH_MENU_OFF;
		} else if (currentKeypadInput == '*') {
			// Confirm setting and return to default page
			if (units_left > 0) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
			}
			currentState = STATE_DEFAULT_PAGE;
		} else if (currentKeypadInput == '#') {
			// Cancel and go back to previous menu
			currentState = STATE_MENU_LEVEL_1;
		}
		break;

	case STATE_LOAD_SWITCH_MENU_OFF:
		//user selected OFF
		if (currentKeypadInput == '1') {
			currentState = STATE_LOAD_SWITCH_MENU_ON;
		} else if (currentKeypadInput == '*') {
			// Confirm setting and return to default page
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
			currentState = STATE_DEFAULT_PAGE;
		} else if (currentKeypadInput == '#') {
			// Cancel and go back to previous menu
			currentState = STATE_MENU_LEVEL_1;
		}
		break;

	case STATE_MENU_LEVEL_2:
		if (currentKeypadInput == '1') {
			currentState = STATE_UNIT_COUNT_MENU;
		} else if (currentKeypadInput == '2') {
			currentState = STATE_UNITS_ADD_MENU;
		} else if (currentKeypadInput == '#') {
			currentState = STATE_MENU_LEVEL_1;
		}
		break;

	case STATE_UNIT_COUNT_MENU:
		if (currentKeypadInput == '1') {
			currentState = STATE_UNIT_COUNT_MENU_ON;
		} else if (currentKeypadInput == '2') {
			currentState = STATE_UNIT_COUNT_MENU_OFF;
		} else if (currentKeypadInput == '*') {
			// Confirm setting and return to default page
			currentState = STATE_DEFAULT_PAGE;
		} else if (currentKeypadInput == '#') {
			// Cancel and go back to previous menu
			currentState = STATE_MENU_LEVEL_2;
		}
		break;

	case STATE_UNIT_COUNT_MENU_ON:
		if (currentKeypadInput == '2') {
			currentState = STATE_UNIT_COUNT_MENU_OFF;
		} else if (currentKeypadInput == '*') {
			// Confirm setting and return to default page - NEED TO ADD THIS FUNCTIONALITY
			count_units = 1;
			currentState = STATE_DEFAULT_PAGE;
		} else if (currentKeypadInput == '#') {
			// Cancel and go back to previous menu
			currentState = STATE_MENU_LEVEL_2;
		}
		break;

	case STATE_UNIT_COUNT_MENU_OFF:
		if (currentKeypadInput == '1') {
			currentState = STATE_UNIT_COUNT_MENU_ON;
		} else if (currentKeypadInput == '*') {
			// Confirm setting and return to default page - NEED TO ADD THIS FUNCTIONALITY
			count_units = 0;
			currentState = STATE_DEFAULT_PAGE;
		} else if (currentKeypadInput == '#') {
			// Cancel and go back to previous menu
			currentState = STATE_MENU_LEVEL_2;
		}
		break;

	case STATE_UNITS_ADD_MENU:
		// Handle numeric input for units value
		if ((currentKeypadInput - '0') >= 0
				&& (currentKeypadInput - '0') <= 9) {
			unit_add = unit_add * 10 + (currentKeypadInput - '0');
			if (unit_add >= 999.9) {
				unit_add = 999.9;
			}
			if (unit_add + units_left >= 999.9) {
				unit_add = (999.9 - units_left);
			}
		} else if (currentKeypadInput == '*') {
			units_left = unit_add + units_left;
			wHTicker= units_left;
			unit_add = 0;
			currentState = STATE_DEFAULT_PAGE;
		} else if (currentKeypadInput == '#') {
			// Cancel and go back to previous menu
			currentState = STATE_MENU_LEVEL_2;
			unit_add = 0;
		}
		break;
	}

	// Update the display based on current state
	updateDisplay();
}

// Initialization function to setup the display and state machine
void initPowerMonitor(void) {
	// Initialize the SSD1306 OLED display
	ssd1306_Init();

	// Set initial state
	currentState = STATE_DEFAULT_PAGE;

	// Display the initial screen
	updateDisplay();
}

void debugLEDWhTicker(void){
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	wHTicker = units_left;
}
void debugLEDUnitsLow(void){

	uint32_t currentTime = HAL_GetTick();

	if (currentTime - lastToggleTimeLEDUnitsLow >= 500){
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
		lastToggleTimeLEDUnitsLow = currentTime;
	}

}
void debugLEDhighUsage(void){

	uint32_t currentTime = HAL_GetTick();

		if (currentTime - lastToggleTimeLEDhighUsage >= 500){
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
			lastToggleTimeLEDhighUsage = currentTime;
		}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
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
