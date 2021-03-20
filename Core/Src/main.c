/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* -----	build types	-----*/

//#define USED_STM32F091
//#define USED_STM32H743
#define USED_STM32F407

//#define COIL_8	// old version
#define COIL_40	// update version

#define BOARD_PROG_BTN_LOW	 0
#define BOARD_PROG_BTN_HIGH	 1

/* -----	build types	-----*/

#if defined(STM32F071xB)
// 64 kByte max size flash
#define START_FLASH_ADDRESS_PAGE_127	0x0800F800
#define END_FLASH_ADDRESS_PAGE_127 		0x0800FFFF
#elif defined(USED_STM32F407)
#define START_FLASH_ADDRESS_SECTOR_6	0x08040000
#else
#define START_FLASH_ADDRESS_PAGE_127	0x0803F800
#define END_FLASH_ADDRESS_PAGE_127 		0x0803FFFF
#endif


#define DEFAULT_SLAVE_ADDR 3

enum { STATE_NORMAL, STATE_PRG };
enum { OPEN_LOCK=1, EXTINGUISH_LED, FIRE_LED} ;

uint8_t state = STATE_NORMAL;
uint8_t slave_addr = DEFAULT_SLAVE_ADDR; // ..until not read from eeprom
uint8_t cntr = 0x10;
uint8_t rx1_data;
uint8_t rx2_data;
char buf[128];

#define TX_BUF_SIZE 10
#define RX_BUF_SIZE 8
#define TXRXBUF_SZ 8
#define MESSAGE_SZ 8
#define NUMBER_DATA 6
//uint16_t rx1buf_fullness = 0;

#define QUEUE_SZ 42

#if defined(USED_STM32F091) || defined(USED_STM32F407)
#define NUMBER_COILS	(uint8_t)40

GPIO_TypeDef * ports_in_mapping[NUMBER_COILS] = {	
	IN_1_GPIO_Port,	IN_2_GPIO_Port, IN_3_GPIO_Port, IN_4_GPIO_Port, IN_5_GPIO_Port, IN_6_GPIO_Port, IN_7_GPIO_Port, IN_8_GPIO_Port, IN_9_GPIO_Port, IN_10_GPIO_Port,
	IN_11_GPIO_Port, IN_12_GPIO_Port, IN_13_GPIO_Port, IN_14_GPIO_Port, IN_15_GPIO_Port, IN_16_GPIO_Port, IN_17_GPIO_Port, IN_18_GPIO_Port, IN_19_GPIO_Port, IN_20_GPIO_Port,
	IN_21_GPIO_Port, IN_22_GPIO_Port, IN_23_GPIO_Port, IN_24_GPIO_Port, IN_25_GPIO_Port, IN_26_GPIO_Port, IN_27_GPIO_Port, IN_28_GPIO_Port, IN_29_GPIO_Port, IN_30_GPIO_Port,
	IN_31_GPIO_Port, IN_32_GPIO_Port, IN_33_GPIO_Port, IN_34_GPIO_Port, IN_35_GPIO_Port, IN_36_GPIO_Port, IN_37_GPIO_Port, IN_38_GPIO_Port, IN_39_GPIO_Port, IN_40_GPIO_Port
};

GPIO_TypeDef * ports_out_mapping[NUMBER_COILS] = {	
	OUT_1_GPIO_Port,	OUT_2_GPIO_Port, OUT_3_GPIO_Port, OUT_4_GPIO_Port, OUT_5_GPIO_Port, OUT_6_GPIO_Port, OUT_7_GPIO_Port, OUT_8_GPIO_Port, OUT_9_GPIO_Port, OUT_10_GPIO_Port,
	OUT_11_GPIO_Port, OUT_12_GPIO_Port, OUT_13_GPIO_Port, OUT_14_GPIO_Port, OUT_15_GPIO_Port, OUT_16_GPIO_Port, OUT_17_GPIO_Port, OUT_18_GPIO_Port, OUT_19_GPIO_Port, OUT_20_GPIO_Port,
	OUT_21_GPIO_Port, OUT_22_GPIO_Port, OUT_23_GPIO_Port, OUT_24_GPIO_Port, OUT_25_GPIO_Port, OUT_26_GPIO_Port, OUT_27_GPIO_Port, OUT_28_GPIO_Port, OUT_29_GPIO_Port, OUT_30_GPIO_Port,
	OUT_31_GPIO_Port, OUT_32_GPIO_Port, OUT_33_GPIO_Port, OUT_34_GPIO_Port, OUT_35_GPIO_Port, OUT_36_GPIO_Port, OUT_37_GPIO_Port, OUT_38_GPIO_Port, OUT_39_GPIO_Port, OUT_40_GPIO_Port
};

const uint16_t number_ports_in[NUMBER_COILS] = {	
	IN_1_Pin,	IN_2_Pin,	IN_3_Pin,	IN_4_Pin,	IN_5_Pin,	IN_6_Pin,	IN_7_Pin,	IN_8_Pin,	IN_9_Pin,	IN_10_Pin,
	IN_11_Pin, IN_12_Pin, IN_13_Pin, IN_14_Pin, IN_15_Pin, IN_16_Pin, IN_17_Pin, IN_18_Pin, IN_19_Pin, IN_20_Pin,
	IN_21_Pin, IN_22_Pin, IN_23_Pin, IN_24_Pin, IN_25_Pin, IN_26_Pin, IN_27_Pin, IN_28_Pin, IN_29_Pin, IN_30_Pin,
	IN_31_Pin, IN_32_Pin, IN_33_Pin, IN_34_Pin, IN_35_Pin, IN_36_Pin, IN_37_Pin, IN_38_Pin, IN_39_Pin, IN_40_Pin
};

const uint16_t number_ports_out[NUMBER_COILS] = { 
	OUT_1_Pin,	OUT_2_Pin,	OUT_3_Pin,	OUT_4_Pin,	OUT_5_Pin,	OUT_6_Pin,	OUT_7_Pin,	OUT_8_Pin,	OUT_9_Pin,	OUT_10_Pin,
	OUT_11_Pin, OUT_12_Pin, OUT_13_Pin, OUT_14_Pin, OUT_15_Pin, OUT_16_Pin, OUT_17_Pin, OUT_18_Pin, OUT_19_Pin, OUT_20_Pin,
	OUT_21_Pin, OUT_22_Pin, OUT_23_Pin, OUT_24_Pin, OUT_25_Pin, OUT_26_Pin, OUT_27_Pin, OUT_28_Pin, OUT_29_Pin, OUT_30_Pin,
	OUT_31_Pin, OUT_32_Pin, OUT_33_Pin, OUT_34_Pin, OUT_35_Pin, OUT_36_Pin, OUT_37_Pin, OUT_38_Pin, OUT_39_Pin, OUT_40_Pin
};
#endif
#ifdef USED_STM32H743
#define NUMBER_COILS	(uint8_t)5

GPIO_TypeDef * ports_in_mapping[NUMBER_COILS] = {	
	IN_1_GPIO_Port,	IN_2_GPIO_Port, IN_3_GPIO_Port, IN_4_GPIO_Port, IN_5_GPIO_Port
};

GPIO_TypeDef * ports_out_mapping[NUMBER_COILS] = {	
	OUT_1_GPIO_Port,	OUT_2_GPIO_Port, OUT_3_GPIO_Port, OUT_4_GPIO_Port, OUT_5_GPIO_Port
};

const uint16_t number_ports_in[NUMBER_COILS] = {	
	IN_1_Pin,	IN_2_Pin,	IN_3_Pin,	IN_4_Pin,	IN_5_Pin
};

const uint16_t number_ports_out[NUMBER_COILS] = { 
	OUT_1_Pin,	OUT_2_Pin,	OUT_3_Pin,	OUT_4_Pin,	OUT_5_Pin
};
#endif

uint32_t queue[QUEUE_SZ];
uint8_t queue_fullness = 0;
uint32_t *p_queue_in = queue;
uint32_t *p_queue_out = queue;

uint32_t curr_task = 0;
uint16_t curr_task_time_cntr = 0;

uint16_t pb8_to_pb15_outputs_prepare_cntr = 0;

#define DEFAUL_TIME_MS 	(uint16_t)31
#define ADDITIONAL_SECOND_TO_OPEN_COIL	(uint16_t)186

uint16_t open_delay = DEFAUL_TIME_MS;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

void led_on (uint16_t lednum);
void led_off (uint16_t lednum);

void rs485_write_ena (void);
void rs485_write_dis (void);

void switch_off_all_coils (void);

void send_to_rs485(uint8_t *data, uint8_t sz);

uint32_t get_next_task_from_queue(void);
void push_task_to_queue(uint32_t task);
void blinking (void);
void reconfigure_pb8_to_pb15_for_output();
void short_pb_to_gnd(uint16_t gpio_pins);
void configure_pb_for_input(uint16_t gpio_pins);
void spin_once(void);
void write_port (uint8_t number_port);
void read_port_input (uint8_t * arr, uint8_t size_arr);
void read_port_output (uint8_t * arr, uint8_t size_arr);
uint8_t get_odr_reg (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define SET_LOW 	0
#define SET_HIGH 	1

const uint16_t morze_code_output[MORZE_SIZE] = {
	 SET_HIGH,	SET_HIGH,	SET_HIGH,	SET_LOW,  SET_HIGH,	SET_HIGH,	SET_HIGH, SET_LOW,  SET_HIGH, SET_HIGH, SET_HIGH, SET_LOW,  SET_HIGH, SET_HIGH, SET_HIGH, SET_LOW,  SET_HIGH, SET_HIGH,  SET_HIGH, SET_LOW,  // code "0" (20 samples)
	 SET_HIGH,  SET_LOW,	SET_HIGH,	SET_HIGH,	SET_HIGH,	SET_LOW,	SET_HIGH,	SET_HIGH, SET_HIGH, SET_LOW,  SET_HIGH, SET_HIGH, SET_HIGH, SET_LOW,  SET_HIGH, SET_HIGH, SET_HIGH, SET_LOW,  // "1" (18 samples)
	 SET_HIGH,	SET_LOW,	SET_HIGH,	SET_LOW,	SET_HIGH,	SET_HIGH,	SET_HIGH, SET_LOW,  SET_HIGH, SET_HIGH, SET_HIGH, SET_LOW,  SET_HIGH, SET_HIGH, SET_HIGH, SET_LOW,  // "2" (16 samples)
	 SET_HIGH,  SET_LOW,  SET_HIGH,	SET_HIGH,	SET_HIGH, SET_LOW,  SET_HIGH, SET_HIGH, SET_HIGH, SET_LOW,  SET_HIGH, SET_HIGH, SET_HIGH, SET_LOW,  SET_HIGH, SET_HIGH,  SET_HIGH, SET_LOW,  // "1" (18 samples)
	SET_LOW, SET_LOW, SET_LOW, SET_LOW, SET_LOW, // pause between morze and address
};

extern volatile uint16_t sys_ms;

typedef struct __attribute__((packed)) _morze {
	volatile uint8_t stage_global;
	volatile uint8_t stage_array;
	uint8_t flag_first_push;
}
morze_code;

morze_code firm_code;

uint16_t cur_ms = 0;
uint16_t prev_ms = 0;
uint16_t calc_time = 0;

uint8_t state_led = 0;

#define BLINK	0
#define MORZE	1

#define DELAY_LED_MS	30000
volatile uint8_t rx1buf[RX_BUF_SIZE];
volatile uint8_t tx1buf[TX_BUF_SIZE];

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  uint8_t tmp_slave_addr;
	
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
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

	tmp_slave_addr = *(uint8_t *)START_FLASH_ADDRESS_SECTOR_6;
	
  if ((tmp_slave_addr != 0xff) && (tmp_slave_addr != 0x00))
  {
    slave_addr = tmp_slave_addr;
  }
	
	HAL_TIM_Base_Start_IT (&htim3);
  HAL_UART_Receive_IT (&huart2, &rx1_data, 1);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 7999;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 300;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OUT_31_Pin|OUT_30_Pin|OUT_29_Pin|OUT_28_Pin
                          |OUT_27_Pin|OUT_22_Pin|OUT_21_Pin|OUT_20_Pin
                          |OUT_40_Pin|OUT_39_Pin|OUT_38_Pin|OUT_37_Pin
                          |OUT_19_Pin|OUT_18_Pin|OUT_9_Pin|OUT_8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, OUT_7_Pin|OUT_6_Pin|OUT_5_Pin|OUT_26_Pin
                          |OUT_25_Pin|OUT_24_Pin|OUT_23_Pin|OUT_2_Pin
                          |OUT_1_Pin|OUT_36_Pin|OUT_35_Pin|OUT_17_Pin
                          |OUT_16_Pin|OUT_12_Pin|OUT_11_Pin|OUT_10_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, OUT_4_Pin|OUT_3_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RS_DIR_Pin|OUT_15_Pin|OUT_14_Pin|OUT_32_Pin
                          |OUT_33_Pin|OUT_34_Pin|OUT_13_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OUT_31_Pin OUT_30_Pin OUT_29_Pin OUT_28_Pin
                           OUT_27_Pin OUT_22_Pin OUT_21_Pin OUT_20_Pin
                           OUT_40_Pin OUT_39_Pin OUT_38_Pin OUT_37_Pin
                           OUT_19_Pin OUT_18_Pin OUT_9_Pin OUT_8_Pin */
  GPIO_InitStruct.Pin = OUT_31_Pin|OUT_30_Pin|OUT_29_Pin|OUT_28_Pin
                          |OUT_27_Pin|OUT_22_Pin|OUT_21_Pin|OUT_20_Pin
                          |OUT_40_Pin|OUT_39_Pin|OUT_38_Pin|OUT_37_Pin
                          |OUT_19_Pin|OUT_18_Pin|OUT_9_Pin|OUT_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT_7_Pin OUT_6_Pin OUT_5_Pin OUT_26_Pin
                           OUT_25_Pin OUT_24_Pin OUT_23_Pin OUT_2_Pin
                           OUT_1_Pin OUT_36_Pin OUT_35_Pin OUT_17_Pin
                           OUT_16_Pin OUT_12_Pin OUT_11_Pin OUT_10_Pin */
  GPIO_InitStruct.Pin = OUT_7_Pin|OUT_6_Pin|OUT_5_Pin|OUT_26_Pin
                          |OUT_25_Pin|OUT_24_Pin|OUT_23_Pin|OUT_2_Pin
                          |OUT_1_Pin|OUT_36_Pin|OUT_35_Pin|OUT_17_Pin
                          |OUT_16_Pin|OUT_12_Pin|OUT_11_Pin|OUT_10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT_4_Pin OUT_3_Pin LED_Pin */
  GPIO_InitStruct.Pin = OUT_4_Pin|OUT_3_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : IN_25_Pin IN_24_Pin IN_27_Pin IN_26_Pin */
  GPIO_InitStruct.Pin = IN_25_Pin|IN_24_Pin|IN_27_Pin|IN_26_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : IN_23_Pin IN_PROG_Pin IN_1_Pin IN_2_Pin
                           IN_3_Pin */
  GPIO_InitStruct.Pin = IN_23_Pin|IN_PROG_Pin|IN_1_Pin|IN_2_Pin
                          |IN_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RS_DIR_Pin OUT_15_Pin OUT_14_Pin OUT_32_Pin
                           OUT_33_Pin OUT_34_Pin OUT_13_Pin */
  GPIO_InitStruct.Pin = RS_DIR_Pin|OUT_15_Pin|OUT_14_Pin|OUT_32_Pin
                          |OUT_33_Pin|OUT_34_Pin|OUT_13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IN_4_Pin IN_5_Pin IN_6_Pin IN_39_Pin
                           IN_40_Pin IN_22_Pin IN_21_Pin IN_20_Pin
                           IN_38_Pin IN_30_Pin IN_10_Pin IN_9_Pin
                           IN_8_Pin IN_7_Pin IN_29_Pin IN_28_Pin */
  GPIO_InitStruct.Pin = IN_4_Pin|IN_5_Pin|IN_6_Pin|IN_39_Pin
                          |IN_40_Pin|IN_22_Pin|IN_21_Pin|IN_20_Pin
                          |IN_38_Pin|IN_30_Pin|IN_10_Pin|IN_9_Pin
                          |IN_8_Pin|IN_7_Pin|IN_29_Pin|IN_28_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IN_19_Pin IN_37_Pin IN_18_Pin IN_36_Pin
                           IN_17_Pin IN_35_Pin IN_16_Pin IN_15_Pin
                           IN_34_Pin IN_14_Pin IN_33_Pin IN_13_Pin
                           IN_32_Pin IN_12_Pin IN_31_Pin IN_11_Pin */
  GPIO_InitStruct.Pin = IN_19_Pin|IN_37_Pin|IN_18_Pin|IN_36_Pin
                          |IN_17_Pin|IN_35_Pin|IN_16_Pin|IN_15_Pin
                          |IN_34_Pin|IN_14_Pin|IN_33_Pin|IN_13_Pin
                          |IN_32_Pin|IN_12_Pin|IN_31_Pin|IN_11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void reconfigure_pb8_to_pb15_for_output (void) {
//	  GPIO_InitTypeDef GPIO_InitStruct;
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
//    |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);
//  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
//    |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;//@@@GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void spin_once()
{
  // spin_once for 5ms loop (timer-3)
  if (curr_task_time_cntr)
  {
    curr_task_time_cntr--;
  }
  else
  {
    if (((0x0000FFFF & curr_task) >> 8) == OPEN_LOCK)
    {
      switch_off_all_coils();
			
//			open_delay = DEFAUL_TIME_MS;
			
//			HAL_GPIO_WritePin(GPIOC, curr_task & 0xff, GPIO_PIN_SET);

//			// get back to last task if device has many outputs for enable
//			curr_task_time_cntr = (155/5);
//			queue_fullness++;
//			p_queue_out--;
    }

    // reset curr_task
    curr_task = 0;

    if (queue_fullness)
    {
      // get next task
      curr_task = get_next_task_from_queue();
      if (curr_task)
      {
        if (((0x0000FFFF & curr_task) >> 8) == OPEN_LOCK)
        {
					write_port ((uint16_t)(0x0000FFFF & curr_task) & 0x3f);
//          HAL_GPIO_WritePin(GPIOC, curr_task & 0xff, GPIO_PIN_SET);

          // start counter for the task
//          curr_task_time_cntr = (155/5);
					curr_task_time_cntr = (uint16_t)((0xFFFF0000 & curr_task) >> 16);
        }
        else if ((curr_task>>8) == FIRE_LED)
        {
          if (pb8_to_pb15_outputs_prepare_cntr > 64)
          {
            HAL_GPIO_WritePin(GPIOB, (curr_task & 0xff)<<8, GPIO_PIN_RESET);
            //@@@short_pb_to_gnd((curr_task & 0xff)<<8);
          }
          else if (pb8_to_pb15_outputs_prepare_cntr == 64)
          {
            pb8_to_pb15_outputs_prepare_cntr = 1000;
            reconfigure_pb8_to_pb15_for_output();
            //@@@pb_output_ena = 1;
          }
          else
            pb8_to_pb15_outputs_prepare_cntr++;
        }
        else if ((curr_task>>8) == EXTINGUISH_LED)
        {
          if (pb8_to_pb15_outputs_prepare_cntr > 64)
          {
            HAL_GPIO_WritePin(GPIOB, (curr_task & 0xff)<<8, GPIO_PIN_SET);
            //@@@configure_pb_for_input((curr_task & 0xff)<<8);
          }
          else if (pb8_to_pb15_outputs_prepare_cntr == 64)
          {
            pb8_to_pb15_outputs_prepare_cntr = 1000;
            reconfigure_pb8_to_pb15_for_output();
            //@@@pb_output_ena = 1;
          }
          else
            pb8_to_pb15_outputs_prepare_cntr++;
        }
      }
    }
  }
}

uint16_t calc_crc(uint8_t *data, uint8_t sz)
{
  uint16_t crc = 0xffff;
  uint16_t tmp;
  uint8_t i, j;
  for (i = 0; i < sz; i++)
  {
    crc = crc ^ *(data + i);
    for (j = 0; j < 8; j++)
    {
      tmp = crc & 1;
      crc >>= 1;
      if (tmp) crc ^= 0xA001;
    }
  }
  return crc;
}

void led_on (uint16_t lednum)
{
#ifdef USED_STM32F091
  HAL_GPIO_WritePin (GPIOF, lednum, GPIO_PIN_SET);
#endif
#ifdef USED_STM32F407
  HAL_GPIO_WritePin (GPIOF, lednum, GPIO_PIN_SET);
#endif
}

void led_off (uint16_t lednum)
{
#ifdef USED_STM32F091
  HAL_GPIO_WritePin (GPIOF, lednum, GPIO_PIN_RESET);
#endif
#ifdef USED_STM32F407
  HAL_GPIO_WritePin (GPIOF, lednum, GPIO_PIN_RESET);
#endif
}

void rs485_write_ena ()
{
#ifdef USED_STM32F091
  HAL_GPIO_WritePin (GPIOA, RS_DIR_Pin, GPIO_PIN_SET);  // set rs485 write-mode
#endif
#ifdef USED_STM32F407
  HAL_GPIO_WritePin (GPIOA, RS_DIR_Pin, GPIO_PIN_SET);  // set rs485 write-mode
#endif
}

void rs485_write_dis ()
{
#ifdef USED_STM32F091
  HAL_GPIO_WritePin (GPIOA, RS_DIR_Pin, GPIO_PIN_RESET);  // set rs485 read-mode
#endif
#ifdef USED_STM32F407
  HAL_GPIO_WritePin (GPIOA, RS_DIR_Pin, GPIO_PIN_RESET);  // set rs485 read-mode
#endif
}

void send_to_rs485(uint8_t *data, uint8_t sz)
{
  rs485_write_ena ();
  HAL_UART_Transmit_IT (&huart2, data, sz);
}

void rx1_accum()
{
  uint8_t *pckt;
	
  for (uint8_t i = 0; i < RX_BUF_SIZE - 1; i++)
    rx1buf[i] = rx1buf[i+1];
	
  rx1buf[RX_BUF_SIZE-1] = rx1_data;
  
  // assume 8-bytes length packet
  pckt = rx1buf + (RX_BUF_SIZE - MESSAGE_SZ);

  // check packet (by crc)
  if (pckt[0] == slave_addr)
  {
    if ((pckt[1] == 2) || (pckt[1] == 5))
    {
			static uint16_t crc_1;
			static uint16_t crc_2;
			static uint16_t crc_2_l;
			static uint16_t crc_2_h;
			
			crc_1 = calc_crc(pckt, NUMBER_DATA);
			crc_2_l = *(uint8_t *)(pckt + NUMBER_DATA);
			crc_2_h = *(uint8_t *)(pckt + NUMBER_DATA + 1);
			
			crc_2 = (uint16_t)(crc_2_h << 8) | crc_2_l;
			
      if (crc_1 == crc_2)
      {
        // pass
#ifdef VERBOSE
        sprintf(buf, "[got command %d]\r\n", pckt[1]);
        HAL_UART_Transmit(&huart1,(uint8_t *)buf, str_len(buf), 10);
#endif
        if (pckt[1] == 2) // read input pins
        {
          // read input pins
          uint8_t discrete_inputs[5] = {0};
          if (pckt[3] == 0)	// read outputs
          {
            // is_opened (need check)
//            discrete_inputs = (GPIOC->IDR & 0xff00) >> 8; //@@@ HAL_GPIO_ReadPin(GPIOC, 0xff00) >> 8;
						//read_port_output((uint8_t *)discrete_inputs, 5);
						read_port_input ((uint8_t *)discrete_inputs, 5);
					}
          else if (pckt[3] == 8)	// read inputs
          {
            // is_occupied (need check)
//            discrete_inputs = (GPIOB->IDR & 0xff00) >> 8; //@@@ HAL_GPIO_ReadPin(GPIOB, 0xff00) >> 8;
						read_port_input ((uint8_t *)discrete_inputs, 5);
          }
					
          tx1buf[0] = slave_addr;
          tx1buf[1] = 2;	// input command
#ifdef COIL_8
          tx1buf[2] = 1; // number of bytes in message
#endif
#ifdef COIL_40
          tx1buf[2] = 5; // number of bytes in message
#endif
					
          if ((pb8_to_pb15_outputs_prepare_cntr > 64) && (pckt[3] == 8))
          {
            tx1buf[3] = discrete_inputs[0]; // do not invert!
            tx1buf[4] = discrete_inputs[1]; // do not invert!
            tx1buf[5] = discrete_inputs[2]; // do not invert!
            tx1buf[6] = discrete_inputs[3]; // do not invert!
            tx1buf[7] = discrete_inputs[4]; // do not invert!
          }
          else
          {
            tx1buf[3] = ~discrete_inputs[0]; // invert for compability!
            tx1buf[4] = ~discrete_inputs[1]; // invert for compability!
            tx1buf[5] = ~discrete_inputs[2]; // invert for compability!
            tx1buf[6] = ~discrete_inputs[3]; // invert for compability!
            tx1buf[7] = ~discrete_inputs[4]; // invert for compability!
          }
					
					// for 40 coils
#ifdef COIL_40
					static uint16_t crc_16;
					crc_16 = calc_crc((uint8_t *)tx1buf, 8);
					tx1buf[9] = (uint8_t)((crc_16 & 0xFF00) >> 8);
					tx1buf[8] = (uint8_t)(crc_16 & 0x00FF);
					
          // send packet
          send_to_rs485((uint8_t *)tx1buf, 10);
#endif
					// for 8 coils
#ifdef COIL_8
					static uint16_t crc_16;
					crc_16 = calc_crc((uint8_t *)tx1buf, 4);
					tx1buf[5] = (uint8_t)((crc_16 & 0xFF00) >> 8);
					tx1buf[4] = (uint8_t)(crc_16 & 0x00FF);
//          *(uint16_t *)(tx1buf + 8) = calc_crc((uint8_t *)tx1buf, 8);
					
          // send packet
          send_to_rs485((uint8_t *)tx1buf, 6);
#endif
        }
        else if (pckt[1] == 5) // write coil
        {
          // create task
          uint32_t task = 0xffffffff;
					// 0x3f - bit mask
					
					uint16_t curr_open_delay = DEFAUL_TIME_MS;
					/* find port number. */
					while (pckt[3] >= 0x28) {
						curr_open_delay += ADDITIONAL_SECOND_TO_OPEN_COIL;
						pckt[3] -= 0x28;
					}
					
          if ((pckt[3] & 0x3f) <= NUMBER_COILS)	// old was -> if ((pckt[3] & 0xf8) == 0)
          {
            if ((pckt[4] == 0xff) && (pckt[5] == 0x00))
            {
							// function for enable outputs
              // open lock
//              task = (uint16_t)(pckt[3] & 0x3f) | (OPEN_LOCK << 8);	// old was -> task = (uint16_t)(1 << (pckt[3] & 0x07)) | (OPEN_LOCK << 8);
            
							task = (uint32_t)((curr_open_delay << 16) | (OPEN_LOCK << 8) | (pckt[3] & 0x3f));
						}
          }
          else if ((pckt[3] & 0xf8) == 0x20)
          {
//            // fire or extinguish led
//            if ((pckt[4] == 0xff) && (pckt[5] == 0x00))
//            {
//              // fire led
//              task = (uint16_t)(1 << (pckt[3] & 0x07)) | (FIRE_LED << 8);
//            }
//            else if ((pckt[4] == 0x00) && (pckt[5] == 0x00))
//            {
//              // extinguish led
//              task = (uint16_t)(1 << (pckt[3] & 0x07)) | (EXTINGUISH_LED << 8);
//            }
          }
					
          tx1buf[0] = slave_addr;
          tx1buf[1] = 5;
          tx1buf[2] = 0;
          tx1buf[3] = pckt[3];
          tx1buf[4] = pckt[4]; // 0xff;
          tx1buf[5] = pckt[5]; // 0x00;
					
          *(uint16_t *)(tx1buf + NUMBER_DATA) = calc_crc((uint8_t *)tx1buf, NUMBER_DATA);
					
          // send packet
          send_to_rs485 ((uint8_t *)tx1buf, MESSAGE_SZ);
					
          // push task to queue
          push_task_to_queue (task);
        }
      }
    }
  } // slave_addr fits

  // assume 10-bytes length packet
//  pckt = rx1buf + (TXRXBUF_SZ-10);
	pckt = rx1buf;
	
  // check packet (by crc)
  if ((pckt[0] == slave_addr) && (pckt[1] == 0x0F))
  {
		static uint16_t crc_1;
		static uint16_t crc_2;
		static uint16_t crc_2_l;
		static uint16_t crc_2_h;
		// previously was 8 instead of 6
		crc_1 = calc_crc(pckt, 6);
		crc_2_l = *(uint8_t *)(pckt + NUMBER_DATA);
		crc_2_h = *(uint8_t *)(pckt + NUMBER_DATA + 1);
		
		crc_2 = (uint16_t)(crc_2_h << 8) | crc_2_l;
		
    if (crc_1 == crc_2)
    {
      uint16_t i;

      // pass
#ifdef VERBOSE
      sprintf(buf, "[got command %d]\r\n", pckt[1]);
      HAL_UART_Transmit(&huart1,(uint8_t *)buf, str_len(buf), 10);
#endif
      // open all locks
      tx1buf[0] = slave_addr;
      tx1buf[1] = 15;
      tx1buf[2] = 0;
      tx1buf[3] = 0;
      tx1buf[4] = 0;
      tx1buf[5] = 4;
      *(uint16_t *)(tx1buf + NUMBER_DATA) = calc_crc((uint8_t *)tx1buf, NUMBER_DATA);
			
      // send packet
      send_to_rs485((uint8_t *)tx1buf, MESSAGE_SZ);

#ifdef COIL_40
      for (i = 0; i < NUMBER_COILS; i++)
      {
				push_task_to_queue ((uint32_t)((DEFAUL_TIME_MS << 16) | (OPEN_LOCK << 8) | i));
//        push_task_to_queue(i | (OPEN_LOCK << 8));
      }
#endif
#ifdef COIL_8
      for (i = 0; i < 8; i++)
      {
        push_task_to_queue(i | (OPEN_LOCK << 8));
      }
			
			__NOP();
#endif
    }
  } // slave_addr fits
}

void switch_off_all_coils ()
{
#ifdef USED_STM32F091
	for (uint8_t i = 0; i < NUMBER_COILS; i++) {
		HAL_GPIO_WritePin (ports_out_mapping[i], number_ports_out[i], GPIO_PIN_RESET);
	}
#endif
#ifdef USED_STM32F407
	for (uint8_t i = 0; i < NUMBER_COILS; i++) {
		HAL_GPIO_WritePin (ports_out_mapping[i], number_ports_out[i], GPIO_PIN_RESET);
	}
#endif
}

uint32_t get_next_task_from_queue()
{
  if (queue_fullness)
  {
    uint32_t ret;
    queue_fullness--;
    ret = *p_queue_out;
    p_queue_out++;
    if ((p_queue_out - queue) >= QUEUE_SZ)
    {
      p_queue_out = queue;
    }
    return ret;
  }
  else
    return 0;
}

void push_task_to_queue(uint32_t task)
{
  if (queue_fullness < QUEUE_SZ)
  {
    *p_queue_in = task;
    p_queue_in++;
    if ((p_queue_in - queue) >= QUEUE_SZ)
    {
      p_queue_in = queue;
    }
    // ---
    queue_fullness++;
  }
}

void programming ()
{
  static uint8_t prg_state = 1;
  // 1 - initial (waiting for prg_btn to release),
  // 2 - waiting for short or long or ultra_long pressing

	#define DEBOUNCE_LIMIT (25/5)
	#define PRG_EXIT_LIMIT (3600/5)
	#define SHORT_PRESS (450/5)
	#define LONG_PRESS (2400/5)
	
  static uint8_t debounce_cntr = 0;
  static uint16_t integral = 0; // long and short pressings
  static uint8_t new_slave_addr = 0;
  static uint8_t curr_prg_btn = 0; // pressed
  uint8_t prg_btn;

#ifdef USED_STM32F091
  prg_btn = HAL_GPIO_ReadPin (GPIOA, IN_PROG_Pin);
#endif
#ifdef USED_STM32F407
  prg_btn = HAL_GPIO_ReadPin (GPIOA, IN_PROG_Pin);
#endif
	
  if (prg_btn != curr_prg_btn) debounce_cntr++; else debounce_cntr = 0;
	
  if (debounce_cntr >= DEBOUNCE_LIMIT)
  {
    // change current btn state
    curr_prg_btn = prg_btn;
    if (curr_prg_btn == 0) led_on (LED_Pin); else led_off (LED_Pin);
		
    // change prg state (if needed)
    if ((prg_state == 1) && (curr_prg_btn == 1)) // btn released
    {
      led_on (LED_Pin);
      prg_state = 2;
    }
    else if ((prg_state == 2) && (curr_prg_btn == 1)) // btn released
    {
      if (integral <= SHORT_PRESS)
      {
        new_slave_addr++;
      }
      else if (integral <= LONG_PRESS)
      {
        new_slave_addr += 10;
      }
      else if (integral > LONG_PRESS) // ultra long
      {
        prg_state = 3;
//        led_on (0); //@@@
      }
    }
    // ---
    integral = 0;
  }
  else // btn state is not changed yet
  {
    integral++;
    // check for short/long/ultra_long pressing
    if ((prg_state == 2) && (curr_prg_btn == 1) && (integral >= PRG_EXIT_LIMIT))	// btn released
		{
      led_off (LED_Pin);
			
      prg_state = 0;
      // write new_slave_addr to eeprom, set current slave_addr
      if (new_slave_addr)
      {
				
#ifdef USED_STM32F091
        // only if not zero
				HAL_FLASH_Unlock ();

				FLASH_EraseInitTypeDef erase_page;

				erase_page.NbPages = 1;
				erase_page.PageAddress = START_FLASH_ADDRESS_PAGE_127;
				erase_page.TypeErase = FLASH_TYPEERASE_PAGES;
				
				uint32_t pageError[1] = {0xFFFFFFFF};
				
				HAL_FLASHEx_Erase (&erase_page, (uint32_t *)pageError);
				HAL_FLASH_Program (FLASH_TYPEPROGRAM_HALFWORD, START_FLASH_ADDRESS_PAGE_127, new_slave_addr);
				HAL_FLASH_Lock ();
				
        slave_addr = new_slave_addr;
#endif
				
#ifdef USED_STM32F407
        // only if not zero
				HAL_FLASH_Unlock ();

				FLASH_EraseInitTypeDef erase_page;

				erase_page.TypeErase = FLASH_TYPEERASE_SECTORS;
				erase_page.VoltageRange = FLASH_VOLTAGE_RANGE_3;
				erase_page.Sector = FLASH_SECTOR_6;
				erase_page.NbSectors = 1;
				
				uint32_t pageError[1] = {0xFFFFFFFF};
				
				HAL_FLASHEx_Erase (&erase_page, (uint32_t *)pageError);
				HAL_FLASH_Program (FLASH_TYPEPROGRAM_HALFWORD, START_FLASH_ADDRESS_SECTOR_6, new_slave_addr);
				HAL_FLASH_Lock ();
				
        slave_addr = new_slave_addr;
#endif
      }
			
      state = STATE_NORMAL;
			
      return;
    }
  }
}

void blinking ()
{
  static uint8_t blink_cntr = 45;
  static uint8_t blink_state = 3;
  // 0 - pause, 1 - short,
  // 2 - long, 3 - setting up
  
  static uint8_t blink_word = 0;
  
  if (blink_cntr == 0) // time to switch blink state
  {
    led_off (LED_Pin); // for sure
    
    // reload blink_word
    if (blink_state == 0)
    {
      blink_word = slave_addr;
    }
    
    if (blink_word >= 10)
    {
      blink_word -= 10;
      blink_state = 2; // long blink
      blink_cntr = 240; // start blinking
      led_on (LED_Pin);
    }
    else if (blink_word)
    {
      blink_word--;
      blink_state = 1; // short blink
      blink_cntr = 80; // start blinking
      led_on (LED_Pin);
    }
    else if (blink_word == 0)
    {
      blink_state = 0; // pause
      blink_cntr = 240;
    }
  }
  else
  {
    if ((blink_state == 1) &&
     (blink_cntr == 40)) led_off (LED_Pin);
    else if ((blink_state == 2) &&
     (blink_cntr == 120)) led_off (LED_Pin);
    else if (blink_state == 3)
    {
      uint8_t rest7 = blink_cntr % 7;
      if (rest7 == 0)
      {
        if ((blink_cntr / 7) % 2) led_off (LED_Pin);
        else led_on (LED_Pin);
      }
    }
    // ---
    blink_cntr--;
  }
}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *p_htim)
{
  if (p_htim == &htim3) //check if the interrupt comes from TIM3
  {
    static uint8_t first_time = 1;
    if (first_time)
    {
      static uint8_t prg_btn;
			
#ifdef USED_STM32F091
			prg_btn = HAL_GPIO_ReadPin (GPIOA, IN_PROG_Pin);
#endif
#ifdef USED_STM32F407
			prg_btn = HAL_GPIO_ReadPin (GPIOA, IN_PROG_Pin);
#endif
//  prg_btn = HAL_GPIO_ReadPin (GPIOE, IN_PROG_Pin);
			      
			if (prg_btn == BOARD_PROG_BTN_LOW)
      {
        // enter programming mode
        state = STATE_PRG;
      }
      else
      {
        state = STATE_NORMAL;
      }
      // ---
      first_time = 0;
    }
    if (state == STATE_NORMAL)
    {
			/* ----- change blinking mode by prog button ----- */
			static uint8_t blink_mode_btn = 0;
			
#ifdef USED_STM32F091
			blink_mode_btn = HAL_GPIO_ReadPin (GPIOA, IN_PROG_Pin);
#endif
#ifdef USED_STM32F407
			blink_mode_btn = HAL_GPIO_ReadPin (GPIOA, IN_PROG_Pin);
#endif
			
			if ((blink_mode_btn == BOARD_PROG_BTN_LOW) && (firm_code.flag_first_push == 0))	// PUSH BUTTTON
      {
				firm_code.flag_first_push = 1;
      }
			else if ((blink_mode_btn == BOARD_PROG_BTN_HIGH) && (firm_code.flag_first_push == 1)) {	// RELEASE BUTTON
				firm_code.flag_first_push = 0;
        firm_code.stage_global = 1;
				firm_code.stage_array++;
				
				HAL_GPIO_WritePin (LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
				HAL_TIM_OC_Start_IT (&htim14, TIM_CHANNEL_1);
			}

			if (firm_code.stage_global == 2) {
					firm_code.stage_array = 0;
					firm_code.stage_global = 0;
			
					HAL_TIM_OC_Stop_IT (&htim14, TIM_CHANNEL_1);
			}
			else if (!firm_code.stage_global) {
				blinking ();
			}
			
			/* ----- change blinking mode by time period ----- */
//			cur_ms = sys_ms;
//			
//			if (cur_ms >= prev_ms) {
//				calc_time = cur_ms - prev_ms;
//			}
//			else {
//				calc_time = MAX_SYS_MS - prev_ms + cur_ms;
//			}
//			
//			if (calc_time >= DELAY_LED_MS) {
//				prev_ms = cur_ms;
//				
//				if (state_led == BLINK) {
//					state_led = MORZE;
//				}
//				else {
//					state_led = BLINK;
//				}
//			}
//			
//			if (state_led == BLINK) {
//				// blink
//				blinking ();
//				
//				firm_code.stage_global = 0;
//			}
//			else {
//				if (firm_code.stage_global == 0) {
//					firm_code.stage_global = 1;
//					firm_code.stage_array++;
//					
//					HAL_GPIO_WritePin (LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//					HAL_TIM_OC_Start_IT (&htim14, TIM_CHANNEL_1);
//					
//				}
//				else if (firm_code.stage_global == 2) {
//				
//					firm_code.stage_array = 0;
//			
//					HAL_TIM_OC_Stop_IT (&htim14, TIM_CHANNEL_1);
//				}
//			}
    }
    else if (state == STATE_PRG)
    {
      // programming
      programming ();
    }
    // spin
    spin_once ();
  }
}

void HAL_TIM_PWM_PulseFinishedCallback (TIM_HandleTypeDef *p_htim) {
	if (p_htim == &htim14) {
		if (firm_code.stage_array == MORZE_SIZE - 1) {
			firm_code.stage_global = 2;
			
			HAL_GPIO_WritePin (LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		}
		else {
			if (morze_code_output[firm_code.stage_array] == 1) {
				HAL_GPIO_WritePin (LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			}
			else if (morze_code_output[firm_code.stage_array] == 0) {
				HAL_GPIO_WritePin (LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			}
			
			firm_code.stage_array++;
		}
	}
}

/**
  * @brief  Rx Transfer completed callback
  * @param  p_huart: UART handler pointer
  * @retval None
  */
void HAL_UART_RxCpltCallback (UART_HandleTypeDef *p_huart)
{
   if (p_huart == &huart2)
  {
    HAL_UART_Receive_IT (p_huart, &rx1_data, 1);
    rx1_accum ();
  }
}

void HAL_UART_ErrorCallback (UART_HandleTypeDef *p_huart)
{
   if (p_huart == &huart2)
  {
		uint32_t err = 0;
		err = HAL_UART_GetError (p_huart);
		
		HAL_UART_DeInit (p_huart);
		for (uint8_t i = 0; i < RX_BUF_SIZE; i++) {
			rx1buf[i] = 0;
		}
		MX_USART2_UART_Init ();
		
		
		HAL_UART_Receive_IT (&huart2, &rx1_data, 1);
		__NOP();
  }
}

/**
  * @brief  Tx Transfer completed callback
  * @param  p_huart: UART handler pointer
  * @retval None
  */
void HAL_UART_TxCpltCallback (UART_HandleTypeDef *p_huart)
{
  if (p_huart == &huart2)
  {
    rs485_write_dis ();
  }
}

/*
brief: case: [number_port] where number_port is number of port
*/
void write_port (uint8_t number_port)
{
	HAL_GPIO_WritePin (ports_out_mapping[number_port], number_ports_out[number_port], GPIO_PIN_SET);
}

void read_port_input (uint8_t * arr, uint8_t size_arr) {
	for (uint8_t i = 0; i < size_arr; i++) {
		arr[i] =	(HAL_GPIO_ReadPin (ports_in_mapping[8 * i], number_ports_in[8 * i])) |
							(HAL_GPIO_ReadPin (ports_in_mapping[8 * i + 1], number_ports_in[8 * i + 1]) << 1) |
							(HAL_GPIO_ReadPin (ports_in_mapping[8 * i + 2], number_ports_in[8 * i + 2]) << 2) |
							(HAL_GPIO_ReadPin (ports_in_mapping[8 * i + 3], number_ports_in[8 * i + 3]) << 3) |
							(HAL_GPIO_ReadPin (ports_in_mapping[8 * i + 4], number_ports_in[8 * i + 4]) << 4) |
							(HAL_GPIO_ReadPin (ports_in_mapping[8 * i + 5], number_ports_in[8 * i + 5]) << 5) |
							(HAL_GPIO_ReadPin (ports_in_mapping[8 * i + 6], number_ports_in[8 * i + 6]) << 6) |
							(HAL_GPIO_ReadPin (ports_in_mapping[8 * i + 7], number_ports_in[8 * i + 7]) << 7);
	}
}

void read_port_output (uint8_t * arr, uint8_t size_arr) {
	for (uint8_t i = 0; i < size_arr; i++) {
		arr[i] =	(get_odr_reg (ports_out_mapping[8 * i], number_ports_out[8 * i]) << 7) |
							(get_odr_reg (ports_out_mapping[8 * i + 1], number_ports_out[8 * i + 1]) << 6) |
							(get_odr_reg (ports_out_mapping[8 * i + 2], number_ports_out[8 * i + 2]) << 5) |
							(get_odr_reg (ports_out_mapping[8 * i + 3], number_ports_out[8 * i + 3]) << 4) |
							(get_odr_reg (ports_out_mapping[8 * i + 4], number_ports_out[8 * i + 4]) << 3) |
							(get_odr_reg (ports_out_mapping[8 * i + 5], number_ports_out[8 * i + 5]) << 2) |
							(get_odr_reg (ports_out_mapping[8 * i + 6], number_ports_out[8 * i + 6]) << 1) |
							(get_odr_reg (ports_out_mapping[8 * i + 7], number_ports_out[8 * i + 7]));
	}
}

uint8_t get_odr_reg (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
  uint8_t bitstatus;

  /* Check the parameters */
  assert_param(IS_GPIO_PIN(GPIO_Pin));

  if ((GPIOx->ODR & GPIO_Pin) != (uint32_t)GPIO_PIN_RESET)
  {
    bitstatus = GPIO_PIN_SET;
  }
  else
  {
    bitstatus = GPIO_PIN_RESET;
  }
  return bitstatus;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
