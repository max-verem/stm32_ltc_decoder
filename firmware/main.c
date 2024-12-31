/* USER CODE BEGIN Header */

/*
 * PAL TV Framerate is 25 fps.
 * Frame duration is 40ms (40000us)
 * Each bit is 500us
 * bit 0 is  1x500us
 * bit 1 is  2x250us
 *
 */

#define BIT_0_DUR 500
#define BIT_1_DUR 250
#define BIT_THRESHOLD 100

#define BIT_NONE 0
#define BIT_ZERO 1
#define BIT_ONE  2

#define TIMER1_PRESCALER_RAW 48
#define TIMER1_PERIOD_RAW 1000
#define TIMER1_PRESCALER (TIMER1_PRESCALER_RAW - 1)
#define TIMER1_PERIOD (TIMER1_PERIOD_RAW - 1)

//#define DURS_BUF 256

#ifdef DURS_BUF
volatile unsigned int durs_idx = 0;
volatile unsigned int durs_buf[DURS_BUF];
#endif

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile unsigned int ltc_last = 0;

#define TIMER1_DIV(DIV) if(!(timer1_cnt % DIV))
#define TIMER1_DIV_BLINK 50
volatile int timer1_cnt = 0;
static void timer1_cb(TIM_HandleTypeDef *htim)
{
	unsigned int now = HAL_GetTick();

	timer1_cnt++;

	/* check if we should solid led if no timecode detected */
	if((ltc_last + 1000) < now)
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	else
	{
		/* blink fast it timecode detected */
		TIMER1_DIV(TIMER1_DIV_BLINK)
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	};

	/* this is for debug timer */
	HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, timer1_cnt % 3 ? GPIO_PIN_SET : GPIO_PIN_RESET);
};

// https://habr.com/ru/articles/476582/
// https://community.st.com/t5/stm32-mcus-products/how-do-you-measure-the-execution-cpu-cycles-for-a-section-of/td-p/213709
volatile unsigned int *DWT_CYCCNT   = (volatile unsigned int *)0xE0001004;
volatile unsigned int *DWT_CONTROL  = (volatile unsigned int *)0xE0001000;
volatile unsigned int *DWT_LAR      = (volatile unsigned int *)0xE0001FB0;
volatile unsigned int *SCB_DHCSR    = (volatile unsigned int *)0xE000EDF0;
volatile unsigned int *SCB_DEMCR    = (volatile unsigned int *)0xE000EDFC;
volatile unsigned int *ITM_TER      = (volatile unsigned int *)0xE0000E00;
volatile unsigned int *ITM_TCR      = (volatile unsigned int *)0xE0000E80;

volatile unsigned int ltc_raw[2], ltc_found = 0;

volatile unsigned int bits_buffer[3], bits_count = 0;

#define SYNC_WORD 0xBFFC // 1011 1111 1111 1100

static void probe_ltc()
{
	if(bits_buffer[2] == SYNC_WORD)
	{
		ltc_found = bits_count;
		ltc_raw[0] = bits_buffer[0];
		ltc_raw[1] = bits_buffer[1];
		bits_count = 0;
	}
};

static void push_bit(int b)
{
	if(b == BIT_NONE)
		bits_count = 0;
	else
	{
		// shift
		bits_buffer[0] >>= 1;
		if(bits_buffer[1] & 1) bits_buffer[0] |= 0x80000000;
		bits_buffer[1] >>= 1;
		if(bits_buffer[2] & 1) bits_buffer[1] |= 0x80000000;
		bits_buffer[2] >>= 1;
		if(b == BIT_ONE)
			bits_buffer[2] |= (1 << 15);

		bits_count++;

		if(bits_count >= 80)
			probe_ltc();
	}
};

volatile unsigned int freq_cnt = 0;
volatile unsigned int bit_prev = BIT_NONE;

static void push_tick(unsigned int cnt)
{
	unsigned int dur;

	if(!freq_cnt)
	  freq_cnt = HAL_RCC_GetHCLKFreq() / 1000000;

	dur = cnt / freq_cnt;
	if((BIT_0_DUR - BIT_THRESHOLD) < dur && dur < (BIT_0_DUR + BIT_THRESHOLD))
	{
		bit_prev = BIT_NONE;
		push_bit(BIT_ZERO);
	}
	else if((BIT_1_DUR - BIT_THRESHOLD) < dur && dur < (BIT_1_DUR + BIT_THRESHOLD))
	{
		if(bit_prev)
		{
			bit_prev = BIT_NONE;
			push_bit(BIT_ONE);
		}
		else
			bit_prev = BIT_ONE;
	}
	else
	{
#ifdef DURS_BUF
		durs_buf[durs_idx] = dur; durs_idx = (durs_idx + 1) % DURS_BUF;
#endif
		bit_prev = BIT_NONE;
		push_bit(BIT_NONE);
	}
};

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	unsigned int cnt;

	if(GPIO_Pin != LTC_IN_Pin)
		return;

	cnt = *DWT_CYCCNT;
	*DWT_CYCCNT = 0;

	push_tick(cnt);
};

//  https://controllerstech.com/pwm-input-in-stm32/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
#if 0
	if(!freq_cnt)
		freq_cnt = HAL_RCC_GetHCLKFreq() / 1000000;
#endif

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		uint32_t per, high, low;

		// Read the IC value
		per = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		high = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		if(per)
		{
			low = per - high;

			push_tick(high);
			push_tick(low);
#if 0
			high /= freq_cnt;
			low /= freq_cnt;

			durs_buf[durs_idx] = high; durs_idx = (durs_idx + 1) % DURS_BUF;
			durs_buf[durs_idx] = low; durs_idx = (durs_idx + 1) % DURS_BUF;
#endif
		}
	}
};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  unsigned int tc = 0;
  unsigned char str[13] = "--:--:--:--\r\n";
  const unsigned char str_map[16] = "0123456789ABCDEF";
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
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  // https://habr.com/ru/articles/476582/
  // https://community.st.com/t5/stm32-mcus-products/how-do-you-measure-the-execution-cpu-cycles-for-a-section-of/td-p/213709
  *SCB_DEMCR |= 0x01000000;
  *DWT_LAR = 0xC5ACCE55; // enable access
  *DWT_CYCCNT = 0; // reset the counter
  *DWT_CONTROL |= 1 ; // enable the counter

  // start timer
  HAL_TIM_RegisterCallback(&htim1, HAL_TIM_PERIOD_ELAPSED_CB_ID, timer1_cb);
  HAL_TIM_Base_Start_IT(&htim1);

  // start PWM counters
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(ltc_found != 80)
		  continue;
	  ltc_found = 0;

	  tc |= (ltc_raw[0] & 0x0000000F) >> 0;
	  tc |= (ltc_raw[0] & 0x00000300) >> 4;
	  tc |= (ltc_raw[0] & 0x000F0000) >> 8;
	  tc |= (ltc_raw[0] & 0x07000000) >> 12;

	  tc |= (ltc_raw[1] & 0x0000000F) << 16;
	  tc |= (ltc_raw[1] & 0x00000700) << 12;
	  tc |= (ltc_raw[1] & 0x000F0000) << 8;
	  tc |= (ltc_raw[1] & 0x03000000) << 4;


	  str[10] = str_map[ tc & 0x0F ]; tc >>= 4;
	  str[ 9] = str_map[ tc & 0x0F ]; tc >>= 4;
	  str[ 7] = str_map[ tc & 0x0F ]; tc >>= 4;
	  str[ 6] = str_map[ tc & 0x0F ]; tc >>= 4;
	  str[ 4] = str_map[ tc & 0x0F ]; tc >>= 4;
	  str[ 3] = str_map[ tc & 0x0F ]; tc >>= 4;
	  str[ 1] = str_map[ tc & 0x0F ]; tc >>= 4;
	  str[ 0] = str_map[ tc & 0x0F ]; tc >>= 4;

	  ltc_last = HAL_GetTick();

	  CDC_Transmit_FS(str, sizeof(str));
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = TIMER1_PRESCALER;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = TIMER1_PERIOD;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SYNC_Pin */
  GPIO_InitStruct.Pin = SYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SYNC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LTC_IN_Pin */
  GPIO_InitStruct.Pin = LTC_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LTC_IN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
  while (1)
  {
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
