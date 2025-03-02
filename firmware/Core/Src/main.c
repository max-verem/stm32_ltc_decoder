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

//#define DURS_BUF 256

#ifdef DURS_BUF
volatile unsigned int durs_idx = 0;
volatile unsigned int durs_buf[DURS_BUF];
#endif

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "SSD1306.h"
#include "font-14x30.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TC_DISPLAY_UNKNOWN 	0xAAAAAAAA
#define TC_DISPLAY_NONE 	0xFFFFFFFF
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

SSD1306_DEF(oled1, hi2c2, SSD1306_I2C_ADDR, 32, 0);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile unsigned int ltc_last = 0;
uint32_t tc_display = TC_DISPLAY_UNKNOWN, tc_displayed = TC_DISPLAY_NONE;

#define TIMER1_DIV(DIV) if(!(timer1_cnt % DIV))
#define TIMER1_DIV_BLINK 50
volatile int timer1_cnt = 0;
static void timer1_cb(TIM_HandleTypeDef *htim)
{
	unsigned int now = HAL_GetTick();

	timer1_cnt++;

	/* check if we should solid led if no timecode detected */
	if((ltc_last + 1000) < now)
	{
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		tc_display = TC_DISPLAY_UNKNOWN;
	}
	else
	{
		/* blink fast it timecode detected */
		TIMER1_DIV(TIMER1_DIV_BLINK)
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	};
#if 0
	/* this is for debug timer */
	HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, timer1_cnt % 3 ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
	SSD1306_refresh(&oled1);
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
		}
	}
};

static uint32_t console_font_14x30_transposed[16 /* glyphs */][14 /* width */];

static void console_font_14x30_transpose()
{
	int g;

	for(g = 0; g < 12; g++)
	{
		int w, b, o;
		const uint8_t *src_row = console_font_14x30 + 2 * 30 * g;

		for(w = 0, o = 0, b = 0x80; w < 14; w++, b >>= 1)
		{
			int h;
			uint32_t W = 0;

			if(!b)
			{
				b = 0x80;
				o = 1;
			}

			for(h = 0; h < 30; h++)
				if(src_row[h * 2 + o] & b)
					W |= 1 << h;

			console_font_14x30_transposed[g][w] = W << 4;
		}
	};
}

volatile int semicolon_offset = 3;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
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
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  // setup and init oled display
  console_font_14x30_transpose();
  SSD1306_setup(&oled1);

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

	  if(ltc_found == 80)
	  {
		  uint32_t tc = 0;

		  ltc_found = 0;

		  tc |= (ltc_raw[0] & 0x0000000F) >> 0;
		  tc |= (ltc_raw[0] & 0x00000300) >> 4;
		  tc |= (ltc_raw[0] & 0x000F0000) >> 8;
		  tc |= (ltc_raw[0] & 0x07000000) >> 12;

		  tc |= (ltc_raw[1] & 0x0000000F) << 16;
		  tc |= (ltc_raw[1] & 0x00000700) << 12;
		  tc |= (ltc_raw[1] & 0x000F0000) << 8;
		  tc |= (ltc_raw[1] & 0x03000000) << 4;

		  tc_display = tc;

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

	  }

	  if(tc_display != tc_displayed && !oled1.dirty && !oled1.busy)
	  {
		  uint32_t bcd;
		  int i = 0, j, s;
#if 1
		  /* this is for estimating display rendering time */
		  HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_SET);
#endif
		  bcd = tc_display;
		  tc_displayed = tc_display;

		  for(j = 0, s = 28; j < 8; j++, s -= 4)
		  {
			  int w, g;

			  // spacer
			  if(j && !(j & 1))
			  for(w = 0; w < 5; w++, i++)
			  {
				  uint32_t W = console_font_14x30_transposed[11 /* glyphs */][w + semicolon_offset /* width */];

				  oled1.fb[0][SSD1306_DATA_OFFSET + i] = W >>  0;
				  oled1.fb[1][SSD1306_DATA_OFFSET + i] = W >>  8;
				  oled1.fb[2][SSD1306_DATA_OFFSET + i] = W >> 16;
				  oled1.fb[3][SSD1306_DATA_OFFSET + i] = W >> 24;
			  }

			  // find glyph index
			  g = bcd >> s;
			  g &= 0x0F;
			  if(g > 10) g = 10;

			  // copy glyph bitmap data
			  for(w = 0; w < 14; w++, i++)
			  {
				  uint32_t W = console_font_14x30_transposed[g /* glyphs */][w /* width */];

				  oled1.fb[0][SSD1306_DATA_OFFSET + i] = W >>  0;
				  oled1.fb[1][SSD1306_DATA_OFFSET + i] = W >>  8;
				  oled1.fb[2][SSD1306_DATA_OFFSET + i] = W >> 16;
				  oled1.fb[3][SSD1306_DATA_OFFSET + i] = W >> 24;
			  }
		  }

		  // mark page dirty
		  oled1.dirty = 0x0f;
#if 1
		  /* this is for estimating display rendering time */
		  HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_RESET);
#endif
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
