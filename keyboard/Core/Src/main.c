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
#include "usb_device.h"

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "stdio.h"
#include "stdarg.h"
#include "stdbool.h"
#include "fonts.h"
#include "ssd1306.h"
#include "DS1307.h"
#include "Button_matrix.h"

#define ADC_BUFF_LEN 32		// sample to moving average filter
#define ADC_THRESHOLD 80 // threshold for changing %volume
#define VOLUME_MAX 100	//MAX VOLUME

#include "usbd_customhid.h"
extern USBD_HandleTypeDef hUsbDeviceFS;

void Send_key(uint8_t modifier, uint8_t keycode);
void Send_consumer(uint16_t keycode);

uint16_t adc_val[ADC_BUFF_LEN];
uint8_t volume = 0;
uint16_t volume_now, volume_old = 0;

char buff_time[20];
char buff_day[20];

char buff_uart[40];
//===================Time==================
DS1307_Typedef time_data;

//====================UART DEBUG============
void Print_uart(const char*format,...)
{
	for(uint8_t i=0; i<40; i++)
	{
		buff_uart[i] = 0;
	}
	va_list args;
	va_start(args, format);
	vsnprintf(buff_uart, sizeof(buff_uart), format, args);
	va_end(args);
	HAL_UART_Transmit(&huart1, (uint8_t*)buff_uart, 40, 1000);
}

//=========================key press handle=============
Matrix_Pin_Typedef rowS[4] = {
	{GPIOA, GPIO_PIN_15}, // ROW 1 - 4
	{GPIOB, GPIO_PIN_3},
	{GPIOB, GPIO_PIN_4},
	{GPIOB, GPIO_PIN_5},
};

Matrix_Pin_Typedef colS[4] = {
	{GPIOA, GPIO_PIN_8}, // col 1 - 4
	{GPIOB, GPIO_PIN_15},
	{GPIOB, GPIO_PIN_14},
	{GPIOB, GPIO_PIN_13},
};


Matrix_Typedef button_matrix = {
	.num_row = 4, 
	.num_col = 4,
	.row_pins = rowS,
	.col_pins = colS
};

typedef enum
{
	KEY_TYPE_KEYBOARD,
	KEY_TYPE_CONSUMER
}KeyType_t;

typedef struct
{
	KeyType_t type;
	uint8_t modifier;
	uint16_t keycode;
}KeyMap_t;

KeyMap_t macro_key[16] = {
	{KEY_TYPE_KEYBOARD, 0x00, 0x46}, 	// KEY_PrtSc 1 
	{KEY_TYPE_KEYBOARD, 0x0A, 0x16},	// KEY_SNIPPING_TOOL 2
	{KEY_TYPE_KEYBOARD, 0x08, 0x07},	//KEY_SHOW_DESKTOP 3
	{KEY_TYPE_KEYBOARD, 0x04, 0x3D},	// KEY_CLOSE_WINDOW 4
	{KEY_TYPE_KEYBOARD, 0x01, 0x1B},	//KEY_CUT 5
	{KEY_TYPE_KEYBOARD, 0x01, 0x06},	//KEY_COPY 6
	{KEY_TYPE_KEYBOARD, 0x01, 0x19},	//KEY_PASTE 7
	{KEY_TYPE_KEYBOARD, 0x01, 0x1D},	//KEY_UNDO 8
	{KEY_TYPE_KEYBOARD, 0x01, 0x1C},	//KEY_REDO 9
	{KEY_TYPE_KEYBOARD, 0x01, 0x04},	// KEY_SELECT_ALL 10
	{KEY_TYPE_CONSUMER, 0x00, 0xE2},	// KEY_MUTE 11
	{KEY_TYPE_CONSUMER, 0x00, 0xCD},	//KEY_PLAY_PAUSE 12
	{KEY_TYPE_CONSUMER, 0x00, 0xB5},	//KEY_NEXT_TRACK 13
	{KEY_TYPE_CONSUMER, 0x00, 0xB6},	//KEY_PREV_TRACK 14
	{KEY_TYPE_KEYBOARD, 0x01, 0x1A},	// KEY_CLOSE_TAB 15
	{KEY_TYPE_KEYBOARD, 0x01, 0x17}		//KEY_NEW_TAB 16
};

void Matrix_key_press_Callback(uint8_t key, KeyEvent_t key_event)
{
	KeyMap_t key_map = macro_key[key-1];
	if(key_event == KEY_PRESS)
	{
		if(key_map.type == KEY_TYPE_KEYBOARD)
		{
			Send_key(key_map.modifier, key_map.keycode);
			Print_uart("Key %d sent, type keyboard\n", key);
		}
		else if(key_map.type == KEY_TYPE_CONSUMER)
		{
			Send_consumer(key_map.keycode);
			Print_uart("Key %d sent, type consumer\n", key);
		}
	}
}

void Send_key(uint8_t modifier, uint8_t keycode)
{
	uint8_t keyboard_report[9] = {0};

	keyboard_report[0] = 0x01;
	keyboard_report[1] = modifier;
	keyboard_report[2] = 0;
	keyboard_report[3] = keycode;
	
	Print_uart("ID: %02X, Modifier: %02X, Keycode: %02X\n", keyboard_report[0], keyboard_report[1], keyboard_report[3]);
	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&keyboard_report, sizeof(keyboard_report));
    HAL_Delay(20);
	
	memset(keyboard_report, 0, sizeof(keyboard_report));
	keyboard_report[0] = 0x01;
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, keyboard_report, sizeof(keyboard_report));
    HAL_Delay(20);
}

void Send_consumer(uint16_t keycode)
{
    uint8_t consumer_report[3];

	consumer_report[0] = 0x02;
	consumer_report[1] = keycode & 0xFF; // LSB
	consumer_report[2] = keycode >> 8;	//MSB
	
	Print_uart("ID: %02X, Keycode: %02X\n", consumer_report[0], consumer_report[1]);
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, consumer_report, sizeof(consumer_report));
    HAL_Delay(20);

    // Send release
	consumer_report[1] = 0;
	consumer_report[2] = 0;
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, consumer_report, sizeof(consumer_report));
    HAL_Delay(20);
}
	
//==========================Display handle==============
const char dow_arr[][10] = {{"SUN"},{"MON"},{"TUE"},{"WED"},
							{"THU"},{"FRI"},{"SAT"}}; //day of week 
	
void Oled_display() // display date, month, year and keyboard map
{
	DS1307_read(&time_data);
	
	//=========================Time==============================
	sprintf(buff_time, "%02d:%02d:%02d", time_data.hour, time_data.min, time_data.sec);
    SSD1306_GotoXY(15, 20); 								//col, row
    SSD1306_Puts(buff_time, &Font_11x18, 1);
	
	//=====================GET DAY====================
	uint8_t dow = DS1307_get_day_of_week(&time_data); 
	SSD1306_GotoXY(90, 5);
	sprintf(buff_day, "%s", dow_arr[dow]);
	SSD1306_Puts(buff_day, &Font_7x10, 1);
	
	//========================Date=======================================
    SSD1306_GotoXY(5, 5);
    sprintf(buff_day, "%02d/%02d/20%02d", time_data.date, time_data.month, time_data.year);
    SSD1306_Puts(buff_day, &Font_7x10, 1);
	
	SSD1306_UpdateScreen();
}

//======================Adjust volume==================

// adjust volume base on adc value but adc value fluctuated
// set threshold, if current_value > previous_value && current_value - prev_value > threshold => adjust
uint8_t Check_adc_threshold() 
{
	uint8_t is_pass_threshold = 0;
	uint16_t current_adc_val = adc_val[0];
	static uint16_t is_first_run = 1;
	static uint16_t prev_adc_val = 0;
	
	if(is_first_run)
	{
		prev_adc_val = current_adc_val;
		is_first_run = 0;
		return 0;
	}
	
	if((current_adc_val > prev_adc_val) && (current_adc_val-prev_adc_val>ADC_THRESHOLD))
	{
		is_pass_threshold = 1;
		prev_adc_val += ADC_THRESHOLD;
	}
	else if((current_adc_val < prev_adc_val) && (prev_adc_val-current_adc_val>ADC_THRESHOLD))
	{
		is_pass_threshold = 2;
		prev_adc_val -= ADC_THRESHOLD;
	}

	return is_pass_threshold;
}


void HID_VolumeControl() // rotary switch adjust volume
{
	uint8_t is_adjust = Check_adc_threshold(); // check adjust allowance
    uint8_t report[3];

    report[0] = 0x02;   // Report ID
    report[1] = 0x00;   // default release
	report[2] = 0x00;   // default release
	
	if(adc_val[0] < 30) adc_val[0] = 0;
	if(adc_val[0] > 4050) adc_val[0] = 4050;
	
    if(is_adjust == 1)   // Volume Up
	{
		report[1] = 0xE9 & 0xFF;
		report[2] = (0xE9 >> 8) & 0xFF;
	}		
        
    else if(is_adjust == 2)  // Volume Down
	{
		report[1] = 0xEA & 0xFF;
		report[2] = (0xEA >> 8) & 0xFF;
	}

    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, sizeof(report));
    HAL_Delay(10);

    // Release 
    report[1] = 0x00;
	report[2] = 0x00;
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, sizeof(report));
    HAL_Delay(10);
}


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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  SSD1306_Init();
  DS1307_Init(&hi2c1);
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val, ADC_BUFF_LEN);
  
//  time_data.hour = 15;
//  time_data.min = 32;
//  time_data.sec = 30;
//  time_data.date = 25;
//  time_data.month = 9;
//  time_data.year = 25;
//  DS1307_write(&time_data);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Matrix_handle(&button_matrix);
	  static uint32_t time = 0;
	  if(HAL_GetTick() - time > 100)
	  {
		  HID_VolumeControl();
		  Oled_display();
		  time = HAL_GetTick();
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ROW1_out_GPIO_Port, ROW1_out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ROW2_out_Pin|ROW3_out_Pin|ROW4_out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : COL4_in_Pin COL3_in_Pin COL2_in_Pin */
  GPIO_InitStruct.Pin = COL4_in_Pin|COL3_in_Pin|COL2_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : COL1_in_Pin */
  GPIO_InitStruct.Pin = COL1_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(COL1_in_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ROW1_out_Pin */
  GPIO_InitStruct.Pin = ROW1_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ROW1_out_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW2_out_Pin ROW3_out_Pin ROW4_out_Pin */
  GPIO_InitStruct.Pin = ROW2_out_Pin|ROW3_out_Pin|ROW4_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
