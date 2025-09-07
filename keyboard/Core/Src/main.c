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
#include "stdbool.h"
#include "usbd_hid.h"
#include "fonts.h"
#include "ssd1306.h"
#include "DS1307.h"

#define ADC_BUFF_LEN 32		// sample to moving average filter
#define ADC_THRESHOLD 80 // threshold for changing %volume
#define VOLUME_MAX 100	//MAX VOLUME

extern USBD_HandleTypeDef hUsbDeviceFS;

uint32_t KB_Scan();
void Key_board_send_report(uint32_t pressed_key);
long map(long x, long in_min, long in_max, long out_min, long out_max);

uint32_t key;
uint16_t adc_val[ADC_BUFF_LEN];
uint8_t volume = 0;
uint16_t volume_now, volume_old = 0;

char buff_time[20];
char buff_day[20];

//===================Time==================
DS1307_Typedef time_data;

//=========================key press handle=============
typedef enum
{
	KEY_PrtSc = 1, //0x46
	KEY_CUT = 2, // 0x02 + 0x1B
	KEY_COPY = 3, // 0x02 + 0x06
	KEY_PASTE = 4, // 0x02 + 0x19
	KEY_MUTE = 5,
}Macro_key;


typedef struct
{
	uint8_t id;
	uint8_t modifier;
	uint8_t reserved;
	uint8_t keycode1;
	uint8_t keycode2;
	uint8_t keycode3;
	uint8_t keycode4;
	uint8_t keycode5;
	uint8_t keycode6;	
}Keyboard_Report_Des;

Keyboard_Report_Des hid_keyboard = {0x01,0,0,0,0,0,0,0,0};

uint32_t KB_Scan()	// Scan button
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5, 1);
	if(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8))
	{
		return KEY_PrtSc;
	}
	if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15))
	{
		return KEY_CUT;
	}
	if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14))
	{
		return KEY_COPY;
	}
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_5, 1);
	if(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8))
	{
		return KEY_PASTE;
	}
	if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15))
	{
		return KEY_MUTE;
	}
	if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14))
	{
		return 6;
	}
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4, 1);
	if(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8))
	{
		return 7;
	}
	if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15))
	{
		return 8;
	}
	if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14))
	{
		return 9;
	}
	return 0;
}

void Send_key(uint8_t modifier, uint8_t keycode)
{
	hid_keyboard.modifier = modifier; // Ctrl, shift, alt
	hid_keyboard.keycode1 = keycode; // copy, paste, cut, ...
	

    USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&hid_keyboard, sizeof(hid_keyboard));
    HAL_Delay(50);

	hid_keyboard.modifier = 0x00; // Ctrl, shift, alt
	hid_keyboard.keycode1 = 0x00; // copy, paste, cut, ...
    USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&hid_keyboard, sizeof(hid_keyboard));
    HAL_Delay(50);
}

void Send_consumer(uint8_t keycode)
{
	uint8_t consumer_report[2] = {0x02, 0x00};

	consumer_report[1] = keycode;
	USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&consumer_report, sizeof(consumer_report));
    HAL_Delay(50);
	
	consumer_report[1] = 0x00; // copy, paste, cut, ...
    USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&consumer_report, sizeof(consumer_report));
    HAL_Delay(50);
}


void Keyboard_Handle(void)
{
	uint32_t key = KB_Scan();
	switch(key)
	{
		case KEY_PrtSc:
			Send_key(0x00, 0x46);
			break;
		case KEY_CUT: // num 2
			Send_key(0x01, 0x1B);
			break;
		case KEY_COPY: //num 3
			Send_key(0x01,0x06);
			break;
		case KEY_PASTE: // num 4
			Send_key(0x01,0x19);
			break;
		case KEY_MUTE:	// num 5
			Send_consumer(0x04);
			break;
		default:
			break;
			
	}
}
//==========================Display handle==============
const char dow_arr[][10] = {{"SUNDAY"},{"MONDAY"},{"TUESDAY"},{"WEDNESDAY"},
							{"THURSDAY"},{"FRIDAY"},{"SATURDAY"}}; //day of week 

void Oled_display() // display date, month, year and keyboard map
{
	DS1307_read(&time_data);
	
	sprintf(buff_time, "%02d : %02d : %02d", time_data.hour, time_data.min, time_data.sec);
    SSD1306_GotoXY(0, 0);
    SSD1306_Puts(buff_time, &Font_7x10, 1);
	
    SSD1306_GotoXY(0, 20);
    sprintf(buff_day, "%02d / %02d / 20%02d", time_data.date, time_data.month, time_data.year);
    SSD1306_Puts(buff_day, &Font_7x10, 1);
	
	uint8_t dow = DS1307_get_day_of_week(&time_data); // GET DAY
	SSD1306_GotoXY(0, 40);
    sprintf(buff_day, "%s", dow_arr[dow]);
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
	 // Report ID (0x02) + 1 byte data
    uint8_t report[2];

    report[0] = 0x02;   // Report ID
    report[1] = 0x00;   // default release
	
	if(adc_val[0] < 30) adc_val[0] = 0;
	if(adc_val[0] > 4050) adc_val[0] = 4050;
	
    if(is_adjust == 1)   // Volume Up
	{
		report[1] = 0x01;
	}		
        
    else if(is_adjust == 2)  // Volume Down
	{
		report[1] = 0x02;
	}

    USBD_HID_SendReport(&hUsbDeviceFS, report, sizeof(report));
    HAL_Delay(10);

    // Release 
    report[1] = 0x00;
    USBD_HID_SendReport(&hUsbDeviceFS, report, sizeof(report));
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
  
//  time_data.hour = 14;
//  time_data.min = 40;
//  time_data.sec = 50;
//  time_data.date = 5;
//  time_data.month = 9;
//  time_data.year = 25;
//  DS1307_write(&time_data);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  
	  // Receive ADC value: 0 - 4095 map 0 - 100
	Keyboard_Handle();
	static uint32_t time = 0;
	volume_now = adc_val[0] * 100 / 4095;
	if(HAL_GetTick() - time > 20)
	{
		Oled_display();
 
		if (volume_now > volume_old + 1)   
		{
			HID_VolumeControl(); 
			volume_old++;         
		}
		else if (volume_now + 1 < volume_old) 
		{
			HID_VolumeControl(); 
			volume_old--;         
		}
		time = HAL_GetTick();
	}

  }
  /* USER A END 3 */
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
  HAL_GPIO_WritePin(GPIOB, ROW1_Pin|ROW2_Pin|ROW3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : COL3_Pin COL2_Pin */
  GPIO_InitStruct.Pin = COL3_Pin|COL2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : COL1_Pin */
  GPIO_InitStruct.Pin = COL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(COL1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW1_Pin ROW2_Pin ROW3_Pin */
  GPIO_InitStruct.Pin = ROW1_Pin|ROW2_Pin|ROW3_Pin;
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
