/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "i2c.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include "i2c_techmaker_sm.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

	#define ADR_I2C_DS3231 0x68

	char DataChar[100];
	uint32_t counter_u32 = 0;

	RTC_TimeTypeDef TimeSt;
	RTC_DateTypeDef DateSt;

	uint8_t DS3231_Seconds = 0x05;
	uint8_t DS3231_Minutes = 0x26;
	uint8_t DS3231_Hours   = 0x16;
	uint8_t DS3231_WeekDay = 0x04;
	uint8_t DS3231_Date    = 0x05;
	uint8_t DS3231_Mouth   = 0x09;
	uint8_t DS3231_Year    = 0x19;

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
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	sprintf(DataChar,"\r\nDS3231_RTC_f103-19 v0.2.0\r\nUART1 for debug started on speed 115200\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

	I2Cdev_init(&hi2c1);
	I2C_ScanBusFlow(&hi2c1 , &huart1);

//	I2Cdev_writeByte( ADR_I2C_DS3231, 0x00, DS3231_Seconds );
//	I2Cdev_writeByte( ADR_I2C_DS3231, 0x01, DS3231_Minutes );
//	I2Cdev_writeByte( ADR_I2C_DS3231, 0x02, DS3231_Hours   );
//	I2Cdev_writeByte( ADR_I2C_DS3231, 0x03, DS3231_WeekDay );
//	I2Cdev_writeByte( ADR_I2C_DS3231, 0x04, DS3231_Date    );
//	I2Cdev_writeByte( ADR_I2C_DS3231, 0x05, DS3231_Mouth   );
//	I2Cdev_writeByte( ADR_I2C_DS3231, 0x06, DS3231_Year    );
//	HAL_Delay(100);


//	Number 123
//	Binary Form 01111011
//	BCD will be 0001 0010 0011


	I2Cdev_readByte( ADR_I2C_DS3231, 0x00, &DS3231_Seconds, 100);
	I2Cdev_readByte( ADR_I2C_DS3231, 0x01, &DS3231_Minutes, 100);
	I2Cdev_readByte( ADR_I2C_DS3231, 0x02, &DS3231_Hours,   100);
	I2Cdev_readByte( ADR_I2C_DS3231, 0x03, &DS3231_WeekDay, 100);
	I2Cdev_readByte( ADR_I2C_DS3231, 0x04, &DS3231_Date,    100);
	I2Cdev_readByte( ADR_I2C_DS3231, 0x05, &DS3231_Mouth,   100);
	I2Cdev_readByte( ADR_I2C_DS3231, 0x06, &DS3231_Year,    100);

	TimeSt.Hours   = (DS3231_Hours   >> 4)*10 + (DS3231_Hours   &0x0F);
	TimeSt.Minutes = (DS3231_Minutes >> 4)*10 + (DS3231_Minutes &0x0F);
	TimeSt.Seconds = (DS3231_Seconds >> 4)*10 + (DS3231_Seconds &0x0F);
	DateSt.WeekDay = (DS3231_WeekDay >> 4)*10 + (DS3231_WeekDay &0x0F);
	DateSt.Date    = (DS3231_Date    >> 4)*10 + (DS3231_Date    &0x0F);
	DateSt.Month   = (DS3231_Mouth   >> 4)*10 + (DS3231_Mouth   &0x0F);
	DateSt.Year    = (DS3231_Year    >> 4)*10 + (DS3231_Year    &0x0F);

	sprintf(DataChar,"%02d:%02d:%02d ",TimeSt.Hours,TimeSt.Minutes,TimeSt.Seconds);
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

	sprintf(DataChar,"%04d/%02d/%02d\r\n",2000+DateSt.Year, DateSt.Month, DateSt.Date);
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

	HAL_RTC_SetTime( &hrtc, &TimeSt, RTC_FORMAT_BIN );
	HAL_RTC_SetDate( &hrtc, &DateSt, RTC_FORMAT_BIN );

	HAL_Delay(500);
	HAL_RTC_GetTime( &hrtc, &TimeSt, RTC_FORMAT_BIN );
	sprintf(DataChar,"%02d:%02d:%02d ",TimeSt.Hours,TimeSt.Minutes,TimeSt.Seconds);
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

	HAL_RTC_GetDate( &hrtc, &DateSt, RTC_FORMAT_BIN );
	sprintf(DataChar,"%04d/%02d/%02d ",2000+DateSt.Year, DateSt.Month, DateSt.Date);
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

	switch(DateSt.WeekDay)
		{
		case  7: sprintf(DataChar,"Sunday"); 		break;
		case  1: sprintf(DataChar,"Monday");		break;
		case  2: sprintf(DataChar,"Tuesday"); 		break;
		case  3: sprintf(DataChar,"Wednesday");		break;
		case  4: sprintf(DataChar,"Thursday");		break;
		case  5: sprintf(DataChar,"Friday");		break;
		case  6: sprintf(DataChar,"Saturday");		break;
		default: sprintf(DataChar,"Out of day");	break;
		} // end switch Date.ST
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

	sprintf(DataChar,"\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	sprintf(DataChar,"%d) ",  (int)counter_u32 );
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	counter_u32++;

	HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);

	HAL_RTC_GetTime( &hrtc, &TimeSt, RTC_FORMAT_BIN );
	sprintf(DataChar,"%02d:%02d:%02d ",TimeSt.Hours,TimeSt.Minutes,TimeSt.Seconds);
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

	HAL_RTC_GetDate( &hrtc, &DateSt, RTC_FORMAT_BIN );
	sprintf(DataChar,"%04d/%02d/%02d ",2000+DateSt.Year, DateSt.Month, DateSt.Date);
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);

	switch(DateSt.WeekDay)
		{
		case  7: sprintf(DataChar,"Sunday\r\n"); 		break;
		case  1: sprintf(DataChar,"Monday\r\n");		break;
		case  2: sprintf(DataChar,"Tuesday\r\n"); 		break;
		case  3: sprintf(DataChar,"Wednesday\r\n");		break;
		case  4: sprintf(DataChar,"Thursday\r\n");		break;
		case  5: sprintf(DataChar,"Friday\r\n");		break;
		case  6: sprintf(DataChar,"Saturday\r\n");		break;
		default: sprintf(DataChar,"Out of day\r\n");	break;
		} // end switch Date.ST
	HAL_UART_Transmit(&huart1, (uint8_t *)DataChar, strlen(DataChar), 100);
	HAL_Delay(3000);

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
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
