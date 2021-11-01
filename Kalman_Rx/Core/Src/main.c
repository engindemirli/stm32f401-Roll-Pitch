/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include <stdio.h>
#include <math.h>
#include "ssd1306.h"
#include "MY_NRF24.h"
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

/**********NRF24L01 variable**********/
uint64_t RxpipeAddrs = 122   ; /*nrf24l01 RX address*/
char RxData[50]              ; /*nrd24l01 RX data buffer*/
uint8_t RX_Check     = 0     ; /*nrd24l01 RX data Checksum*/
bool Data_Correct    = false ; /*nrd24l01 RX is data correct flag*/

/**********OLED variable**********/
char buff_pitch[4] ; /*pitch value in string format*/
float Pitch   ; /*pitch value in float format*/
char buff_roll[4] ; /*roll value in string format*/
float Roll    ;	/*roll value in float format*/
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
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

/*********Initialize OLED************/
ssd1306_Init(&hi2c3);

/***********Initialize NRF****************/
NRF24_begin(GPIOA, NRFL24L01_CSN_Pin, NRF24L01_CS_Pin, hspi1);
nrf24_DebugUART_Init(huart1);
NRF24_setAutoAck(false);
NRF24_setChannel(34);
NRF24_openReadingPipe(1, RxpipeAddrs);
NRF24_startListening();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  /* If the data available, reads data and checks is format true*/
	  if(NRF24_available())
	  {
		  NRF24_read(RxData, 32);
	  	  HAL_UART_Transmit(&huart1, (uint8_t *)RxData, 32, 10);

	  	  if(RxData[1] == 0xFF && RxData[10] == 0XFF)
	  	  {
	  		  for(uint8_t i = 1 ; i<11 ; i++)
	  		  {
	  			  RX_Check = RX_Check + RxData[i];
	  		  }
	  		  if(RX_Check == RxData[0])
	  		  {
	  			  Data_Correct = true ;
	  			  RX_Check     = 0    ;
	  		  }
	  	  }

	  }
	  if(Data_Correct == true) /*if the RX data is suitable for data structure */
	  {
		  /****Assign THE RX data to relevant buffer*****/
		  buff_pitch[0] = RxData[2] ;
		  buff_pitch[1] = RxData[3] ;
		  buff_pitch[2] = RxData[4] ;
		  buff_pitch[3] = RxData[5] ;

		  buff_roll[0]  = RxData[6] ;
		  buff_roll[1]  = RxData[7] ;
		  buff_roll[2]  = RxData[8] ;
		  buff_roll[3]  = RxData[9] ;

		  Pitch = atof(buff_pitch) ;
		  Roll  = atof(buff_roll)  ;

		  /*****write the data to OLED*******/
		  ssd1306_Fill(Black);
		  ssd1306_SetCursor(0, 0);
		  ssd1306_WriteString("ON  :", Font_10x14, White);
		  ssd1306_SetCursor(0, 16);
		  ssd1306_WriteString("ARKA:", Font_10x14, White);
		  ssd1306_SetCursor(0, 32);
		  ssd1306_WriteString("SAG :", Font_10x14, White);
		  ssd1306_SetCursor(0, 48);
		  ssd1306_WriteString("SOL :", Font_10x14, White);

		  ssd1306_SetCursor(78, 0);
		  ssd1306_WriteString(buff_pitch, Font_10x14, White);
		  sprintf(buff_pitch,"%0.0f",(Pitch*-1.0F));
		  ssd1306_SetCursor(78, 16);
		  ssd1306_WriteString(buff_pitch, Font_10x14, White);


		  ssd1306_SetCursor(78, 32);
		  ssd1306_WriteString(buff_roll, Font_10x14, White);

		  Roll = Roll * (-1.0f);
		  sprintf(buff_roll,"%0.0f",Roll);
		  ssd1306_SetCursor(78, 48);
		  ssd1306_WriteString(buff_roll, Font_10x14, White);



		  ssd1306_UpdateScreen(&hi2c3);
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

		  Data_Correct = false ;
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
