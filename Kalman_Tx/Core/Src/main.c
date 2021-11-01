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
//#include "mpu6050.h"
#include <stdio.h>
#include <math.h>
#include "ssd1306.h"
#include "MY_NRF24.h"
#include "MPU6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 * http://samselectronicsprojects.blogspot.com/2014/07/getting-roll-pitch-and-yaw-from-mpu-6050.html
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//MPU6050_t MPU6050;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*nrf24l01*/
uint64_t TxpipeAddrs = 122;
char myTxData[32] ;
uint8_t Tx_check = 0;
MPU6050_t MPU6050;

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init(&hi2c3);	//OLED
  MPU6050_Init(&hi2c1); //MPU

  /***********NRF Ayarlari****************/
    NRF24_begin(GPIOA, NRFL24L01_CSN_Pin, NRF24L01_CS_Pin, hspi1);
  	NRF24_stopListening();
  	NRF24_openWritingPipe(TxpipeAddrs);
  	NRF24_setAutoAck(false);
  	NRF24_setChannel(34);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#define PI 3.1415926535897932384626433832795
#define RAD_TO_DEG  57.295779513082320876798154814105

  	int minVal=265;
  	int maxVal=402;
  	char buffx[10];
  	char buffy[10];
  	char buffz[10];

  	double x;
  	double y;
  	double z;
  	uint8_t ort = 0;

  	while(1)
  	{
  		MPU6050_Read_Accel(&hi2c1, &MPU6050);
  		ort++;
		int xAng = map(MPU6050.Accel_X_RAW,minVal,maxVal,-90,90);
		int yAng = map(MPU6050.Accel_Y_RAW,minVal,maxVal,-90,90);
		int zAng = map(MPU6050.Accel_Z_RAW,minVal,maxVal,-90,90);
		int s = sqrt(yAng*yAng+zAng*zAng);
		int a = sqrt(xAng*xAng+zAng*zAng);
		x= RAD_TO_DEG * (atan2(yAng, a));
		y= RAD_TO_DEG * (atan2(xAng,s ));
		z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
		if(ort == 100)
		{
			sprintf(buffx,"%0.0f",x);

			sprintf(buffy,"%0.0f",y);
			sprintf(buffz,"%0.0f",z);
			myTxData[1]=0XFF;
			myTxData[2]=buffx[0];
			myTxData[3]=buffx[1];
			myTxData[4]=buffx[2];
			myTxData[5]=buffx[3];
			myTxData[6]=buffy[0];
			myTxData[7]=buffy[1];
			myTxData[8]=buffy[2];
			myTxData[9]=buffy[3];
			myTxData[10]=0XFF;
            for(uint8_t i = 1 ; i<11 ; i++)
			{
            	Tx_check = Tx_check + myTxData[i];
			}
			myTxData[0] = Tx_check;
			Tx_check = 0;
			HAL_Delay(100);
			NRF24_write(myTxData, 32);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			ort = 0;
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
