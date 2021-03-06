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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BOTterfly-H/config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define nbOfSensors 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//VL53L0X_Dev_t myDevice[nbOfSensors];

char msg[1];
int it_uart = 0;
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
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	// Liaison s??rie
	HAL_UART_Receive_IT(&huart2,(uint8_t*)msg,1);

	printf("\r\n --------------------- TIME OF FLIGHT --------------------- \r\n");

	TOF_Init_SetI2C(&TOF_Sensor[0], &hi2c1, 0x55);
	TOF_Init_SetGPIOs(&TOF_Sensor[0], XSHUT0_GPIO_Port, XSHUT0_Pin, tof_0_GPIO_Port, tof_0_Pin);
	TOF_Init_SetEXTI(&TOF_Sensor[0], tof_0_EXTI_IRQn);

	TOF_Init_SetI2C(&TOF_Sensor[1], &hi2c1, 0x58);
	TOF_Init_SetGPIOs(&TOF_Sensor[1], XSHUT1_GPIO_Port, XSHUT1_Pin, tof_1_GPIO_Port, tof_1_Pin);
	TOF_Init_SetEXTI(&TOF_Sensor[1], tof_1_EXTI_IRQn);

	TOF_Init_SetI2C(&TOF_Sensor[2], &hi2c1, 0x5b);
	TOF_Init_SetGPIOs(&TOF_Sensor[2], XSHUT2_GPIO_Port, XSHUT2_Pin, tof_2_GPIO_Port, tof_2_Pin);
	TOF_Init_SetEXTI(&TOF_Sensor[2], tof_2_EXTI_IRQn);

	TOF_Init(TOF_Sensor);


	printf("\r\n ---------------------   RGB SENSOR   --------------------- \r\n");

	RGB_Init_SetTimer(&RGB_Sensor, &htim3, TIM_CHANNEL_1);

	RGB_Init_SetParamGPIOs(&RGB_Sensor, RGB_OE_GPIO_Port, RGB_OE_Pin, RGB_LED_GPIO_Port, RGB_LED_Pin);
	RGB_Init_SetOutFreqGPIOs(&RGB_Sensor, RGB_S0_GPIO_Port, RGB_S0_Pin, RGB_S1_GPIO_Port, RGB_S1_Pin);
	RGB_Init_SetColorFilterGPIOs(&RGB_Sensor, RGB_S2_GPIO_Port, RGB_S2_Pin, RGB_S3_GPIO_Port, RGB_S3_Pin);

	RGB_Init(&RGB_Sensor);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		if(it_uart){
			if(msg[0] == '\r'){
				printf("\r\n");
			}else{
				HAL_UART_Transmit(&huart2, (uint8_t*)msg, 1, HAL_MAX_DELAY);
			}
			it_uart = 0;
		}

		printf("\r\n ----------------------- RGB SENSOR ----------------------- \r\n");
		if(RGB_Sensor.isFloorRed){
			printf("The floor is red\r\n");
		}else{
			printf("The floor is not red\r\n");
		}
		printf("RED : %d\r\n", RGB_Sensor.red);
		printf("GREEN : %d\r\n", RGB_Sensor.green);
		printf("BLUE : %d\r\n", RGB_Sensor.blue);

		printf("\r\n ----------------------- TOF SENSORS ----------------------- \r\n");
		printf("Tof 0 - DISTANCE = %d mm\r\n",TOF_Sensor[0].rangeMillimeter);
		printf("Tof 1 - DISTANCE = %d mm\r\n",TOF_Sensor[1].rangeMillimeter);
		printf("Tof 2 - DISTANCE = %d mm\r\n",TOF_Sensor[2].rangeMillimeter);

		HAL_Delay(250);

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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 80;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
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
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(&huart2,(uint8_t*)msg,1);
	if(huart->Instance == USART2){
		it_uart = 1;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
