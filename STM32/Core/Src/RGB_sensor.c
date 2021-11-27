/*
 * RGB_sensor.c
 *
 *  Created on: Nov 12, 2021
 *      Author: jerem
 */

#include "RGB_sensor.h"


uint8_t RGB_init(){
	printf("\r\n ------------------------   RGB   ------------------------ \r\n");
	uint8_t FULL_RANGE = 100;

	// OE : Output Enable -> DISABLE
	HAL_GPIO_WritePin(RGB_OE_GPIO_Port, RGB_OE_Pin, GPIO_PIN_SET);
	HAL_Delay(200);

	RGB_setOFscaling(FULL_RANGE);
	RGB_setFilter(RGB_RED);

	// Allume les LEDs pour éclairer le sol
	HAL_GPIO_WritePin(RGB_LED_GPIO_Port, RGB_LED_Pin, GPIO_PIN_SET);

	// OE : Output Enable -> ENABLE
	HAL_Delay(200);
	HAL_GPIO_WritePin(RGB_OE_GPIO_Port, RGB_OE_Pin, GPIO_PIN_RESET);

	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);

	return 0;
}

uint8_t RGB_setFilter(uint8_t color){
	uint8_t status = RGB_ERROR_NONE;

	switch(color)
	{
	case RGB_RED:
		printf("Filtre ROUGE\r\n");
		HAL_GPIO_WritePin(RGB_S2_GPIO_Port, RGB_S2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RGB_S3_GPIO_Port, RGB_S3_Pin, GPIO_PIN_RESET);
		break;
	case RGB_BLUE:
		printf("Filtre BLEU\r\n");
		HAL_GPIO_WritePin(RGB_S2_GPIO_Port, RGB_S2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RGB_S3_GPIO_Port, RGB_S3_Pin, GPIO_PIN_SET);
		break;
	case RGB_GREEN:
		printf("Filtre VERT\r\n");
		HAL_GPIO_WritePin(RGB_S2_GPIO_Port, RGB_S2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RGB_S3_GPIO_Port, RGB_S3_Pin, GPIO_PIN_SET);
		break;
	case RGB_CLEAR:
		printf("Filtre OFF\r\n");
		HAL_GPIO_WritePin(RGB_S2_GPIO_Port, RGB_S2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RGB_S3_GPIO_Port, RGB_S3_Pin, GPIO_PIN_RESET);
		break;
	default:
		status = RGB_ERROR_FILTER;
		printf("RGB - Error setFilter : %d\r\n", status);
	}

	return status;
}

uint8_t RGB_setOFscaling(uint8_t scale){
	uint8_t status = RGB_ERROR_NONE;

	switch(scale)
	{
	case 0:
		printf("fo : power down\r\n");
		HAL_GPIO_WritePin(RGB_S0_GPIO_Port, RGB_S0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RGB_S1_GPIO_Port, RGB_S1_Pin, GPIO_PIN_RESET);
		break;
	case 2:
		printf("fo : 2\r\n");
		HAL_GPIO_WritePin(RGB_S0_GPIO_Port, RGB_S0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RGB_S1_GPIO_Port, RGB_S1_Pin, GPIO_PIN_SET);
		break;
	case 20:
		printf("fo : 20\r\n");
		HAL_GPIO_WritePin(RGB_S0_GPIO_Port, RGB_S0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RGB_S1_GPIO_Port, RGB_S1_Pin, GPIO_PIN_RESET);
		break;
	case 100:
		printf("fo : 100\r\n");
		HAL_GPIO_WritePin(RGB_S0_GPIO_Port, RGB_S0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RGB_S1_GPIO_Port, RGB_S1_Pin, GPIO_PIN_SET);
		break;
	default:
		status = RGB_ERROR_OF_SCALING;
		printf("RGB - Error setOFscaling : %d\r\n", status);
	}

	return status;
}

uint16_t RGB_getFreq(){
	// TIM3 : CH1

	return 0;
}

#define APBCLOCK 42000000
#define PRESCALER 42

static uint16_t IC_Val1 = 0;
static uint16_t IC_Val2 = 0;
static uint16_t difference = 0;
static uint8_t isFirstCaptured = 0;

float frequency = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		if(isFirstCaptured == 0){
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			isFirstCaptured = 1;
		}
		else{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			difference = IC_Val2 - IC_Val1;
			float refClock = APBCLOCK/(PRESCALER);
			frequency = refClock/difference;
			__HAL_TIM_SET_COUNTER(htim, 0);
			isFirstCaptured = 0;
		}
	}
}



