/*
 * RGB_sensor.c
 *
 *  Created on: Nov 12, 2021
 *      Author: jerem
 */

#include "RGB_sensor.h"

uint8_t RGB_Init_SetParamGPIOs(RGB_struct* RGB_Sensor, GPIO_TypeDef* OutputEnable_GPIOx, uint16_t OutputEnable_GPIO_Pin,
		GPIO_TypeDef* LED_GPIOx, uint16_t LED_GPIO_Pin){
	RGB_Sensor->OutputEnable_GPIOx = OutputEnable_GPIOx;
	RGB_Sensor->OutputEnable_GPIO_Pin = OutputEnable_GPIO_Pin;
	RGB_Sensor->LED_GPIOx = LED_GPIOx;
	RGB_Sensor->LED_GPIO_Pin = LED_GPIO_Pin;

	return 0;
}

uint8_t RGB_Init_SetOutFreqGPIOs(RGB_struct* RGB_Sensor, GPIO_TypeDef* OutFreq1_GPIOx, uint16_t OutFreq1_GPIO_Pin,
		GPIO_TypeDef* OutFreq2_GPIOx, uint16_t OutFreq2_GPIO_Pin){
	RGB_Sensor->OutFreq1_GPIOx = OutFreq1_GPIOx;
	RGB_Sensor->OutFreq1_GPIO_Pin = OutFreq1_GPIO_Pin;
	RGB_Sensor->OutFreq2_GPIOx = OutFreq2_GPIOx;
	RGB_Sensor->OutFreq2_GPIO_Pin = OutFreq2_GPIO_Pin;

	return 0;
}

uint8_t RGB_Init_SetColorFilterGPIOs(RGB_struct* RGB_Sensor, GPIO_TypeDef* ColorFilter1_GPIOx, uint16_t ColorFilter1_GPIO_Pin,
		GPIO_TypeDef* ColorFilter2_GPIOx, uint16_t ColorFilter2_GPIO_Pin){
	RGB_Sensor->ColorFilter1_GPIOx = ColorFilter1_GPIOx;
	RGB_Sensor->ColorFilter1_GPIO_Pin = ColorFilter1_GPIO_Pin;
	RGB_Sensor->ColorFilter2_GPIOx = ColorFilter2_GPIOx;
	RGB_Sensor->ColorFilter2_GPIO_Pin = ColorFilter2_GPIO_Pin;

	return 0;
}

uint8_t RGB_Init(RGB_struct* RGB_Sensor){
	uint8_t FULL_RANGE = 100;

	// OE : Output Enable -> DISABLE
	HAL_GPIO_WritePin(RGB_Sensor->OutputEnable_GPIOx, RGB_Sensor->OutputEnable_GPIO_Pin, GPIO_PIN_SET);
	HAL_Delay(200);

	RGB_SetOFscaling(RGB_Sensor, FULL_RANGE);
	RGB_SetFilter(RGB_Sensor, RGB_RED);

	// Turn on the LEDs to lights the floor
	HAL_GPIO_WritePin(RGB_Sensor->LED_GPIOx, RGB_Sensor->LED_GPIO_Pin, GPIO_PIN_SET);

	// OE : Output Enable -> ENABLE
	HAL_Delay(200);
	HAL_GPIO_WritePin(RGB_Sensor->OutputEnable_GPIOx, RGB_Sensor->OutputEnable_GPIO_Pin, GPIO_PIN_RESET);

	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);

	return 0;
}

uint8_t RGB_SetFilter(RGB_struct* RGB_Sensor, uint8_t color){
	uint8_t status = RGB_ERROR_NONE;

	switch(color)
	{
	case RGB_RED:
		printf("Filtre ROUGE\r\n");
		HAL_GPIO_WritePin(RGB_Sensor->ColorFilter1_GPIOx, RGB_Sensor->ColorFilter1_GPIO_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RGB_Sensor->ColorFilter2_GPIOx, RGB_Sensor->ColorFilter2_GPIO_Pin, GPIO_PIN_RESET);
		break;
	case RGB_BLUE:
		printf("Filtre BLEU\r\n");
		HAL_GPIO_WritePin(RGB_Sensor->ColorFilter1_GPIOx, RGB_Sensor->ColorFilter1_GPIO_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RGB_Sensor->ColorFilter2_GPIOx, RGB_Sensor->ColorFilter2_GPIO_Pin, GPIO_PIN_SET);
		break;
	case RGB_GREEN:
		printf("Filtre VERT\r\n");
		HAL_GPIO_WritePin(RGB_Sensor->ColorFilter1_GPIOx, RGB_Sensor->ColorFilter1_GPIO_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RGB_Sensor->ColorFilter2_GPIOx, RGB_Sensor->ColorFilter2_GPIO_Pin, GPIO_PIN_SET);
		break;
	case RGB_CLEAR:
		printf("Filtre OFF\r\n");
		HAL_GPIO_WritePin(RGB_Sensor->ColorFilter1_GPIOx, RGB_Sensor->ColorFilter1_GPIO_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RGB_Sensor->ColorFilter2_GPIOx, RGB_Sensor->ColorFilter2_GPIO_Pin, GPIO_PIN_RESET);
		break;
	default:
		status = RGB_ERROR_FILTER;
		printf("RGB - Error setFilter : %d\r\n", status);
	}

	return status;
}

uint8_t RGB_SetOFscaling(RGB_struct* RGB_Sensor, uint8_t scale){
	uint8_t status = RGB_ERROR_NONE;

	switch(scale)
	{
	case 0:
		printf("fo : power down\r\n");
		HAL_GPIO_WritePin(RGB_Sensor->OutFreq1_GPIOx, RGB_Sensor->OutFreq1_GPIO_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RGB_Sensor->OutFreq2_GPIOx, RGB_Sensor->OutFreq2_GPIO_Pin, GPIO_PIN_RESET);
		break;
	case 2:
		printf("fo : 2\r\n");
		HAL_GPIO_WritePin(RGB_Sensor->OutFreq1_GPIOx, RGB_Sensor->OutFreq1_GPIO_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RGB_Sensor->OutFreq2_GPIOx, RGB_Sensor->OutFreq2_GPIO_Pin, GPIO_PIN_SET);
		break;
	case 20:
		printf("fo : 20\r\n");
		HAL_GPIO_WritePin(RGB_Sensor->OutFreq1_GPIOx, RGB_Sensor->OutFreq1_GPIO_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RGB_Sensor->OutFreq2_GPIOx, RGB_Sensor->OutFreq2_GPIO_Pin, GPIO_PIN_RESET);
		break;
	case 100:
		printf("fo : 100\r\n");
		HAL_GPIO_WritePin(RGB_Sensor->OutFreq1_GPIOx, RGB_Sensor->OutFreq1_GPIO_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RGB_Sensor->OutFreq2_GPIOx, RGB_Sensor->OutFreq2_GPIO_Pin, GPIO_PIN_SET);
		break;
	default:
		status = RGB_ERROR_OF_SCALING;
		printf("RGB - Error setOFscaling : %d\r\n", status);
	}

	return status;
}

uint16_t RGB_GetFreq(){
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



