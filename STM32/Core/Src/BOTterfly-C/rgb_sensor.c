/*
 * rgb_sensor.c
 *
 *  Created on: 8 déc. 2021
 *      Author: jerem
 */

#include "BOTterfly-H/rgb_sensor.h"
#include "BOTterfly-H/config.h"


uint8_t RGB_Init_SetTimer(RGB_struct* rgbSensor, TIM_HandleTypeDef *htim, uint32_t Channel){
	rgbSensor->Timer_Handle = htim;
	rgbSensor->Timer_Channel = Channel;

	HAL_TIM_IC_Start_IT(rgbSensor->Timer_Handle, rgbSensor->Timer_Channel);
	return 0;
}

uint8_t RGB_Init_SetParamGPIOs(RGB_struct* rgbSensor, GPIO_TypeDef* OutputEnable_GPIOx, uint16_t OutputEnable_GPIO_Pin,
		GPIO_TypeDef* LED_GPIOx, uint16_t LED_GPIO_Pin){
	rgbSensor->OutputEnable_GPIOx = OutputEnable_GPIOx;
	rgbSensor->OutputEnable_GPIO_Pin = OutputEnable_GPIO_Pin;
	rgbSensor->LED_GPIOx = LED_GPIOx;
	rgbSensor->LED_GPIO_Pin = LED_GPIO_Pin;

	return 0;
}

uint8_t RGB_Init_SetOutFreqGPIOs(RGB_struct* rgbSensor, GPIO_TypeDef* OutFreq1_GPIOx, uint16_t OutFreq1_GPIO_Pin,
		GPIO_TypeDef* OutFreq2_GPIOx, uint16_t OutFreq2_GPIO_Pin){
	rgbSensor->OutFreq1_GPIOx = OutFreq1_GPIOx;
	rgbSensor->OutFreq1_GPIO_Pin = OutFreq1_GPIO_Pin;
	rgbSensor->OutFreq2_GPIOx = OutFreq2_GPIOx;
	rgbSensor->OutFreq2_GPIO_Pin = OutFreq2_GPIO_Pin;

	return 0;
}

uint8_t RGB_Init_SetColorFilterGPIOs(RGB_struct* rgbSensor, GPIO_TypeDef* ColorFilter1_GPIOx, uint16_t ColorFilter1_GPIO_Pin,
		GPIO_TypeDef* ColorFilter2_GPIOx, uint16_t ColorFilter2_GPIO_Pin){
	rgbSensor->ColorFilter1_GPIOx = ColorFilter1_GPIOx;
	rgbSensor->ColorFilter1_GPIO_Pin = ColorFilter1_GPIO_Pin;
	rgbSensor->ColorFilter2_GPIOx = ColorFilter2_GPIOx;
	rgbSensor->ColorFilter2_GPIO_Pin = ColorFilter2_GPIO_Pin;

	return 0;
}

uint8_t RGB_Init(RGB_struct* rgbSensor){

	// OE : Output Enable -> DISABLE
	HAL_GPIO_WritePin(rgbSensor->OutputEnable_GPIOx, rgbSensor->OutputEnable_GPIO_Pin, GPIO_PIN_SET);
	HAL_Delay(200);

	RGB_SetOFscaling(rgbSensor, RGB_OF_02_RANGE);
	RGB_SetFilter(rgbSensor, RGB_RED);

	// Turn on the LEDs to lights the floor
	HAL_GPIO_WritePin(rgbSensor->LED_GPIOx, rgbSensor->LED_GPIO_Pin, GPIO_PIN_SET);

	// OE : Output Enable -> ENABLE
	HAL_Delay(200);
	HAL_GPIO_WritePin(rgbSensor->OutputEnable_GPIOx, rgbSensor->OutputEnable_GPIO_Pin, GPIO_PIN_RESET);

	//HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);

	return 0;
}

uint8_t RGB_SetFilter(RGB_struct* rgbSensor, uint8_t color){
	uint8_t status = RGB_ERROR_NONE;

	switch(color)
	{
	case RGB_RED:
		//printf("Filtre ROUGE\r\n");
		HAL_GPIO_WritePin(rgbSensor->ColorFilter1_GPIOx, rgbSensor->ColorFilter1_GPIO_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(rgbSensor->ColorFilter2_GPIOx, rgbSensor->ColorFilter2_GPIO_Pin, GPIO_PIN_RESET);
		break;
	case RGB_BLUE:
		//printf("Filtre BLEU\r\n");
		HAL_GPIO_WritePin(rgbSensor->ColorFilter1_GPIOx, rgbSensor->ColorFilter1_GPIO_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(rgbSensor->ColorFilter2_GPIOx, rgbSensor->ColorFilter2_GPIO_Pin, GPIO_PIN_SET);
		break;
	case RGB_GREEN:
		//printf("Filtre VERT\r\n");
		HAL_GPIO_WritePin(rgbSensor->ColorFilter1_GPIOx, rgbSensor->ColorFilter1_GPIO_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(rgbSensor->ColorFilter2_GPIOx, rgbSensor->ColorFilter2_GPIO_Pin, GPIO_PIN_SET);
		break;
	case RGB_CLEAR:
		//printf("Filtre OFF\r\n");
		HAL_GPIO_WritePin(rgbSensor->ColorFilter1_GPIOx, rgbSensor->ColorFilter1_GPIO_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(rgbSensor->ColorFilter2_GPIOx, rgbSensor->ColorFilter2_GPIO_Pin, GPIO_PIN_RESET);
		break;
	default:
		status = RGB_ERROR_FILTER;
		printf("RGB - Error setFilter : %d\r\n", status);
	}

	return status;
}

uint8_t RGB_SetOFscaling(RGB_struct* rgbSensor, uint8_t scale){
	uint8_t status = RGB_ERROR_NONE;

	switch(scale)
	{
	case 0:
		printf("fo : power down\r\n");
		HAL_GPIO_WritePin(rgbSensor->OutFreq1_GPIOx, rgbSensor->OutFreq1_GPIO_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(rgbSensor->OutFreq2_GPIOx, rgbSensor->OutFreq2_GPIO_Pin, GPIO_PIN_RESET);
		break;
	case 2:
		printf("fo : 2\r\n");
		HAL_GPIO_WritePin(rgbSensor->OutFreq1_GPIOx, rgbSensor->OutFreq1_GPIO_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(rgbSensor->OutFreq2_GPIOx, rgbSensor->OutFreq2_GPIO_Pin, GPIO_PIN_SET);
		break;
	case 20:
		printf("fo : 20\r\n");
		HAL_GPIO_WritePin(rgbSensor->OutFreq1_GPIOx, rgbSensor->OutFreq1_GPIO_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(rgbSensor->OutFreq2_GPIOx, rgbSensor->OutFreq2_GPIO_Pin, GPIO_PIN_RESET);
		break;
	case 100:
		printf("fo : 100\r\n");
		HAL_GPIO_WritePin(rgbSensor->OutFreq1_GPIOx, rgbSensor->OutFreq1_GPIO_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(rgbSensor->OutFreq2_GPIOx, rgbSensor->OutFreq2_GPIO_Pin, GPIO_PIN_SET);
		break;
	default:
		status = RGB_ERROR_OF_SCALING;
		printf("RGB - Error setOFscaling : %d\r\n", status);
	}

	return status;
}



static uint16_t icVal1 = 0;
static uint16_t icVal2 = 0;
static uint16_t difference = 0;
static uint8_t isFirstCaptured = 0;

float frequency = 0;
uint8_t colorFilter = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		if(isFirstCaptured == 0){
			icVal1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			isFirstCaptured = 1;
		}
		else{
			icVal2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			difference = icVal2 - icVal1;
			float refClock = RGB_APBCLOCK/(RGB_PRESCALER);

			frequency = refClock/difference;
			__HAL_TIM_SET_COUNTER(htim, 0);

			/**********************************************************/
			switch(colorFilter)
			{
			case RGB_RED:
				RGB_Sensor.red = (uint16_t)frequency;
				colorFilter = RGB_GREEN;
				RGB_SetFilter(&RGB_Sensor, colorFilter);
				break;
			case RGB_GREEN:
				RGB_Sensor.green = (uint16_t)frequency;
				colorFilter = RGB_BLUE;
				RGB_SetFilter(&RGB_Sensor, colorFilter);
				break;
			case RGB_BLUE:
				RGB_Sensor.blue = (uint16_t)frequency;
				colorFilter = RGB_RED;
				RGB_SetFilter(&RGB_Sensor, colorFilter);
				break;
			default:
				printf("RGB - Error setFilter\r\n");
			}
			if((RGB_Sensor.red >= (1.5*RGB_Sensor.green)) & (RGB_Sensor.red >= (1.5*RGB_Sensor.blue))){
				RGB_Sensor.isFloorRed = 1;
			}else{
				RGB_Sensor.isFloorRed = 0;
			}
			/**********************************************************/

			isFirstCaptured = 0;
		}
	}
}




