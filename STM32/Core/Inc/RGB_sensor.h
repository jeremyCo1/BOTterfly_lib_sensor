/*
 * RGB_sensor.h
 *
 *  Created on: Nov 12, 2021
 *      Author: jerem
 */

#ifndef INC_RGB_SENSOR_H_
#define INC_RGB_SENSOR_H_

#include "stdio.h"
#include "gpio.h"
#include "tim.h"

// Example at the end of this file


/*************** STRUCTURE ***************/

typedef struct RGB_struct{
	// Input Capture (Timer)

	// Input Param Pins (GPIO Out)
	GPIO_TypeDef* OutputEnable_GPIOx;
	uint16_t OutputEnable_GPIO_Pin;
	GPIO_TypeDef* LED_GPIOx;
	uint16_t LED_GPIO_Pin;

	// Input OutFreq Pins (GPIO Out)
	GPIO_TypeDef* OutFreq1_GPIOx;
	uint16_t OutFreq1_GPIO_Pin;
	GPIO_TypeDef* OutFreq2_GPIOx;
	uint16_t OutFreq2_GPIO_Pin;

	// Input ColorFilter Pins (GPIO Out)
	GPIO_TypeDef* ColorFilter1_GPIOx;
	uint16_t ColorFilter1_GPIO_Pin;
	GPIO_TypeDef* ColorFilter2_GPIOx;
	uint16_t ColorFilter2_GPIO_Pin;

}RGB_struct;


/***************   DEFINE   ***************/

#define RGB_ERROR_NONE 0
#define RGB_ERROR_FILTER 1
#define RGB_ERROR_OF_SCALING 2

#define RGB_RED 0
#define RGB_BLUE 1
#define RGB_GREEN 2
#define RGB_CLEAR 3

extern float frequency;


/*************** PROTOTYPES ***************/

uint8_t RGB_Init_SetParamGPIOs(RGB_struct* RGB_Sensor, GPIO_TypeDef* OutputEnable_GPIOx, uint16_t OutputEnable_GPIO_Pin,
		GPIO_TypeDef* LED_GPIOx, uint16_t LED_GPIO_Pin);

uint8_t RGB_Init_SetOutFreqGPIOs(RGB_struct* RGB_Sensor, GPIO_TypeDef* OutFreq1_GPIOx, uint16_t OutFreq1_GPIO_Pin,
		GPIO_TypeDef* OutFreq2_GPIOx, uint16_t OutFreq2_GPIO_Pin);

uint8_t RGB_Init_SetColorFilterGPIOs(RGB_struct* RGB_Sensor, GPIO_TypeDef* ColorFilter1_GPIOx, uint16_t ColorFilter1_GPIO_Pin,
		GPIO_TypeDef* ColorFilter2_GPIOx, uint16_t ColorFilter2_GPIO_Pin);

uint8_t RGB_Init(RGB_struct* RGB_Sensor);

uint8_t RGB_SetFilter(RGB_struct* RGB_Sensor, uint8_t color);
uint8_t RGB_SetOFscaling(RGB_struct* RGB_Sensor, uint8_t scale);

uint16_t RGB_GetFreq();


/***************  EXAMPLE  ***************/

//#include "CONFIG.h"
//
//int main()
//{
//	RGB_Init_SetParamGPIOs(&RgbSensor, RGB_OE_GPIO_Port, RGB_OE_Pin, RGB_LED_GPIO_Port, RGB_LED_Pin);
//	RGB_Init_SetOutFreqGPIOs(&RgbSensor, RGB_S0_GPIO_Port, RGB_S0_Pin, RGB_S1_GPIO_Port, RGB_S1_Pin);
//	RGB_Init_SetColorFilterGPIOs(&RgbSensor, RGB_S2_GPIO_Port, RGB_S2_Pin, RGB_S3_GPIO_Port, RGB_S3_Pin);
//
//	RGB_Init(&RgbSensor);
//
//	while(1){
//
//	}
//	return 0;
//}

#endif /* INC_RGB_SENSOR_H_ */
