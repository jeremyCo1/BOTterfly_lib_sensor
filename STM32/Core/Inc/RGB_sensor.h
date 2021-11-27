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
#include "string.h"

#define RGB_ERROR_NONE 0
#define RGB_ERROR_FILTER 1
#define RGB_ERROR_OF_SCALING 2

#define RGB_RED 0
#define RGB_BLUE 1
#define RGB_GREEN 2
#define RGB_CLEAR 3

extern float frequency;

uint8_t RGB_init();
uint16_t RGB_getFreq();

uint8_t RGB_setFilter(uint8_t color);
uint8_t RGB_setOFscaling(uint8_t scale);

#endif /* INC_RGB_SENSOR_H_ */
