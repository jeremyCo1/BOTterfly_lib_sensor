/*
 * CONFIG.h
 *
 *  Created on: Nov 27, 2021
 *      Author: jerem
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "tof_sensor.h"
#include "RGB_sensor.h"

/* TOF SENSORS */
#define TOF_nbOfSensor 3

struct VL53L0X_Dev_t TofSensor[TOF_nbOfSensor];

/* RGB SENSORS */
struct RGB_struct RgbSensor;

#endif /* INC_CONFIG_H_ */
