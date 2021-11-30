/*
 * tof_sensor.h
 *
 *  Created on: 17 oct. 2021
 *      Author: jerem
 */

#ifndef INC_TOF_SENSOR_H_
#define INC_TOF_SENSOR_H_

#include <stdio.h>
#include <gpio.h>
#include <i2c.h>

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_device.h"


/* -------------- STRUCTURE -------------- */

// cf vl53l0x_platform.h

//typedef struct {
//    I2C_HandleTypeDef *I2cHandle;
//    uint8_t   I2cDevAddr;
//    uint8_t   comms_type;
//    uint16_t  comms_speed_khz;
//    uint16_t  rangeMillimeter;
//} VL53L0X_Dev_t;

//VL53L0X_Dev_t tof_sensor[nbOfSensors];


/* -------------- PROTOTYPE -------------- */

uint8_t Tof_Init_SetI2C(VL53L0X_Dev_t* device, I2C_HandleTypeDef *hi2c, uint8_t I2cAddr);

uint8_t Tof_Init_SetGPIOs(VL53L0X_Dev_t* device, GPIO_TypeDef* XSHUT_GPIOx, uint16_t XSHUT_GPIO_Pin,
		GPIO_TypeDef* EXTI_GPIOx, uint16_t EXTI_GPIO_Pin);

uint8_t Tof_Init_SetEXTI(VL53L0X_Dev_t* device, IRQn_Type EXTIx_IRQn);

uint8_t Tof_Init(VL53L0X_Dev_t* device);

// Initialization Flow
uint8_t Tof_InitializationFlow(VL53L0X_Dev_t* device, uint8_t interruptPin);

// Device initialization
uint8_t tof_initialization(VL53L0X_Dev_t* device);

// Device calibration
uint8_t tof_calibration(VL53L0X_Dev_t* device);

// System settings
uint8_t tof_settings(VL53L0X_Dev_t* device, uint8_t interruptPin);

uint8_t tof_setDeviceAddr(VL53L0X_Dev_t* device, uint8_t new_addr);

uint8_t tof_getDeviceInfo(VL53L0X_Dev_t* myDevice, VL53L0X_DeviceInfo_t* deviceInfo);



/* --------------  EXAMPLE  -------------- */

//#include "tof_sensor.h"
//
//int main()
//{
//	tof_init();
//	while(1){
//		printf("tof_0 : Range %d mm\r\n",tof_sensor[0].rangeMillimeter);
//		printf("tof_1 : Range %d mm\r\n",tof_sensor[1].rangeMillimeter);
//		printf("tof_2 : Range %d mm\r\n",tof_sensor[2].rangeMillimeter);
//		HAL_Delay(250);
//	}
// return 0;
//}


#endif /* INC_TOF_SENSOR_H_ */
