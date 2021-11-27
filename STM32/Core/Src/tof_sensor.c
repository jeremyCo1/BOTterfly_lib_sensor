/*
 * tof_sensor.c
 *
 *  Created on: 17 oct. 2021
 *      Author: jerem
 */

#include "tof_sensor.h"

uint8_t tof_init(void){
	//VL53L0X_DeviceInfo_t deviceInfo[nbOfSensors];

	for(int i=0; i<nbOfSensors; i++){
		tof_sensor[i].I2cHandle = &hi2c1;
		tof_sensor[i].I2cDevAddr = 0x52;
		tof_sensor[i].comms_speed_khz = 400;
		tof_sensor[i].comms_type = 1;
	}

	// Broches XSHUT à l'état bas (actif)
	HAL_GPIO_WritePin(XSHUT0_GPIO_Port, XSHUT0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(XSHUT1_GPIO_Port, XSHUT1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(XSHUT2_GPIO_Port, XSHUT2_Pin, GPIO_PIN_RESET);
	HAL_Delay(200);

	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);

	// Initialization Flow
	HAL_GPIO_WritePin(XSHUT0_GPIO_Port, XSHUT0_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	tof_initializationFlow(&tof_sensor[0], (uint8_t)tof_0_GPIO_Port->ODR);
	tof_setDeviceAddr(&tof_sensor[0], 0x55);
	HAL_Delay(200);
	HAL_GPIO_WritePin(XSHUT1_GPIO_Port, XSHUT1_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	tof_initializationFlow(&tof_sensor[1], (uint8_t)tof_1_GPIO_Port->ODR);
	tof_setDeviceAddr(&tof_sensor[1], 0x58);
	HAL_Delay(200);
	HAL_GPIO_WritePin(XSHUT2_GPIO_Port, XSHUT2_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	tof_initializationFlow(&tof_sensor[2], (uint8_t)tof_2_GPIO_Port->ODR);
	tof_setDeviceAddr(&tof_sensor[2], 0x5b);
	HAL_Delay(200);

	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	HAL_Delay(200);

	return 0;
}

uint8_t tof_initializationFlow(VL53L0X_Dev_t* device, uint8_t interruptPin){
	// Device initialization (~ 40ms)
	tof_initialization(device);
	HAL_Delay(50);

	// Calibration data loading (~ 1ms)
	tof_calibration(device);
	HAL_Delay(5);

	// System settings (~ 1ms)
	tof_settings(device, interruptPin);
	HAL_Delay(5);

	VL53L0X_StartMeasurement(device);

	return 0;
}

uint8_t tof_initialization(VL53L0X_Dev_t* device){
	uint8_t status = VL53L0X_ERROR_NONE;

	// DataInit
	if((status = VL53L0X_DataInit(device)) != VL53L0X_ERROR_NONE){
		printf("ToF - Error DataInit : %d\r\n", status);
		return 1;
	}

	// StaticInit
	if((status = VL53L0X_StaticInit(device)) != VL53L0X_ERROR_NONE){
		printf("ToF - Error StaticInit : %d\r\n", status);
		return 1;
	}

	return 0;
}


// SPADs calibration
uint32_t *refSpadCount; uint8_t *isApertureSpads;
// Temperature calibration
uint8_t *pVhvSettings; uint8_t *pPhaseCal;

uint8_t tof_calibration(VL53L0X_Dev_t* device){
	uint8_t status = VL53L0X_ERROR_NONE;

	// SPADs calibration (~ 10ms)
	if((status = VL53L0X_PerformRefSpadManagement(device, refSpadCount, isApertureSpads)) != VL53L0X_ERROR_NONE){
		printf("ToF - Error SPADs calibration : %d\r\n", status);
		return 1;
	}

	// Temperature calibration (~ 40ms)
	if((status = VL53L0X_PerformRefCalibration(device, pVhvSettings, pPhaseCal)) != VL53L0X_ERROR_NONE){
		printf("ToF - Error Temperature calibration : %d\r\n", status);
		return 1;
	}

	return 0;
}

uint8_t tof_settings(VL53L0X_Dev_t* device, uint8_t interruptPin){
	uint8_t status = VL53L0X_ERROR_NONE;

	// Device mode
	if((status = VL53L0X_SetDeviceMode(device, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING)) != VL53L0X_ERROR_NONE){
		printf("ToF - Error SetDeviceMode : %d\r\n", status);
		return 1;
	}

	// Set GPIO : Pour interruptions
	status = VL53L0X_SetGpioConfig(device, interruptPin, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY, VL53L0X_INTERRUPTPOLARITY_HIGH);
	if(status != VL53L0X_ERROR_NONE){
		printf("ToF - Error SetGPIO : %d %d\r\n",status,interruptPin);
		return 1;
	}

	return 0;
}

uint8_t tof_setDeviceAddr(VL53L0X_Dev_t* device, uint8_t new_addr){
	uint8_t status = VL53L0X_ERROR_NONE;

	if((status = VL53L0X_SetDeviceAddress(device, new_addr)) != VL53L0X_ERROR_NONE){
		printf("ToF - Error SetDeviceAddress : %d\r\n", status);
		return 1;
	}
	device->I2cDevAddr = new_addr;

	return 0;
}

uint8_t tof_getDeviceInfo(VL53L0X_Dev_t* myDevice, VL53L0X_DeviceInfo_t* deviceInfo){
	uint8_t status = VL53L0X_ERROR_NONE;

	if((status = VL53L0X_GetDeviceInfo(myDevice, deviceInfo)) != VL53L0X_ERROR_NONE){
		printf("ToF - Error GetDeviceInfo : %d\r\n", status);
		return 1;
	}

	printf("VL53L0X_GetDeviceInfo:\r\n");
	printf("Device Name : %s\r\n", deviceInfo->Name);
	printf("Device Type : %s\r\n", deviceInfo->Type);
	printf("Device ID : %s\r\n", deviceInfo->ProductId);
	printf("ProductRevisionMajor : %d\r\n", deviceInfo->ProductRevisionMajor);
	printf("ProductRevisionMinor : %d\r\n", deviceInfo->ProductRevisionMinor);

	return 0;
}

VL53L0X_RangingMeasurementData_t VL53L0X_RangingMeasurementData;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == tof_0_Pin){
		uint32_t InterruptMask = 0;
		VL53L0X_GetRangingMeasurementData(&tof_sensor[0], &VL53L0X_RangingMeasurementData);
		tof_sensor[0].rangeMillimeter = VL53L0X_RangingMeasurementData.RangeMilliMeter;
		VL53L0X_ClearInterruptMask(&tof_sensor[0], InterruptMask);
	}else
	if(GPIO_Pin == tof_1_Pin){
		uint32_t InterruptMask = 0;
		VL53L0X_GetRangingMeasurementData(&tof_sensor[1], &VL53L0X_RangingMeasurementData);
		tof_sensor[1].rangeMillimeter = VL53L0X_RangingMeasurementData.RangeMilliMeter;
		VL53L0X_ClearInterruptMask(&tof_sensor[1], InterruptMask);
	}else
	if(GPIO_Pin == tof_2_Pin){
		uint32_t InterruptMask = 0;
		VL53L0X_GetRangingMeasurementData(&tof_sensor[2], &VL53L0X_RangingMeasurementData);
		tof_sensor[2].rangeMillimeter = VL53L0X_RangingMeasurementData.RangeMilliMeter;
		VL53L0X_ClearInterruptMask(&tof_sensor[2], InterruptMask);
	}
}


