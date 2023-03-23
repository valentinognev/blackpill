/*
 * imu.h
 *
 *  Created on: Mar 22, 2023
 *      Author: valentin
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_


#include "mpu6050.h"
#include "main.h"

typedef struct ImuData {
	float AccXData;
	float AccYData;
	float AccZData;
	float Temp;
	float GyroXData;
	float GyroYData;
	float GyroZData;
	float MagXData;
	float MagYData;
	float MagZData;
}IMU_tstImuData;

void Init__vMPU_9255(uint8_t u8asax[3]);
IMU_tstImuData GetData__stMPU_9255();

void Init__vMPU_6050();
IMU_tstImuData GetData__stMPU_6050();

void _delay_ms(int time);


#endif /* APPDRV_IMU_INC_IMU_H_ */

