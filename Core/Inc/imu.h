/*
 * imu.h
 *
 *  Created on: Mar 22, 2023
 *      Author: valentin
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_


#include "mpu6500.h"
#include "main.h"

typedef struct ImuData {
	int16_t AccXData;
	int16_t AccYData;
	int16_t AccZData;
	float Temp;
	int16_t GyroXData;
	int16_t GyroYData;
	int16_t GyroZData;
	int16_t MagXData;
	int16_t MagYData;
	int16_t MagZData;

	float Ax;
	float Ay;
	float Az;
	float Gx;
	float Gy;
	float Gz;

	float KalmanAngleX;
	float KalmanAngleY;
	float KalmanAngleZ;
} IMU_tstImuData;

// Kalman structure
typedef struct
{
	double Q_angle;
	double Q_bias;
	double R_measure;
	double angle;
	double bias;
	double P[2][2];
} Kalman_t;

void Init__vMPU_9255(uint8_t u8asax[3]);
IMU_tstImuData GetData__stMPU_9255();

void Init__vMPU_6500();
IMU_tstImuData GetData__stMPU_6500();

void _delay_ms(int time);


#endif /* APPDRV_IMU_INC_IMU_H_ */

