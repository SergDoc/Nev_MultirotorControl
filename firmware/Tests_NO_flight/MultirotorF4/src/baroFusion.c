/*
 * baroFusion.c
 *
 *  Created on: 21.02.2013
 *      Author: rob
 */

#include "board.h"
#include "baroFusion.h"

#define	F_CUT_PRESSURE     2.0f

float fc_pressure = 0.5f / (M_PI * F_CUT_PRESSURE);

// derived from the angle kalman code
float Q_angle = 0.006;	// 0.001
float Q_gyro = 0.001;	// 0.003
float R_angle = 0.01;	// 0.03
float x_angle = 0;
float x_bias = 0;
float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
float dt, y, S;
float K_0, K_1;


float kalmanBaroCalculate(float pressure, float accRate, float dt)
{
	static float lastVal = 0.0;
//	lastVal = lastVal + (dt / (fc_pressure + dt)) * (pressure - lastVal);

	accRate *= 0.00025;
	x_angle += dt * (accRate - x_bias);
	P_00 += -dt * (P_10 + P_01) + Q_angle * dt;
	P_01 += -dt * P_11;
	P_10 += -dt * P_11;
	P_11 += Q_gyro * dt;

	y = pressure - x_angle;
	S = P_00 + R_angle;
	K_0 = P_00 / S;
	K_1 = P_10 / S;

	x_angle += K_0 * y;
	x_bias += K_1 * y;
	P_00 -= K_0 * P_00;
	P_01 -= K_0 * P_01;
	P_10 -= K_1 * P_00;
	P_11 -= K_1 * P_01;

	return x_angle;
}
