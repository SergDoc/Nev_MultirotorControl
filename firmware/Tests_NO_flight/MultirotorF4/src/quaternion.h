/*
 * quaternion.h
 *
 *  Created on: 26.01.2013
 *      Author: rob
 */

#ifndef QUATERNION_H_
#define QUATERNION_H_

#define QUAT_N                  1.0f		// factor that determines the convergence speed of the quat's numerical error: (n * dt) < 1
#define QUAT_GRAVITY		9.80665f	// m/s^2
#define QUAT_BIAS_FACTOR	0.001f		//(TIMESTEP * 1.0f)


void quatToMatrix(float m[3][3], float *q);
void rotateVecByMatrix(float *vr, float *v, float m[3][3]);
void rotateVecByRevMatrix(float *vr, float *v, float m[3][3]);
void RPY2Quaternion(const float rpy[3], float q[4]);

#endif /* QUATERNION_H_ */
