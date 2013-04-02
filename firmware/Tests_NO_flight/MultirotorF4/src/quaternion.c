/*
 * quaternion.c
 *
 *  Created on: 26.01.2013
 *      Author: rob
 */

// derived from:    Copyright © 2011  Bill Nesbitt
#include "math.h"
#include "board.h"
#include "quaternion.h"

void normalizeVector3(float *v)
{
	float mag;

	mag = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

	v[0] /= mag;
	v[1] /= mag;
	v[2] /= mag;
}

void normalizeVector4(float *v)
{
	float mag;

	mag = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3]);

	v[0] /= mag;
	v[1] /= mag;
	v[2] /= mag;
	v[3] /= mag;
}

void RPY2Quaternion(const float rpy[3], float q[4])
{
	float phi, theta, psi;
	float cphi, sphi, ctheta, stheta, cpsi, spsi;

	phi = DEG2RAD * rpy[0] / 2;
	theta = DEG2RAD * rpy[1] / 2;
	psi = DEG2RAD * rpy[2] / 2;
	cphi = cosf(phi);
	sphi = sinf(phi);
	ctheta = cosf(theta);
	stheta = sinf(theta);
	cpsi = cosf(psi);
	spsi = sinf(psi);

	q[0] = cphi * ctheta * cpsi + sphi * stheta * spsi;
	q[1] = sphi * ctheta * cpsi - cphi * stheta * spsi;
	q[2] = cphi * stheta * cpsi + sphi * ctheta * spsi;
	q[3] = cphi * ctheta * spsi - sphi * stheta * cpsi;

	if (q[0] < 0)
	{		// q0 always positive for uniqueness
		q[0] = -q[0];
		q[1] = -q[1];
		q[2] = -q[2];
		q[3] = -q[3];
	}
}

void rotateVecByMatrix(float *vr, float *v, float m[3][3])
{
	vr[0] = m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2];
	vr[1] = m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2];
	vr[2] = m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2];
}

void rotateVecByRevMatrix(float *vr, float *v, float m[3][3])
{
	vr[0] = m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2];
	vr[1] = m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2];
	vr[2] = m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2];
}

// no need to normalize as our quat stays very close to norm
void quatToMatrix(float m[3][3], float *q)
{
	float tmp1, tmp2;
	float sqw = q[0] * q[0];
	float sqx = q[1] * q[1];
	float sqy = q[2] * q[2];
	float sqz = q[3] * q[3];

	// get the invert square length
	//	float invs = 1.0f / (sqx + sqy + sqz + sqw);

	// rotation matrix is scaled by inverse square length
	m[0][0] = (sqx - sqy - sqz + sqw); // * invs;
	m[1][1] = (-sqx + sqy - sqz + sqw); // * invs;
	m[2][2] = (-sqx - sqy + sqz + sqw); // * invs;

	tmp1 = q[1] * q[2];
	tmp2 = q[3] * q[0];
	m[1][0] = 2.0f * (tmp1 + tmp2); // * invs;
	m[0][1] = 2.0f * (tmp1 - tmp2); // * invs;

	tmp1 = q[1] * q[3];
	tmp2 = q[2] * q[0];
	m[2][0] = 2.0f * (tmp1 - tmp2); // * invs;
	m[0][2] = 2.0f * (tmp1 + tmp2); // * invs;

	tmp1 = q[2] * q[3];
	tmp2 = q[1] * q[0];
	m[2][1] = 2.0f * (tmp1 + tmp2); // * invs;
	m[1][2] = 2.0f * (tmp1 - tmp2); // * invs;
}

void quatExtractEuler(float *q, float *yaw, float *pitch, float *roll)
{
	float q0, q1, q2, q3;

	q0 = q[1];
	q1 = q[2];
	q2 = q[3];
	q3 = q[0];

	*yaw = atan2f((2.0f * (q0 * q1 + q3 * q2)), (q3 * q3 - q2 * q2 - q1 * q1 + q0 * q0));
	*pitch = asinf(-2.0f * (q0 * q2 - q1 * q3));
	*roll = atanf((2.0f * (q1 * q2 + q0 * q3)) / (q3 * q3 + q2 * q2 - q1 * q1 - q0 * q0));
}

// result and source can be the same
void rotateQuat(float *qr, float *q, float *rate, float dT)
{
	float q1[4];
	float s, t, lg;
	float qMag;

	s = sqrtf(rate[0] * rate[0] + rate[1] * rate[1] + rate[2] * rate[2]) * 0.5f;
	t = -(0.5f * sinf(s) / s);
	rate[0] *= t;
	rate[1] *= t;
	rate[2] *= t;

	// create Lagrange factor to control quat's numerical integration errors
	qMag = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
	lg = cosf(s) + (1.0f - qMag * qMag) * dT * dT;

	// rotate
	q1[0] = q[0];
	q1[1] = q[1];
	q1[2] = q[2];
	q1[3] = q[3];

	qr[0] = lg * q1[0] + rate[0] * q1[1] + rate[1] * q1[2] + rate[2] * q1[3];
	qr[1] = -rate[0] * q1[0] + lg * q1[1] - rate[2] * q1[2] + rate[1] * q1[3];
	qr[2] = -rate[1] * q1[0] + rate[2] * q1[1] + lg * q1[2] - rate[0] * q1[3];
	qr[3] = -rate[2] * q1[0] - rate[1] * q1[1] + rate[0] * q1[2] + lg * q1[3];
}

void quatMatrixMultiply(float mr[3][3], float ma[3][3], float mb[3][3])
{
	mr[0][0] = ma[0][0] * mb[0][0] + ma[0][1] * mb[1][0] + ma[0][2] * mb[2][0];
	mr[0][1] = ma[0][0] * mb[0][1] + ma[0][1] * mb[1][1] + ma[0][2] * mb[2][1];
	mr[0][2] = ma[0][0] * mb[0][2] + ma[0][1] * mb[1][2] + ma[0][2] * mb[2][2];

	mr[1][0] = ma[1][0] * mb[0][0] + ma[1][1] * mb[1][0] + ma[1][2] * mb[2][0];
	mr[1][1] = ma[1][0] * mb[0][1] + ma[1][1] * mb[1][1] + ma[1][2] * mb[2][1];
	mr[1][2] = ma[1][0] * mb[0][2] + ma[1][1] * mb[1][2] + ma[1][2] * mb[2][2];

	mr[2][0] = ma[2][0] * mb[0][0] + ma[2][1] * mb[1][0] + ma[2][2] * mb[2][0];
	mr[2][1] = ma[2][0] * mb[0][1] + ma[2][1] * mb[1][1] + ma[2][2] * mb[2][1];
	mr[2][2] = ma[2][0] * mb[0][2] + ma[2][1] * mb[1][2] + ma[2][2] * mb[2][2];
}

void quatMatrixTranspose(float mt[3][3], float m[3][3])
{
	mt[0][0] = m[0][0];
	mt[0][1] = m[1][0];
	mt[0][2] = m[2][0];

	mt[1][0] = m[0][1];
	mt[1][1] = m[1][1];
	mt[1][2] = m[2][1];

	mt[2][0] = m[0][2];
	mt[2][1] = m[1][2];
	mt[2][2] = m[2][2];
}

