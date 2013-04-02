/*
 * kalman1D.h
 *
 *  Created on: 11.01.2013
 *      Author: rob
 */

#ifndef KALMAN1D_H_
#define KALMAN1D_H_


typedef struct
{
	float d;
	float fc;
	float m_x[2];
	float m_p[4];
	float m_q[4];
	float m_r;
} kalman1D_t;

void initKalman1D(kalman1D_t *kalmanState, float q, float r, float p, float x);
void kalman1DUpdate32(kalman1D_t *kalmanState, int32_t *pvalue, float dt);
void kalman1DUpdate(kalman1D_t *kalmanState, int16_t *pvalue, float dt);


#endif /* KALMAN1D_H_ */
