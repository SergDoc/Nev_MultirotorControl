/*
 * Kalman.c
 *
 *  Created on: 10.01.2013
 *      Author: bouwerob
 *
 *	Code derived from Philip R. Braica
 */

#include <stdint.h>
#include "board.h"
#include "kalman1D.h"

void initKalman1D(kalman1D_t *kalmanState, float q, float r, float p, float x)
{
	kalmanState->m_q[0] = q;
	kalmanState->m_q[1] = q * 1e-3f;
	kalmanState->m_r = r;
	kalmanState->m_p[0] = kalmanState->m_p[3] = p;
	kalmanState->m_p[1] = kalmanState->m_p[2] = 0;
	kalmanState->m_x[0] = x;
	kalmanState->m_x[1] = 0;
}


void kalman1DUpdate32(kalman1D_t *kalmanState, int32_t *pvalue, float dt)
{
	float m = *pvalue;
	int i;
float y0=0.0f;
float y1=0.0f;
	float s=0.0f;
	float k=0.0f;
	// Predict:
	//   X = F*X + H*U
	//   P = F*X*F^T + Q.
	// Update:
	//   Y = M – H*X          Called the innovation = measurement – state transformed by H.
	//   S = H*P*H^T + R      S = Residual covariance = covariane transformed by H + R
	//   K = P * H^T *S^-1    K = Kalman gain = variance / residual covariance.
	//   X = X + K*Y          Update with gain the new measurement
	//   P = (I – K * H) * P  Update covariance to this time.

	// X = F*X + H*U
	float oldX = kalmanState->m_x[0];
	kalmanState->m_x[0] = kalmanState->m_x[0] + (dt * kalmanState->m_x[1]);

	// P = F*X*F^T + Q
	kalmanState->m_p[0] = kalmanState->m_p[0] + dt * (kalmanState->m_p[2] + kalmanState->m_p[1]) + dt * dt * kalmanState->m_p[3] + kalmanState->m_q[0];
	kalmanState->m_p[1] = kalmanState->m_p[1] + dt * kalmanState->m_p[3] + kalmanState->m_q[1];
	kalmanState->m_p[2] = kalmanState->m_p[2] + dt * kalmanState->m_p[3] + kalmanState->m_q[2];
	kalmanState->m_p[3] = kalmanState->m_p[3] + kalmanState->m_q[3];

	// Y = M – H*X
	 y0 = m - kalmanState->m_x[0];
	 y1 = ((m - oldX) / dt) - kalmanState->m_x[1];

	// S = H*P*H^T + R
	// Because H = [1, 0] this is easy, and s is a single value not a matrix to invert.
	 s = kalmanState->m_p[0] + kalmanState->m_r;

	// K = P * H^T *S^-1
	k = kalmanState->m_p[0] / s;

	// X = X + K*Y
	kalmanState->m_x[0] += y0 * k;
	kalmanState->m_x[1] += y1 * k;

	// P = (I – K * H) * P
	for (i = 0; i < 4; i++)
	{
		kalmanState->m_p[i] = kalmanState->m_p[i] - k * kalmanState->m_p[i];
	}

	// latest estimate.
	*pvalue = (int32_t) kalmanState->m_x[0];
}

void kalman1DUpdate(kalman1D_t *kalmanState, int16_t *pvalue, float dt)
{
	int32_t m = *pvalue;
	kalman1DUpdate32(kalmanState, &m, dt);
	*pvalue = (int16_t) m;
}

