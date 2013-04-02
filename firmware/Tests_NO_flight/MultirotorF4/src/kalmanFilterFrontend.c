/*
 * kalman.c
 *
 *  Created on: 25.11.2012
 *      Author: robert
 *
 *      acts more as a filter front end for testing :)
 */

#include <stdint.h>
#include "board.h"
#include "kalmanFilterFrontend.h"
#include "kalman1D.h"
#include "barofilter.h"

// accelerometer
kalman1D_t kax;
kalman1D_t kay;
kalman1D_t kaz;

// gyro
kalman1D_t kgx;
kalman1D_t kgy;
kalman1D_t kgz;

/*
 http://hicode.wordpress.com/2011/10/21/1-d-kalman-filter-for-smoothing-gps-accelerometer-signals/
 //http://en.wikipedia.org/wiki/Kalman_filter
 //run kalman filtering
 //x_k = Ax_{k-1} + Bu_k + w_k
 //z_k = Hx_k+v_k
 //time update
 //x_k = Ax_{k-1} + Uu_k
 //P_k = AP_{k-1}A^T + Q
 //measurement update
 //K_k = P_k H^T(HP_kH^T + R)^T
 //x_k = x_k + K_k(z_k - Hx_k)
 //P_k = (I - K_kH)P_k

 a = 1.0;
 h = 1.0;
 p = 0.1;
 q = 0.001;
 r = 10;
 */

static void initKalmanGyro(int16_t gyros[3])
{
	// real bad on my small jakub frame
//#define Q 0.0625 // process noise covariance
//#define	R 4.0	// measurement noise covariance
//#define P 0.47	// estimation error covariance

// small jakub frame
//#define Q 1.0 	// process noise covariance
//#define	R 0.01	// measurement noise covariance
//#define P 0.22	// estimation error covariance

// working the larger jakub frame
#define Q 10.0 	// process noise covariance
#define	R 0.1	// measurement noise covariance
#define P 0.22	// estimation error covariance	<-- rise to 0.6 is to twitchy - or lower to 0.22 for much more fun
	initKalman1D(&kgx, Q, R, P, gyros[0]);
	initKalman1D(&kgy, Q, R, P, gyros[1]);
	initKalman1D(&kgz, Q, R, P, gyros[2]);

#undef Q
#undef R
#undef P
}

static void initKalmanAccel(int16_t acc[3])
{
	// small jakub frame
//#define Q 0.0625		// process noise covariance
//#define	R 1.0		// measurement noise covariance
//#define P 0.22		// estimation error covariance

//#define Q 0.0625		// process noise covariance
//#define	R 4.0			// measurement noise covariance
//#define P 0.47			// estimation error covariance

#define Q 0.1			// process noise covariance
#define	R 10.0			// measurement noise covariance
#define P 0.22			// estimation error covariance
	initKalman1D(&kax, Q, R, P, acc[0]);
	initKalman1D(&kay, Q, R, P, acc[1]);
	initKalman1D(&kaz, Q, R, P, acc[2]);

#undef Q
#undef R
#undef P
}

void accelKalmanfilterStep(int16_t acc[3])
{
	static int _init = 0;
	static int32_t _lastTime = 0;
	uint32_t currentTime = micros();
	float dT = (currentTime - _lastTime) * 1e-6;
	_lastTime = currentTime;

	if (!_init)
	{
		_init = 1;
		initKalmanAccel(acc);
	}
	else
	{
		kalman1DUpdate(&kax, &acc[0], dT);
		kalman1DUpdate(&kay, &acc[1], dT);
		kalman1DUpdate(&kaz, &acc[2], dT);
	}
}

void gyroKalmanfilterStep(int16_t gyros[3])
{
	static int _init = 0;
	static uint32_t _lastTime = 0;
	uint32_t currentTime = micros();
	float dT = (currentTime - _lastTime) * 1e-6;
	_lastTime = currentTime;

	if (!_init)
	{
		_init = 1;
		initKalmanGyro(gyros);
	}
	else
	{
		kalman1DUpdate(&kgx, &gyros[0], dT);
		kalman1DUpdate(&kgy, &gyros[1], dT);
		kalman1DUpdate(&kgz, &gyros[2], dT);
	}
}
