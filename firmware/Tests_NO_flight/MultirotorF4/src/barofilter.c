/*
 * barofilter.c
 *
 *  Created on: 27.01.2013
 *      Author: rob
 */

#include <stdint.h>
#include "board.h"



#define	F_CUT_BARO     2.0f

typedef struct
{
	float q; //process noise covariance
	float r; //measurement noise covariance
	float x; //value
	float p; //estimation error covariance
} kalman_state;

// baro
kalman_state bar;

void kalmanBaroInit(float q, float r, float p, float intial_value)
{
	bar.q = q;
	bar.r = r;
	bar.p = p;
	bar.x = intial_value;
}

void kalmanBaroUpdate(float measurement)
{
	//prediction update
	//omit x = x
	float k; //kalman gain

	bar.p = bar.p + bar.q;

	//measurement update
	k = bar.p / (bar.p + bar.r);
	bar.x = bar.x + k * (measurement - bar.x);
	bar.p = (1 - k) * bar.p;
}

/**
 * temporary solution
 */
void baroKalmanfilterStep(int32_t *pressure)
{
	static float dTerm = 0.0;
	static float fc;
 float tmp =0.0f;
	static int _init = 0;
	static int32_t _lastTime = 0;
	uint32_t currentTime = micros();
	float dt = (currentTime - _lastTime) * 1e-6;
	_lastTime = currentTime;

	if (!_init)
	{
		_init = 1;
#define Q 1.625			// process noise covariance
#define	R 60.0			// measurement noise covariance
#define P 1.4			// estimation error covariance

		 tmp = *pressure;
		fc = 0.5f / (M_PI * F_CUT_BARO);
		kalmanBaroInit(Q, R, P, tmp);

#undef Q
#undef R
#undef P
	}
	else
	{

		// a pt1 element to stop heavy vibrations and propeller wash
	    // apply PT1 calculation
		float m = *pressure;
		m = dTerm + (dt / (fc + dt)) * (m - dTerm);
		dTerm = m;
		kalmanBaroUpdate(m);
		*pressure = (int32_t)bar.x;
	}
}



/*
 NAME - the instable baro filter
 kapogee.c - A third order Kalman filter for
 RDAS raw data files.
 SYNOPSIS
 kapogee <infile
 DESCRIPTION:
 Performs Kalman filtering on standard input
 data using a third order, or constant acceleration,
 propagation model.
 The standard input data is of the form:
 Column 1: Time of the measurement (seconds)
 Column 2: Acceleration Measurement (ignored)
 Column 3: Pressure Measurement (ADC counts)
 All arithmetic is performed using 32 bit floats.
 The standard output data is of the form:
 Liftoff detected at time: <time>
 Apogee detected at time: <time>
 AUTHOR
 David Schultz
 */

#define MEASUREMENTSIGMA 0.8
#define MODELSIGMA 0.02
#define MEASUREMENTVARIANCE MEASUREMENTSIGMA	// *MEASUREMENTSIGMA
#define MODELVARIANCE MODELSIGMA				// *MODELSIGMA

// the variables:
int liftoff = 0;

float est[3] =
	{ 0, 0, 0 };
float estp[3] =
	{ 0, 0, 0 };
float pest[3][3] =
	{ 0.002, 0, 0,
	  0, 0.004, 0,
	  0, 0, 0.002 };
float pestp[3][3] =
	{ 0, 0, 0,
	  0, 0, 0,
	  0, 0, 0 };
float phi[3][3] =
	{ 1, 0, 0,
	  0, 1, 0,
	 0, 0, 1.0 };
float phit[3][3] =
	{ 1, 0, 0,
	  0, 1, 0,
	  0, 0, 1.0 };
float gain[3] =
	{ 0.010317, 0.010666, 0.004522 };
float term[3][3];

/**
 * the filter is unstable - need to adjust the variance to meet with the bmp085
 */
float baroFilter(float pressure)
{
	static int _init = 0;
	static int32_t _lastTime = 0;
	uint32_t currentTime = micros();
	float dt = (currentTime - _lastTime) * 1e-6;
	_lastTime = currentTime;

	 _init = 0;
	if (!_init)
	{
		_init = 1;
		est[0] = pressure;
//#ifdef DEBUG
//		printf("%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
//		time, pressure, est[0], est[1], est[2], sqrt(pest[0][0]),
//		sqrt(pest[1][1]), sqrt(pest[2][2]), gain[0], gain[1], gain[2]);
//#endif
//#ifdef DEBUG
//		printf("%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
//		time, pressure, est[0], est[1], est[2], sqrt(pest[0][0]),
//		sqrt(pest[1][1]), sqrt(pest[2][2]), gain[0], gain[1], gain[2]);
//#endif

		return pressure;
	}

	/*
	 Fill in state transition matrix and its transpose
	 */
	dt *= 4;
	phi[0][1] = dt;
	phi[1][2] = dt;
	phi[0][2] = dt * dt * 0.5;
	phit[1][0] = dt;
	phit[2][1] = dt;
	phit[2][0] = dt * dt * 0.5;

	/* Propagate state */
	estp[0] = phi[0][0] * est[0] + phi[0][1] * est[1] + phi[0][2] * est[2];
	estp[1] = phi[1][0] * est[0] + phi[1][1] * est[1] + phi[1][2] * est[2];
	estp[2] = phi[2][0] * est[0] + phi[2][1] * est[1] + phi[2][2] * est[2];
	/* Simplified version (phi is constant)
	 estp[0] = est[0] + est[1]*dt + est[2]*dt*dt/2.0;
	 estp[1] = est[1] + est[2]*dt;
	 estp[2] = est[2]; */
	/* Propagate state covariance */
	term[0][0] = phi[0][0] * pest[0][0] + phi[0][1] * pest[1][0] + phi[0][2] * pest[2][0];
	term[0][1] = phi[0][0] * pest[0][1] + phi[0][1] * pest[1][1] + phi[0][2] * pest[2][1];
	term[0][2] = phi[0][0] * pest[0][2] + phi[0][1] * pest[1][2] + phi[0][2] * pest[2][2];
	term[1][0] = phi[1][0] * pest[0][0] + phi[1][1] * pest[1][0] + phi[1][2] * pest[2][0];
	term[1][1] = phi[1][0] * pest[0][1] + phi[1][1] * pest[1][1] + phi[1][2] * pest[2][1];
	term[1][2] = phi[1][0] * pest[0][2] + phi[1][1] * pest[1][2] + phi[1][2] * pest[2][2];
	term[2][0] = phi[2][0] * pest[0][0] + phi[2][1] * pest[1][0] + phi[2][2] * pest[2][0];
	term[2][1] = phi[2][0] * pest[0][1] + phi[2][1] * pest[1][1] + phi[2][2] * pest[2][1];
	term[2][2] = phi[2][0] * pest[0][2] + phi[2][1] * pest[1][2] + phi[2][2] * pest[2][2];
	pestp[0][0] = term[0][0] * phit[0][0] + term[0][1] * phit[1][0] + term[0][2] * phit[2][0];
	pestp[0][1] = term[0][0] * phit[0][1] + term[0][1] * phit[1][1] + term[0][2] * phit[2][1];
	pestp[0][2] = term[0][0] * phit[0][2] + term[0][1] * phit[1][2] + term[0][2] * phit[2][2];
	pestp[1][0] = term[1][0] * phit[0][0] + term[1][1] * phit[1][0] + term[1][2] * phit[2][0];
	pestp[1][1] = term[1][0] * phit[0][1] + term[1][1] * phit[1][1] + term[1][2] * phit[2][1];
	pestp[1][2] = term[1][0] * phit[0][2] + term[1][1] * phit[1][2] + term[1][2] * phit[2][2];
	pestp[2][0] = term[2][0] * phit[0][0] + term[2][1] * phit[1][0] + term[2][2] * phit[2][0];
	pestp[2][1] = term[2][0] * phit[0][1] + term[2][1] * phit[1][1] + term[2][2] * phit[2][1];
	pestp[2][2] = term[2][0] * phit[0][2] + term[2][1] * phit[1][2] + term[2][2] * phit[2][2];
	pestp[0][0] = pestp[0][0] + MODELVARIANCE;
	/*
	 Calculate Kalman Gain
	 */
#ifndef TEST
	gain[0] = (phi[0][0] * pestp[0][0] + phi[0][1] * pestp[1][0] + phi[0][2] * pestp[2][0]) / (pestp[0][0] + MEASUREMENTVARIANCE);
	gain[1] = (phi[1][0] * pestp[0][0] + phi[1][1] * pestp[1][0] + phi[1][2] * pestp[2][0]) / (pestp[0][0] + MEASUREMENTVARIANCE);
	gain[2] = (phi[2][0] * pestp[0][0] + phi[2][1] * pestp[1][0] + phi[2][2] * pestp[2][0]) / (pestp[0][0] + MEASUREMENTVARIANCE);
#endif
	/*
	 Update state and state covariance
	 */
	est[0] = estp[0] + gain[0] * (pressure - estp[0]);
	est[1] = estp[1] + gain[1] * (pressure - estp[0]);
	est[2] = estp[2] + gain[2] * (pressure - estp[0]);
	pest[0][0] = pestp[0][0] * (1.0 - gain[0]);
	pest[0][1] = pestp[1][0] * (1.0 - gain[0]);
	pest[0][2] = pestp[2][0] * (1.0 - gain[0]);
	pest[1][0] = pestp[0][1] - gain[1] * pestp[0][0];
	pest[1][1] = pestp[1][1] - gain[1] * pestp[1][0];
	pest[1][2] = pestp[2][1] - gain[1] * pestp[2][0];
	pest[2][0] = pestp[0][2] - gain[2] * pestp[0][0];
	pest[2][1] = pestp[1][2] - gain[2] * pestp[1][0];
	pest[2][2] = pestp[2][2] - gain[2] * pestp[2][0];
	/*
	 Output
	 */
	if (liftoff == 0)
	{
		if (est[1] < -5.0)
		{
			liftoff = 1;
			//uartPrint("Liftoff detected\r\n");
		}
	}
	else
	{
		if (est[1] > 0)
		{
			//uartPrint("Apogee detected\r\n");
		}
	}


	return est[0];
}

