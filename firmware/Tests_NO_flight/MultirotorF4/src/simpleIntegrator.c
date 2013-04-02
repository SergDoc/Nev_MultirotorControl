/*
 * rotationMatrix.c
 *
 *  Created on: 07.02.2013
 *      Author: rob
 */

#include "board.h"
#include "simpleIntegrator.h"

float m[3][3];	// rotation matrix

typedef struct
{
	float accNedSum;
	int samples;
	float velN;
	float velE;
	float velD;
	float posN;
	float posE;
	float alt;
	float agl;				// above ground level from range sensor

} navStruct_t;

navStruct_t navData;

void resetIntegrator()
{
	navData.accNedSum = 0.0;
	navData.samples = 0;

	navData.velE = 0.0;
	navData.velN = 0.0;
	navData.velD = 0.0;

	navData.posE = 0.0;
	navData.posN = 0.0;
	navData.alt  = 0.0;

	navData.agl = 0.0;
}

void accIntegratorStep(float accel_ned[3], float dt)
{
	static int nullValueCounter[3] = {0,0,0};
	int i;
	float vel[3];

	// new velocity
	for (i = 0; i < 3; i++)
	{
		// from m to cm
		accel_ned[i] *= 100.0f;

		if (fabs(accel_ned[i]) > 1.0f)
		{
			vel[i] = accel_ned[i] * dt;
			nullValueCounter[i] = 0;
		}
		else
		{
			nullValueCounter[i]++;
			if (nullValueCounter[i] > 2)
			{
				vel[i] = 0;
				// reset velocity for this axe - stops the running integrator
				switch (i)
				{
				case 0:
					navData.velE = 0.0;
					break;
				case 1:
					navData.velN = 0.0;
					break;
				case 2:
					navData.velD = 0.0;
					break;
				}
			}
		}
	}

	// update velocity
	navData.velE -= vel[0];
	navData.velN -= vel[1];
	navData.velD += vel[2];

	// update position
	navData.posE += navData.velE * 0.5f * dt;
	navData.posN += navData.velN * 0.5f * dt;
	navData.alt += navData.velD * 0.5f * dt;

	navData.agl -= navData.velD * 0.5f * dt;

	navData.accNedSum += accel_ned[2];
	navData.samples++;
}

// average the samples since the last reading
float getNedZ()
{
	float average = navData.accNedSum;
	if (navData.samples > 0)
	{
		average /= (float) navData.samples;
		navData.samples = 0;
	}
	return average;
}

// centimeters
void getPosition(int *x, int *y, int *z)
{
	*x = (int) navData.posN;
	*y = (int) navData.posE;
	*z = (int) navData.alt;
}

// centimeters
float getZPosition()
{
	return navData.alt;
}

float getZVelocity()
{
	return navData.velD;
}
