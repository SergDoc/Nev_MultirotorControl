/*
 * rotationMatrix.h
 *
 *  Created on: 07.02.2013
 *      Author: rob
 */

#ifndef ROTATIONMATRIX_H_
#define ROTATIONMATRIX_H_

void accIntegratorStep(float accel_ned[3], float dt);
void getPosition(int *x, int *y, int *z);
float getZPosition();
void resetIntegrator();
float getNedZ();
float getZVelocity();


#endif /* ROTATIONMATRIX_H_ */
