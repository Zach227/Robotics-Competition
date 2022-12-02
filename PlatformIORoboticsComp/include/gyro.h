#ifndef GYRO
#define GRYO

#include <Arduino.h>


void loopGyro();

void setupGyro();

int turnDetect();

int notTurning();

float getAngle(bool reset);


#endif