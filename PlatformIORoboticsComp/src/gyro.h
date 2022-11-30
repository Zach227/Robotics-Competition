#ifndef GYRO
#define GRYO

#include <Arduino.h>


void loopGyro();

void setupGyro();

int turnDetect();

float getAngle(bool reset);


#endif