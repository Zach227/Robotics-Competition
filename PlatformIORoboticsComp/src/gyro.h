#ifndef GYRO
#define GYRO

#include <Arduino.h>


void loopGyro();

void setupGyro();

int turnDetect();

float getAngle(bool reset);


#endif