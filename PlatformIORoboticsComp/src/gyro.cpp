
#include <Arduino.h>
#include <gyro.h>
// Basic demo for accelerometer readings from Adafruit MPU6050
#include <MPU6050_light.h>
#include <Wire.h>

#define alphaTurn 0.75
#define UpperAngularSpeed 25

#define alphaNot 0.1
#define LowerAngularSpeed 10

MPU6050 mpu(Wire);  // class constructor for mpu

int turnDetect(){
  mpu.update();
  static float angularSpeedZ = 0;
  float raw = mpu.getGyroZ();
  if (raw > -4) {
      angularSpeedZ = (angularSpeedZ*alphaTurn) + raw*(1-alphaTurn);
  }
  Serial.print(">raw: ");
  Serial.println(raw);
  Serial.print(">Angular Speed: ");
  Serial.println(angularSpeedZ);
  if(angularSpeedZ >= UpperAngularSpeed){
    return 1;
  }
  else
    return 0;
}

int notTurning(){
  mpu.update();
  static float angularSpeedZ = 0;
  float raw = mpu.getGyroZ();
  if (raw > -4) {
      angularSpeedZ = (angularSpeedZ*alphaNot) + raw*(1-alphaNot);
  }
  Serial.print(">N raw: ");
  Serial.println(raw);
  Serial.print(">N Angular Speed: ");
  Serial.println(angularSpeedZ);
  if(angularSpeedZ <= LowerAngularSpeed){
    return 1;
  }
  else
    return 0;
}

float getAngle(bool reset){
  if(reset)
    mpu.calcOffsets(true, true);
  float angleZ = mpu.getAngleZ();
  return angleZ;
}


void setupGyro(void) {
  Serial.begin(9600);
  Wire.begin();  // begin I2C wire class

  // initialize mpu with user selectable scale and range
  mpu.begin();
  delay(1000);
  mpu.calcOffsets(true,true);  // auto calibrate gyro and accels
    // IMU must be stationary and level during calibration
}

void loopGyro() {
  mpu.update();  // get new measurements from IMU
}



