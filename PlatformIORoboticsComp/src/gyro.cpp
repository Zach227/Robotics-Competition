
#include <Arduino.h>
#include <gyro.h>
// Basic demo for accelerometer readings from Adafruit MPU6050
#include <MPU6050_light.h>
#include <Wire.h>

#define GATE 70
#define alphaTurn 0.9
#define UpperAngularSpeed 30

#define alphaNot 0.1
#define LowerAngularSpeed 10

MPU6050 mpu(Wire);  // class constructor for mpu

float getAngle(bool reset){
  static int pastAngle = mpu.getAngleZ();
  if(reset)
    pastAngle = mpu.getAngleZ();
  int angleZ = mpu.getAngleZ();
  return (angleZ - pastAngle);
}

int turnDetect(){
  mpu.update();
  static float angularSpeedZ = 0;
  float raw = mpu.getGyroZ();
  if (raw > -4 && raw - angularSpeedZ < GATE) {
      angularSpeedZ = (angularSpeedZ*alphaTurn) + raw*(1-alphaTurn);
  }
  Serial.print(">raw: ");
  Serial.println(raw);
  Serial.print(">Angular Speed: ");
  Serial.println(angularSpeedZ);
  getAngle(true);
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
  if (raw > -4 && raw - angularSpeedZ < GATE) {
      angularSpeedZ = (angularSpeedZ*alphaNot) + raw*(1-alphaNot);
  }
  Serial.print(">N raw: ");
  Serial.println(raw);
  Serial.print(">N Angular Speed: ");
  Serial.println(angularSpeedZ);
    if(angularSpeedZ <= LowerAngularSpeed)
      return 1;
    else
      return 0;
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



