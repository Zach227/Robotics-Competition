
#include <Arduino.h>
#include <gyro.h>
// Basic demo for accelerometer readings from Adafruit MPU6050
#include <MPU6050_light.h>
#include <Wire.h>

#define GATE 100
#define alphaTurn 0.8
#define UpperAngularSpeed 25

// #define alphaNot 0.1
#define LowerAngularSpeed 5

MPU6050 mpu(Wire);  // class constructor for mpu

float getAngle(int reset){
  static int pastAngle = mpu.getAngleZ();
  if(reset == 2){
    pastAngle = mpu.getAngleZ();
    Serial.println("Reset");
  }
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
  if (raw > -4) {
      angularSpeedZ = raw;
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
  getAngle(2);
    // IMU must be stationary and level during calibration
}

void loopGyro() {
  mpu.update();  // get new measurements from IMU
}
