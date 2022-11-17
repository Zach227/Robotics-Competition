
#include <Arduino.h>
#include <gyro.h>
// Basic demo for accelerometer readings from Adafruit MPU6050
#include <MPU6050_light.h>
#include <Wire.h>

MPU6050 mpu(Wire);  // class constructor for mpu

void printGyros(){
  float gyroX = mpu.getGyroX();
  float gyroY = mpu.getGyroY();
  float gyroZ = mpu.getGyroZ();
  Serial.print(gyroX);
  Serial.print(",");
  Serial.print(gyroY);
  Serial.print(",");
  Serial.print(gyroZ); 
  Serial.println("");  
  delay(500);
}
void displayAngles(){
  float a = mpu.getAngleZ();
  Serial.println(a);

}

int turnStartDetect(bool reset){
  float angleZ = mpu.getAngleZ();
  static float previousAngleZ = mpu.getAngleZ();
  if(reset)
    previousAngleZ = mpu.getAngleZ();
  if(angleZ - previousAngleZ >= 7){
    return 1;
  }
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
  //printGyros();  // write gyro values to serial port
  //displayAngles();
  int a = turnStartDetect(false);
  if(a){
    Serial.println("TURN");
    turnStartDetect(true);
  }
  //printAccels();  // write accels to serial port
}



