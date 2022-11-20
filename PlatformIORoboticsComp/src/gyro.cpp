
#include <Arduino.h>
#include <gyro.h>
// Basic demo for accelerometer readings from Adafruit MPU6050
#include <MPU6050_light.h>
#include <Wire.h>

MPU6050 mpu(Wire);  // class constructor for mpu

void printGyros(){
  //float gyroX = mpu.getGyroX();
  //float gyroY = mpu.getGyroY();
  float gyroZ = mpu.getAccAngleX();
  //Serial.print(gyroX);
  Serial.print(",");
  //Serial.print(gyroY);
  Serial.print(",");
  Serial.print(gyroZ); 
  Serial.println("");  
  delay(500);
}
void displayAngles(){
  float a = mpu.getGyroZ();
  Serial.println(a);

}

int turnDetect(){
  mpu.update();
  float angularSpeedZ = mpu.getGyroZ();
  //Serial.println(angularSpeedZ);
  if(angularSpeedZ >= 7){
    return 1;
  }
  else
    return 0;
}

float getAngle(bool reset){
  static float angleZ = 0;
  static float angleOffset = 0;
  if(reset)
    angleOffset = mpu.getAngleZ();
  float angleReading = mpu.getAngleZ();
  angleZ = angleReading - angleOffset;
  return angleZ;
}


void setupGyro(void) {
  Serial.begin(9600);
  Wire.begin();  // begin I2C wire class

  // initialize mpu with user selectable scale and range
  mpu.begin();
  delay(1000);
  mpu.calcOffsets(true,true);  // auto calibrate gyro and accels
  getAngle(true);
    // IMU must be stationary and level during calibration
}



void loopGyro() {
  mpu.update();  // get new measurements from IMU
  //printGyros();  // write gyro values to serial port
  //displayAngles();
  //printAccels();  // write accels to serial port
}



