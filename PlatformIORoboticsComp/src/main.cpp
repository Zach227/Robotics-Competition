#include <Arduino.h>

//information for the robot control
int checkStallCount = 20;
unsigned long currentTime = 0;
unsigned long triggerTime;
unsigned long pastRRot = 0;
unsigned long pastLRot = 0;
unsigned long totalRRot = 0;
unsigned long totalLRot = 0;
unsigned long leftCount = 0;     //Encoder value from the interrupt function LEFT
unsigned long rightCount = 0;    //Encoder value from the interrupt function RIGHT
//Pin Assignments
const int interruptPinR = 3;
const int interruptPinL = 2;
const int motorLS = 10;
const int motorRS = 11;
//Adjustable speed and movement properties
int leftDutyC = 100;
int rightDutyC = 100;
int encoderCountL = 0;
int encoderCountR = 0;
int freqSpeed = 100;
unsigned long nowTime = 0;
bool setTime = false;
bool trigger = false;
int speedMotor = 50;

void checkStall(){
    if (setTime == false){        //only will happen on the initialization of the function
        nowTime = millis();
        pastRRot = totalRRot;
        pastLRot = totalLRot;
        setTime = true;
        Serial.println("set time");
    }
    if ((millis() - nowTime) >= (unsigned long) freqSpeed){
        encoderCountL = (totalLRot - pastLRot);
        encoderCountR = (totalRRot - pastRRot);
        Serial.println(encoderCountR);
        if(millis() - triggerTime >= 2000){
          if (encoderCountL <= checkStallCount || encoderCountL <= checkStallCount){
            triggerTime = millis();
            trigger = true;
            //Serial.println("trigger on");
          }
        }
        setTime = false;
        Serial.print("trueee");
    }
}

void moveForward(int leftRotation, int rightRotation, int speedL, int speedR) {
    rightDutyC = speedR;
    leftDutyC = speedL;
    Serial.print("Moving Forward");
    rightCount = (rightRotation + totalRRot);
    leftCount = (leftRotation + totalLRot);
    while (totalLRot <= leftCount){          //turn on the motors while rotations are less than the value
        analogWrite(motorLS, leftDutyC);
        analogWrite(motorRS, rightDutyC);
        if (totalRRot >= rightCount){         //this will turn off the right motor if the right side reaches the rotation count first
            analogWrite(motorRS,0);
            Serial.println("Motor Right Stopped First");
        }
    }
     analogWrite(motorLS, 0);
    while (totalRRot < rightCount){           //then it will check to see if the right motor should stay on or off
        analogWrite(motorRS,rightDutyC);
        if (trigger == true){
          leftCount = leftCount - leftRotation;
          rightCount = rightCount - rightRotation;
        }
    }
    analogWrite(motorLS, 0);
    analogWrite(motorRS,0);
    Serial.println("Movement Completed");
}
void moveBackward(int leftRotation, int rightRotation, int speedL, int speedR) {
    rightDutyC = speedR;
    leftDutyC = speedL;
    Serial.print("Moving Forward");
    rightCount = (rightRotation + totalRRot);
    leftCount = (leftRotation + totalLRot);
    while (totalLRot <= leftCount){          //turn on the motors while rotations are less than the value
        analogWrite(motorLS, leftDutyC);
        analogWrite(motorRS, rightDutyC);
        if (totalRRot >= rightCount){         //this will turn off the right motor if the right side reaches the rotation count first
            analogWrite(motorRS,0);
            Serial.println("Motor Right Stopped First");
        }
    }
     analogWrite(motorLS, 0);
    while (totalRRot < rightCount){           //then it will check to see if the right motor should stay on or off
        analogWrite(motorRS,rightDutyC);
        if (trigger == true){
          leftCount = leftCount - leftRotation;
          rightCount = rightCount - rightRotation;
        }
    }
    analogWrite(motorLS, 0);
    analogWrite(motorRS,0);
    Serial.println("Movement Completed");
}
void turnLeft(int turnRotate, int speedM) {
    rightDutyC = speedM;
    leftDutyC = speedM;
    Serial.println("Moving LEFT");
    rightCount = (turnRotate + totalRRot);
    leftCount = (turnRotate + totalLRot);
    while (totalLRot <= leftCount){          //turn on the motors while rotations are less than the value
        analogWrite(motorLS, leftDutyC);
        analogWrite(motorRS, rightDutyC);
        if (totalRRot >= rightCount){         //this will turn off the right motor if the right side reaches the rotation count first
            analogWrite(motorRS,0);
        }
    }
    analogWrite(motorLS,0);
    while (totalRRot < rightCount){           //then it will check to see if the right motor should stay on or off
        analogWrite(motorRS,rightDutyC);
    }
    analogWrite(motorLS, 0);
    analogWrite(motorRS,0);
    Serial.println("turn completed");
}
void turnRight(int turnRotate, int speedM) {
    rightDutyC = speedM;
    leftDutyC = speedM;
    Serial.println("Moving RIGHT");
    rightCount = (turnRotate + totalRRot);
    leftCount = (turnRotate + totalLRot);
    while (totalLRot <= leftCount){          //turn on the motors while rotations are less than the value
        analogWrite(motorLS, leftDutyC);
        analogWrite(motorRS, rightDutyC);
        if (totalRRot >= rightCount){         //this will turn off the right motor if the right side reaches the rotation count first
            analogWrite(motorRS, 0);
        }
    }
    analogWrite(motorLS, 0);
    while (totalRRot < rightCount){           //then it will check to see if the right motor should stay on or off
        analogWrite(motorRS, rightDutyC);
    }
    analogWrite(motorLS, 0);
    analogWrite(motorRS, 0);
    Serial.println("turn completed");
}
void addRotR(){
    totalRRot += 1;
}
void addRotL(){
    totalLRot += 1;
}


void setup() {
    Serial.begin(9600);
    // pin modes and interrupts
    pinMode(motorLS, OUTPUT);
    pinMode(motorRS, OUTPUT);
    pinMode(interruptPinR, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPinR), addRotR, CHANGE);
    pinMode(interruptPinL, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPinL), addRotL, CHANGE);
    
    Serial.println("Setup Complete");
    delay(2000);
}

void loop() {
  moveForward(1000, 1000, 100, 75);
  moveForward(1000, 1000, 75, 100);
}
