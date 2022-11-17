#include <Arduino.h>
//Pin Assignments
const int interruptPinR = 3;
const int interruptPinL = 2;
const int motorLS = 10;
const int motorRS = 11;

//information for the robot control
unsigned long totalRRot = 0;       //Encoder value from the interrupt function RIGHT 
unsigned long totalLRot = 0;        //Encoder value from the interrupt function LEFT
bool trigger = false;


//Adjustable speed and movement properties
int leftDutyC = 100;
int rightDutyC = 100;
unsigned long leftCount = 0;     
unsigned long rightCount = 0; 
void moveForward(int LeftRotation, int RightRotation){
    currentRightRotation = totalRRot;
    currentLeftRotation = totalLRot;
    analogWrite(motorLS, 255);
    analogWrite(mortorRS, 255);
    while(((totalRRot - currentRightRotation)  < RightRotation) ||(((totalLRot - currentLeftRotation) < LeftRotation))){
        
    }
    if((totalRRot - currentRightRotation)  >= RightRotation){
            analogWrite(motorRS, 0 );
            while((totalLRot - currentLeftRotation) < LeftRotation){
            }
            analogWrite(motorLS, 0);
        }
    else{
        analogWrite(motorLS,0);
        while((totalRRot - currentRightRotation) < RightRotation){
        }
        analogWrite(motorRS, 0);
    }
    analogWrite(motorRS, 0);
    analogWrite(motorLS,0);

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
    delay(1000);
    analogWrite(motorLS, 255);
    analogWrite(motorRS, 255);
    delay(500);
    analogWrite(motorLS, 0);
    analogWrite(motorRS, 0);
    delay(1000);
    moveForward(10, 10);
    delay(4000);
}
