#include <Arduino.h>
//Pin Assignments
const int interruptPinR = 3;
const int interruptPinL = 2;
const int motorL = 10;
const int motorR = 11;

//information for the robot control
unsigned long totalRRot = 0;       //Encoder value from the interrupt function RIGHT 
unsigned long totalLRot = 0;        //Encoder value from the interrupt function LEFT

int measureSpeedDiffL(int target){
    static int period = 60; //measure every period 
    static int lastCount = 0;
    unsigned long nowTime = millis();
    static unsigned long nextToggle = 0;  //time set when the measurement will be take again
    if(nowTime > nextToggle){
        int speed = totalLRot - lastCount;
        lastCount = totalLRot;
        nextToggle = millis() + period;
        return target - speed;
    }
    else{
        return 0;
    }
}

int measureSpeedDiffR(int target){
    static int period = 60; //measure every period 
    static int lastCount = 0;
    unsigned long nowTime = millis();
    static unsigned long nextToggle = 0;  //time set when the measurement will be take again
    if(nowTime > nextToggle){
        int speed = totalRRot - lastCount;
        lastCount = totalRRot;
        nextToggle = millis() + period;
        return target - speed;
    }
    else{
        return 0;
    }
}

int motorLS = 155;
int motorRS = 155;

void adjustSpeed(int speedL, int speedR){
    int goalSpeedL = speedL;
    int differenceL = measureSpeedDiffL(goalSpeedL);
    if(differenceL > 7 && motorLS < 250){
        motorLS = motorLS + 2; 
        //Serial.print(motorLS);
        //Serial.println("big change up");
    }
    else if(differenceL > 1 && motorLS < 252){
        motorLS = motorLS + 1; 
        //Serial.println("little change up");
    }
    else if(differenceL < -7 && motorLS > 20){
        motorLS = motorLS - 2; 
        //Serial.println("big change down");
    }
    else if(differenceL < -1 && motorLS > 20){
        motorLS = motorLS - 1; 
        //Serial.println("litte change down");
    }
    analogWrite(motorL, motorLS);

    int goalSpeedR = speedR;
    int differenceR = measureSpeedDiffR(goalSpeedR);
    if(differenceR > 7 && motorRS < 250){
        motorRS = motorRS + 2; 
        //Serial.print(motorRS);
        //Serial.println("big change up");
    }
    else if(differenceR > 1 && motorRS < 252){
        motorRS = motorRS + 1; 
        //Serial.println("little change up");
    }
    else if(differenceR < -7 && motorRS > 20){
        motorRS = motorRS - 2; 
        //Serial.println("big change down");
    }
    else if(differenceR < -1 && motorRS > 20){
        motorRS = motorRS - 1; 
        //Serial.println("litte change down");
    }
    analogWrite(motorR, motorRS);



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
    pinMode(motorL, OUTPUT);
    pinMode(motorR, OUTPUT);
    pinMode(interruptPinR, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPinR), addRotR, CHANGE);
    pinMode(interruptPinL, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPinL), addRotL, CHANGE);
    delay(2000);
    Serial.println("Setup Complete");
    totalLRot = 0;
    totalRRot = 0;
    analogWrite(motorR, motorRS);
    analogWrite(motorL, motorLS);
}

void loop() {
    //delay(30);
    adjustSpeed(10, 10);
}