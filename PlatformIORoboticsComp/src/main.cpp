#include <Arduino.h>
#include <HCSR04.h>
// Pin Assignments
const int interruptPinR = 3;
const int interruptPinL = 2;
const int motorL = 10;
const int motorR = 11;

// information for the robot control
unsigned long totalRRot = 0; // Encoder value from the interrupt function RIGHT
unsigned long totalLRot = 0; // Encoder value from the interrupt function LEFT

int measureSpeedDiffL(int target)
{
    static int period = 60; // measure every period
    static int lastCount = 0;
    unsigned long nowTime = millis();
    static unsigned long nextToggle = 0; // time set when the measurement will be take again
    if (nowTime > nextToggle)
    {
        int speed = totalLRot - lastCount;
        lastCount = totalLRot;
        nextToggle = millis() + period;
        return target - speed;
    }
    else
    {
        return 0;
    }
}

int measureSpeedDiffR(int target)
{
    static int period = 60; // measure every period
    static int lastCount = 0;
    unsigned long nowTime = millis();
    static unsigned long nextToggle = 0; // time set when the measurement will be take again
    if (nowTime > nextToggle)
    {
        int speed = totalRRot - lastCount;
        lastCount = totalRRot;
        nextToggle = millis() + period;
        return target - speed;
    }
    else
    {
        return 0;
    }
}

int motorLS = 155;
int motorRS = 155;

void adjustSpeed(int speedL, int speedR)
{
    int goalSpeedL = speedL;
    int differenceL = measureSpeedDiffL(goalSpeedL);
    if (differenceL > 7 && motorLS < 250)
    {
        motorLS = motorLS + 2;
        // Serial.print(motorLS);
        Serial.println("big change up");
    }
    else if (differenceL > 1 && motorLS < 252)
    {
        motorLS = motorLS + 1;
        Serial.println("little change up");
    }
    else if (differenceL < -7 && motorLS > 20)
    {
        motorLS = motorLS - 2;
        Serial.println("big change down");
    }
    else if (differenceL < -1 && motorLS > 20)
    {
        motorLS = motorLS - 1;
        Serial.println("litte change down");
    }
    analogWrite(motorL, motorLS);

    int goalSpeedR = speedR;
    int differenceR = measureSpeedDiffR(goalSpeedR);
    if (differenceR > 7 && motorRS < 250)
    {
        motorRS = motorRS + 2;
        // Serial.print(motorRS);
        // Serial.println("big change up");
    }
    else if (differenceR > 1 && motorRS < 252)
    {
        motorRS = motorRS + 1;
        // Serial.println("little change up");
    }
    else if (differenceR < -7 && motorRS > 20)
    {
        motorRS = motorRS - 2;
        // Serial.println("big change down");
    }
    else if (differenceR < -1 && motorRS > 20)
    {
        motorRS = motorRS - 1;
        // Serial.println("litte change down");
    }
    analogWrite(motorR, motorRS);
}

void addRotR()
{
    totalRRot += 1;
}
void addRotL()
{
    totalLRot += 1;
}

const byte triggerPin = 6;
const byte echoPin = 5;
UltraSonicDistanceSensor distanceSensor(triggerPin, echoPin);

float leftWallDistance(int target){
    // Every 500 miliseconds, do a measurement using the sensor and print the distance in centimeters.
    float distance = distanceSensor.measureDistanceCm();
    if (distance != -1 && distance < 40){
        float difference = distance - target;
        // Serial.println(difference);
        return difference;
    }
    else
    {
        return 0;
    }
}

int rightSpeed = 100;
void ultrasonicAdjust(int target)
{
    int difference = leftWallDistance(target);
    if (difference > 2)
    {
        rightSpeed++;
    }
    if (difference < -2)
    {
        rightSpeed--;
    }
    if (rightSpeed > 255)
    {
        rightSpeed = 255;
    }
    if (rightSpeed < 20)
    {
        rightSpeed = 20;
    }
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
    static int speedL = 100;
    static int speedR = 100;
    float distance = distanceSensor.measureDistanceCm();
    if (distance > 30)
    {
        speedL = 90;
        speedR = 125;
        
    }
    else if (distance < 15)
    {
        speedL = 150;
        speedR = 90;
    }
    else
    {
        speedL = 100;
        speedR = 100;
    }
    analogWrite(motorL, speedL);
    analogWrite(motorR, speedR);
    // ultrasonicAdjust(10); //keep it 10cm from the wall
    // Serial.println(rightSpeed);
    // analogWrite(motorL, 100);
    // analogWrite(motorR, rightSpeed);
}