#include <Arduino.h>
#include <HCSR04.h>
#include <gyro.h>
#include <MPU6050_light.h>
#include <Wire.h>

#define SPEED_TIME_MAX 50
// Pin Assignments
const int interruptPinR = 3;
const int interruptPinL = 2;
const int motorL = 10;
const int motorR = 11;
const int buttonFrontPin = A2;
const int buttonBackPin = A3;

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

float leftWallDistance(int target)
{
    // Every 500 miliseconds, do a measurement using the sensor and print the distance in centimeters.
    float distance = distanceSensor.measureDistanceCm();
    if (distance != -1 && distance < 40)
    {
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

void ultrasonic()
{
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
}

void setup()
{
    Serial.begin(9600);
    // pin modes and interrupts
    pinMode(buttonBackPin, INPUT_PULLUP);
    pinMode(buttonFrontPin, INPUT_PULLUP);
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
    setupGyro();
}

// void statemachine()
// {
//     static int state = 0;
//     static int buttonFront = 0;
//     static int buttonBack = 0;
//     buttonFront = digitalRead(buttonFrontPin);
//     buttonBack = digitalRead(buttonBackPin);
//     if ((buttonBack == 0) && (buttonFront == 1))
//         state = 1;
//     else if ((buttonBack == 1) && (buttonFront == 0))
//         state = 2;
//     else if ((buttonBack == 0) && (buttonFront == 0))
//         state = 3;
//     else
//         state = 0;
//     switch (state)
//     {
//     case 0: // both buttons pressed OK
//         motorLS = 155;
//         motorRS = 159;
//         break;
//     case 1: // the back button is not pushed increase left motor speed
//         motorLS = 155;
//         motorRS = 155;
//         break;
//     case 2: // the front button is not pushed increase right motor speed
//         motorLS = 120;
//         motorRS = 155;
//         break;
//     case 3: // both buttons not pushed increase right motor speed
//         motorLS = 90;
//         motorRS = 190;
//         break;
//     }
//     analogWrite(motorR, motorRS);
//     analogWrite(motorL, motorLS);
//     Serial.println(state);
// }

// Define SM states
typedef enum
{
    START,
    GO_TO_WALL,
    STRAIGHT_PUSH_BACK_BTN,
    STRAIGHT_PUSH_FRONT_BTN,
    STRAIGHT_MAINTAIN,
    SUPER_SPEED,
    TURN_PUSH_FRONT_BTN,
    TURN_MAINTAIN
} sm_state_t;

// SM Variables
sm_state_t currentState;

void SM_init()
{
    currentState = GO_TO_WALL;
}

void SM_tick()
{
    // Read Values
    static int buttonFront = 0;
    static int buttonBack = 0;
    static int speedTimer = 0;
    buttonFront = digitalRead(buttonFrontPin);
    buttonBack = digitalRead(buttonBackPin);
    int turn = turnDetect();
    //Serial.println(turn);
    // SM Transitions
    switch (currentState)
    {
    case START:
        currentState = GO_TO_WALL;
        break;
    case GO_TO_WALL:
        if (buttonFront)
            currentState = STRAIGHT_PUSH_BACK_BTN;
        break;
    case STRAIGHT_PUSH_BACK_BTN:
        if (turn)
            currentState = TURN_PUSH_FRONT_BTN;
        else if (buttonBack)
            currentState = STRAIGHT_MAINTAIN;
        break;
    case STRAIGHT_PUSH_FRONT_BTN:
        if (turn)
            currentState = TURN_PUSH_FRONT_BTN;
        else if (buttonFront)
            currentState = STRAIGHT_MAINTAIN;
        break;
    case SUPER_SPEED:
        if(speedTimer >= SPEED_TIME_MAX):
            currentState = STRAIGHT_MAINTAIN;
        break;
    case STRAIGHT_MAINTAIN:
        if (turn)
            currentState = TURN_PUSH_FRONT_BTN;
        else if (!buttonFront)
            currentState = STRAIGHT_PUSH_FRONT_BTN;
        else if (!buttonBack)
            currentState = STRAIGHT_PUSH_BACK_BTN;
        break;
    case TURN_PUSH_FRONT_BTN:
        if (!turn)
            currentState = STRAIGHT_MAINTAIN;
        else if (buttonFront)
            currentState = TURN_MAINTAIN;
        break;
    case TURN_MAINTAIN:
        if (!turn){
            currentState = SUPER_SPEED;
            speedTimer = 0;
        }            
        break;
    }

    // SM Actions
    switch (currentState)
    {
    case START:
        break;
    case GO_TO_WALL:        //slight turn left
        motorLS = 135;
        motorRS = 150;
        break;
    case STRAIGHT_PUSH_FRONT_BTN:       //hard turn left
        motorLS = 70;
        motorRS = 115;
        break;
    case STRAIGHT_PUSH_BACK_BTN:        //go straight very slight left
        motorLS = 150;
        motorRS = 155;
        break;
    case STRAIGHT_MAINTAIN:             //straight with left
        motorLS = 95;
        motorRS = 110;
        break;
    case SUPER_SPEED:             //super straight
        motorLS = 230;
        motorRS = 255;
        speedTimer++;
        break;
    case TURN_PUSH_FRONT_BTN:               //hard left
        motorLS = 55;
        motorRS = 80;
        break;
    case TURN_MAINTAIN:                 //left turn
        motorLS = 100;
        motorRS = 130;
        break;
    }

    analogWrite(motorR, motorRS);
    analogWrite(motorL, motorLS);
    Serial.println(currentState);
}

void loop(){
    SM_tick();
    //loopGyro();
}
