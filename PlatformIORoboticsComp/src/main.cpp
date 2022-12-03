#include <Arduino.h>
//#include <HCSR04.h>
#include <gyro.h>
#include <MPU6050_light.h>
#include <Wire.h>
#include <Servo.h>

#define SPEED_MAX_TIME 200
#define UpperSpeedThreshold 55
#define LowerSpeedThreshold 10
// Pin Assignments
const int interruptPinR = 3;
const int interruptPinL = 2;
const int motorL = 5;
const int motorR = 11;
const int button = A1;

// information for the robot control
unsigned long totalRRot = 0; // Encoder value from the interrupt function RIGHT
unsigned long totalLRot = 0; // Encoder value from the interrupt function LEFT

int motorLS = 0;
int motorRS = 0;

class IRLight
{
public:
    int reciever;
    int light;
    IRLight(int pinRec, int pinLight) { // Constructor with parameters
        reciever = pinRec;
        light = pinLight;
    }
    void setup(){
        pinMode(light, OUTPUT);
    }
    int IRBias = 100;

    void calibrateIR(){ 
        int bias = 0;
        digitalWrite(light, LOW);
        for (int i = 0; i < 10; i++)
        {
            bias = bias + analogRead(reciever);
            delay(30);
        }
        IRBias = bias/10;    
        Serial.print("Calibration complete. Bias set to (F, B): ");
        Serial.println(IRBias);
    } 

    int readIR(){
        //digitalWrite(light, HIGH);
        int value = analogRead(reciever) - IRBias;
        //Serial.print("IR value");
        //Serial.println(value);
        return value;
    }
    int start(){
        digitalWrite(light, LOW);
        if(readIR() > 100){
            Serial.println("go");
            return 1;
        }
        else
            return 0;
        //digitalWrite(light, HIGH);
    }

};
IRLight front(A6,7);
//IRLight back(A7, 6);

Servo frontServo;
Servo backServo;

int checkStall(){
  static bool setTime = false;
  static unsigned long nowTime = 0;
  static unsigned long pastRRot = 0;
  static unsigned long pastLRot = 0;
  static unsigned long freqSpeed = 200;

  if (setTime == false){        //only will happen on the initialization of the function
        nowTime = millis();
        pastRRot = totalRRot;
        pastLRot = totalLRot;
        setTime = true;
  }
   if ((millis() - nowTime)>= freqSpeed){
        int encoderCountL = (totalLRot - pastLRot);
        int encoderCountR = (totalRRot - pastRRot);
        setTime = false;
        int averageCount  = (encoderCountL + encoderCountR)/2;
        Serial.print(">Average Count:");
        Serial.println(averageCount);
        return averageCount;
    }
    else
        return UpperSpeedThreshold - 1;
}

void addRotR()
{
    totalRRot += 1;
}
void addRotL()
{
    totalLRot += 1;
}

// Define SM states
typedef enum
{
    WAIT,
    START,
    SUPER_SPEED,
    STRAIGHT_MAINTAIN,
    TURN_MAINTAIN,
    STOP
} sm_state_t;

// SM Variables
sm_state_t currentState;

void SM_init()
{
    currentState = WAIT;
}

void SM_tick()
{
    // Read Values
    static unsigned long pastTime = 0;
    static int speedTimer = 0;
    static int start = 0;
    static int turn = turnDetect();
    static int notTurn = notTurning();
    int angle = getAngle(false);
    int buttonValue = digitalRead(button);
    if(!buttonValue){
        // currentState = STOP;
        start = true;
    }
    // SM Transitions
    switch (currentState)
    {
    case STOP:
        break;
    case WAIT:
        //start = front.start();
        if(start){
            frontServo.write(165);
            backServo.write(5);
            delay(230);
            currentState = START;
        }
        break;
    case START:
        //turn = turnDetect();
        if (turn)
            currentState = TURN_MAINTAIN;
        break;
    case STRAIGHT_MAINTAIN:
        turn = turnDetect();
        if (turn)
            currentState = TURN_MAINTAIN;
        break;
    case TURN_MAINTAIN:
        notTurn = notTurning();
        if (notTurn){
            speedTimer = millis() - pastTime;
            if(angle > 160){
                currentState = SUPER_SPEED;
            }
            else 
                currentState = STRAIGHT_MAINTAIN;
            getAngle(true);
        }
        break;
    case SUPER_SPEED:
        speedTimer = millis() - pastTime;
        if(speedTimer >= SPEED_MAX_TIME){
            currentState = STRAIGHT_MAINTAIN;
            pastTime = millis();
        }
        break;
    }

    // SM Actions
    switch (currentState)
    {
    case WAIT:
        motorLS = 0;
        motorRS = 0;
        break;
    case START:
        motorLS = 70;
        motorRS = 130;
        break;
    case STRAIGHT_MAINTAIN:             //straight with left
        motorLS = 65;
        motorRS = 125;
        break;
    case TURN_MAINTAIN:                 //left turn
        motorLS = 65;
        motorRS = 125;
        break;
    case SUPER_SPEED:
        motorLS = 248;
        motorRS = 255;
        break;
    case STOP:
        motorLS = 0;
        motorRS = 0;
        break;
    }

    analogWrite(motorR, motorRS);
    analogWrite(motorL, motorLS);
    Serial.print(">State:");
    Serial.println(currentState);
}

void setup() 
{
    Serial.begin(9600);
    // pin modes and interrupts
    pinMode(button, INPUT_PULLUP);      //should be 0V on press and 5v on no press
    front.setup();
    frontServo.attach(9);
    backServo.attach(6);
    frontServo.write(165);
    backServo.write(5);
    pinMode(motorL, OUTPUT);
    pinMode(motorR, OUTPUT);
    pinMode(interruptPinR, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPinR), addRotR, CHANGE);
    pinMode(interruptPinL, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPinL), addRotL, CHANGE);
    totalLRot = 0;
    totalRRot = 0;
    setupGyro();
    SM_init();
    int buttonValue = digitalRead(button);
    while(buttonValue){
        buttonValue = digitalRead(button);
        Serial.println("waiting");
        delay(100);
    }
    frontServo.write(0);
    backServo.write(170);
    front.calibrateIR();
    Serial.println("Setup Complete");
}

void loop(){
    SM_tick();
    loopGyro();
}

