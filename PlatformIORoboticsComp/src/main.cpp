#include <Arduino.h>
//#include <HCSR04.h>
#include <gyro.h>
#include <MPU6050_light.h>
#include <Wire.h>
// Pin Assignments
const int interruptPinR = 3;
const int interruptPinL = 2;
const int motorL = 10;
const int motorR = 11;
// const int IRFront = A6;
// const int IRlightF = 5;
// const int IRBack = A7;
// const int IRlightB = 6;
const int button = A1;

// information for the robot control
unsigned long totalRRot = 0; // Encoder value from the interrupt function RIGHT
unsigned long totalLRot = 0; // Encoder value from the interrupt function LEFT

int motorLS = 100;
int motorRS = 100;

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
        int value = analogRead(reciever) - IRBias;
        return value;
    }

};
IRLight front(A6,5);
IRLight back(A7, 6);

bool checkStall(){
  static bool setTime = false;
  static unsigned long nowTime = 0;
  static unsigned long pastRRot = 0;
  static unsigned long pastLRot = 0;
  static int triggerTime = 0; 
  static unsigned long freqSpeed = 200;
  static int checkStallCount = 1;

  if (setTime == false){        //only will happen on the initialization of the function
        nowTime = millis();
        pastRRot = totalRRot;
        pastLRot = totalLRot;
        setTime = true;
  }
   if ((millis() - nowTime)>= freqSpeed){
        int encoderCountL = (totalLRot - pastLRot);
        int encoderCountR = (totalRRot - pastRRot);
        /*Serial.print("right: ");
        Serial.println(encoderCountR);
        Serial.print("left: ");
        Serial.println(encoderCountL);
        Serial.print("time since trigger");
        Serial.println(millis() - triggerTime);*/
        if(millis() - triggerTime >= 1000){;
          setTime = false;
          if (encoderCountL <= checkStallCount){
            triggerTime = millis();
            Serial.println("triggered L");
            return true;
          }
          else if(encoderCountR <= checkStallCount){
            triggerTime = millis();
            Serial.println("triggered R");
            return true;
          }
          else{
            return false;
          }
        }
        else
            return false;
    }
    else{
        return false;
    }
}

void addRotR()
{
    totalRRot += 1;
}
void addRotL()
{
    totalLRot += 1;
}

void setup()
{
    Serial.begin(9600);
    // pin modes and interrupts
    pinMode(button, INPUT_PULLUP);      //should be 0V on press and 5v on no press
    front.setup();
    back.setup();
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
    setupGyro();
    int buttonValue = digitalRead(button);
    while(buttonValue){
        buttonValue = digitalRead(button);
        Serial.println("waiting");
        delay(100);
    }
    front.calibrateIR();
    back.calibrateIR();
}

// Define SM states
typedef enum
{
    START,
    GO_TO_WALL,
    STRAIGHT_MAINTAIN,
    TURN_COMING,
    TURN_MAINTAIN,
    TURN_FINISH
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
    int turn = turnDetect();
    float angle = getAngle(false);
    static unsigned long pastLRotation = 0;
    int rotationLDifference = totalLRot - pastLRotation;
    //Serial.println(turn);
    // SM Transitions
    switch (currentState)
    {
    case START:
        currentState = GO_TO_WALL;
        break;
    case GO_TO_WALL:
            currentState = STRAIGHT_MAINTAIN;
        break;
    case STRAIGHT_MAINTAIN:
        if(rotationLDifference > 175)//track long length is 275 rotations
            currentState = TURN_COMING;
        if (turn)
            currentState = TURN_MAINTAIN;
        break;
    case TURN_COMING:
        if(turn){
            currentState = TURN_MAINTAIN;
        }
        break;
    case TURN_MAINTAIN:
        if (angle >= 160){
            currentState = TURN_FINISH;
        }
        if (!turn){
            currentState = STRAIGHT_MAINTAIN;
            pastLRotation = totalLRot;
            getAngle(true);
        }
        break;
    case TURN_FINISH:
        if(!turn){
            currentState = STRAIGHT_MAINTAIN;
            pastLRotation = totalLRot;
            getAngle(true);
        }
    }

    // SM Actions
    switch (currentState)
    {
    case START:
        break;
    case GO_TO_WALL:        //slight turn left
        motorLS = 40;
        motorRS = 90;
        break;
    case STRAIGHT_MAINTAIN:             //straight with left
        motorLS = 40;
        motorRS = 90;
        break;
    case TURN_MAINTAIN:                 //left turn
        motorLS = 40;
        motorRS = 110;
        break;
    case TURN_COMING:
        motorLS = 40;
        motorRS = 110;
    case TURN_FINISH:
        motorLS = 40;
        motorRS = 90;
    }

    analogWrite(motorR, motorRS);
    analogWrite(motorL, motorLS);
    Serial.println(currentState);
}

void looop(){
    SM_tick();
    if(checkStall()){
        /*back up and try again*/
    }
    loopGyro();
}

void loop(){
    Serial.println(front.readIR());
}