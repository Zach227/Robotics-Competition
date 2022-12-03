#include <Arduino.h>
#include <gyro.h>
#include <Servo.h>

// Pin Assignments
const int interruptPinR = 3;
const int interruptPinL = 2;
const int motorL = 5;
const int motorR = 11;
const int button = A1;

const int ledblue = 7;
const int ledyellow = 8;

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
        digitalWrite(light, LOW);
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
        return 15 - 1;
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
    QUICKSTART,
    START,
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
    static int start = 0;
    int turn = turnDetect();
    int notTurn = notTurning();
    int buttonValue = digitalRead(button);
    if(!buttonValue){
        currentState = STOP;
        //start = true;
    }

    // SM Transitions
    switch (currentState)
    {
    case STOP:
        break;
    case WAIT:
        start = front.start();
        if(start){
            frontServo.write(165);
            backServo.write(5);
            delay(250);
            currentState = QUICKSTART;
        }
        break;
    case QUICKSTART: 
        break;
    case START:
        if (turn)
            currentState = TURN_MAINTAIN;
        break;
    case STRAIGHT_MAINTAIN:
        if (turn)
            currentState = TURN_MAINTAIN;
        break;
    case TURN_MAINTAIN:
        if (notTurn){
            currentState = STRAIGHT_MAINTAIN;
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
    case QUICKSTART:
        motorLS = 235;
        motorRS = 255;
        analogWrite(motorR, motorRS);
        analogWrite(motorL, motorLS);
        delay(300);
        currentState = START;
    case START:
        motorLS = 45;
        motorRS = 100;
        break;
    case STRAIGHT_MAINTAIN:             //straight with left
        motorLS = 50;
        motorRS = 140;
        break;
    case TURN_MAINTAIN:                 //left turn
        motorLS = 50;
        motorRS = 140;
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
    if(currentState == TURN_MAINTAIN)
        digitalWrite(ledblue, HIGH);
    else
        digitalWrite(ledblue, LOW);
    if(currentState == STRAIGHT_MAINTAIN)
        digitalWrite(ledyellow, HIGH);
    else
        digitalWrite(ledyellow, LOW);
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
    pinMode(ledblue, OUTPUT);
    pinMode(ledyellow, OUTPUT);
    pinMode(motorL, OUTPUT);
    pinMode(motorR, OUTPUT);
    // pinMode(interruptPinR, INPUT_PULLUP);
    // attachInterrupt(digitalPinToInterrupt(interruptPinR), addRotR, CHANGE);
    // pinMode(interruptPinL, INPUT_PULLUP);
    // attachInterrupt(digitalPinToInterrupt(interruptPinL), addRotL, CHANGE);
    totalLRot = 0;
    totalRRot = 0;
    setupGyro();
    SM_init();
    digitalWrite(ledyellow, HIGH);
    digitalWrite(ledblue, HIGH);
    int buttonValue = digitalRead(button);
    while(buttonValue){
        buttonValue = digitalRead(button);
        Serial.println("waiting");
        delay(100);
    }
    digitalWrite(ledyellow, LOW);
    digitalWrite(ledblue, LOW);
    frontServo.write(0);
    backServo.write(170);
    front.calibrateIR();
    Serial.println("Setup Complete");
}

void loop(){
    SM_tick();
}
