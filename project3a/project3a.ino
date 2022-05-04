/*
   Gamepad module provides three different mode namely Digital, JoyStick and Accerleometer.
   You can reduce the size of library compiled by enabling only those modules that you want to
   use. For this first define CUSTOM_SETTINGS followed by defining INCLUDE_modulename.
   Explore more on: https://thestempedia.com/docs/dabble/game-pad-module/
*/
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <Dabble.h>
#include <SD.h>
#include <stdio.h>
#include <stdarg.h>
// #include <Servo.h>
#include <QTRSensors.h>
#include "src/CustomPID.hpp"

#define sign(x) ((x) < 0 ? -1 : 1)
#define logn(arg) Serial.println(arg)
#define logv(arg)             \
    Serial.print(#arg " = "); \
    Serial.println(arg)
#define log(arg) Serial.print(arg)

#include <stdlib.h>

#include "motor.hpp"
#include "dabble.hpp"
#include "robot.hpp"

struct SensorData {
    static constexpr uint8_t erp = 21;
    int16_t dRight;
    static constexpr uint8_t elp = 18;
    int16_t dLeft;
    static constexpr uint8_t efp = 19;
    int16_t dFront;
};

SensorData data;

uint16_t trig = 12;

void isrF()
{
    static uint32_t timer = 0;
    if (digitalRead(data.efp) == HIGH) {
    // Serial.println("Front");
        timer = micros();
    } else {
        data.dFront = 0.0343 * (micros() - timer) / 2.0;
    }
}

void isrR()
{
    static uint32_t timer = 0;
    if (digitalRead(data.erp) == HIGH) {
    // Serial.println("Right");
        timer = micros();
    } else {
        data.dRight = 0.0343 * (micros() - timer) / 2.0;
    }
}

void isrL()
{
    static uint32_t timer = 0;
    // Serial.println("Left");
    if (digitalRead(data.elp) == HIGH) {
        timer = micros();
    } else {
        data.dLeft = 0.0343 * (micros() - timer) / 2.0;
    }
}

Dab dabble;

QTRSensors qtr;

const int buttonPin = 53;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

int manual(Robot *robot)
{
    auto c = dabble.input();
    switch (c)
    {
    case Dab::Up:
        robot->move(10);
        break;
    case Dab::Down:
        robot->move(-10);
        break;
    case Dab::Left:
        robot->rotate(-180);
        break;
    case Dab::Right:
        robot->rotate(180);
        break;
    case Dab::Select:
        robot->stop();
        return Calibration;
    case Dab::Start:
        robot->stop();
        return MazeSolve;
    }
    if (c != Dab::Up && c != Dab::Down && c != Dab::Right && c != Dab::Left)
    {
        robot->stop();
    }
    return Manual;
}


int lineFollowing(Robot *robot)
{}

double in, out, set;

static const int16_t goodDistance = 8;
static const int16_t speed = 80;
float kp = 0.8*speed/goodDistance;
float ki = 0.06;   
float kd = 0; //2.0*kp;   

CustomPID mpid(&in, &out, &set, kp, ki, kd, DIRECT);

double lastError = 0;

const int sign = 1;

int mazeSolve(Robot *r)
{
    static const int16_t foundDistance = 15;
    static const int16_t frontFound = 8;
    static bool pause;
    set = goodDistance;
    int16_t diff = data.dLeft - goodDistance;
    

    if (pause) {
        pause = r->isMoving();
        return MazeSolve;
    }
    // // dead end
    if (data.dFront < frontFound && data.dRight < foundDistance && data.dLeft < foundDistance) {
        r->rotate(90, sign*180); // turn around
        pause = true;
    } 
    
    // Front and right, go left
    else if (data.dFront < frontFound && data.dRight < foundDistance) {
        r->rotate(-90, sign*90); // turn left
        pause = true;
    }
    // front and left, go right
    else if (data.dFront < frontFound && data.dLeft < foundDistance) {
        r->rotate(90, sign*90);
        pause = true;
    }

    // Front. go left
    else if (data.dFront < frontFound) {
        r->rotate(90, sign*45);
        pause = true;
    }
    // else if (data.dLeft > 30) {
    //     r->move(3);
    // }
    else {
            // Find error based on targtet
    
        float error = goodDistance -  data.dLeft;
        Serial.print("right=");Serial.print(data.dLeft);
        // Compute the motor adjustment
        float delta = kp*error + kd*(error - lastError);
        Serial.print("error=");Serial.print(error);Serial.print("delta=");Serial.print(delta);
        // Change motor speeds.  
        //  If delta = 0, then PWM the same on both motors.
        //  If delta > 0, then speed greater on right wheel
        //  If delta < 0, then speed greater on the left wheel. 
        float rspeed = speed-delta;
        float lspeed = speed+delta;
        float minspeed = speed/2.2;
        // Limit the speeds between 0 and speed.
        if (lspeed > speed)
            lspeed = speed;
        else if (lspeed < minspeed)
            lspeed = minspeed;
        if (rspeed > speed)
            rspeed = speed;
        else if (rspeed < minspeed)
            rspeed = minspeed;  
        // in = data.dLeft;
        // mpid.Compute(millis());
        r->left_.pwm(lspeed);
        r->right_.pwm(rspeed);
        lastError = error;

        // logv(set);
    }

    return MazeSolve;
}


int calibrateSensorArray(Robot* robot)
{}

void algorithm(Robot *robot)
{
    static RobotState state = MazeSolve;
    dabble.tick();

    switch (state)
    {
    case Manual:
        state = manual(robot);
        break;
    case SdCard:
        break;
    case LineFollow:
        state = lineFollowing(robot);
        break;
    case MazeSolve:
        state = mazeSolve(robot);
        break;
    case Calibration:
        state = calibrateSensorArray(robot);
        break;
    default:
        state = Manual;
        break;
    }
}

Robot r(algorithm);

ISR(TIMER4_COMPA_vect)
{
    // Serial.println("Called");
    digitalWrite(trig, HIGH);
    digitalWrite(trig, LOW);
}


void setup()
{
    Serial.begin(115200);
    delay(100);
    r.init();
    pinMode(A9, INPUT);
    Serial.println("Setup");
    dabble.init();
    Serial.begin(115200);
    pinMode(13, OUTPUT);
    pinMode(trig, OUTPUT);
    pinMode(data.efp, INPUT);
    pinMode(data.erp, INPUT);
    pinMode(data.elp, INPUT);
    digitalWrite(13, LOW);
    
    
    cli();//stop interrupts

    mpid.SetMode(AUTOMATIC);

    //set timer4 interrupt at 1Hz
    TCCR4A = 0;// set entire TCCR1A register to 0
    TCCR4B = 0;// same for TCCR1B
    TCNT4  = 0;//initialize counter value to 0
    // set compare match register for 1hz increments
    OCR4A = 1000;// = (16*10^6) / (1*1024) - 1 (must be <65536)
    // turn on CTC mode
    TCCR4B |= (1 << WGM12);
    // Set CS12 and CS10 bits for 1024 prescaler
    TCCR4B |= (1 << CS12) | (1 << CS10);  
    // enable timer compare interrupt
    TIMSK4 |= (1 << OCIE4A);

    sei();//allow interrupts
    attachInterrupt(digitalPinToInterrupt(data.efp), isrF, CHANGE);
    attachInterrupt(digitalPinToInterrupt(data.erp), isrR, CHANGE);
    EIFR = 8;
    attachInterrupt(digitalPinToInterrupt(data.elp), isrL, CHANGE);
}

int pos = 0;

void loop()
{
    // digitalWrite(trig, 1);
    r.tick();
}
