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

uint16_t echo = 21;
uint16_t trig = 12;
double distance;


void isr()
{
    static uint32_t timer = 0;
    if (digitalRead(echo) == HIGH) {
        timer = micros();
    } else {
        distance = 0.0343 * (micros() - timer) / 2.0;
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
        return LineFollow;
    }
    if (c != Dab::Up && c != Dab::Down && c != Dab::Right && c != Dab::Left)
    {
        robot->stop();
    }
    return Manual;
}


int lineFollowing(Robot *robot)
{
    static int substate = 0;
    static int prev =0;
    static uint32_t prevTime = 0;
    // Check the ping sensor
    // read calibrated sensor values and obtain a measure of the line position
    // from 0 to 7000 (for a white line, use readLineWhite() instead)
    static uint32_t updateTime = 0;
    static double radius = 25.0;
    char c = dabble.input();
    if (c == Dab::X) {
        radius = 25.0;
        robot->stop();
        return Manual;
    }

    if (substate == 0) {
        if (distance > 5) {
            substate = 1;
        }
        return LineFollow;
    }

    if (distance < 5) {
        robot->rotate(180, 180);
        substate = 2;
    }

    if (substate == 2) {
        if (robot->isNotMoving()) {
            substate = 1;
        }
        return LineFollow;
    }

    int16_t position = qtr.readLineBlack(sensorValues);

    int16_t difference = position - 3500;
    const double radiusMax = 25;
    // double radius = sign(difference) * (((radiusMax - 1.0) / 3500.0) * abs(difference) + radiusMax);
    const double speed = 5;
    const double moveSpeed = speed * 3;
    const double arcSpeed = speed * 0.75;

    uint16_t diff = abs(difference);
    int s = sign(difference);

    const int RIGHT = 0;
    const int LEFT = 7;
    bool pass = false;
    for (int i = 0; i < 8; i++) {
        if (sensorValues[i] < 400) {
            pass = true;
        }
    }
    if (!pass)  {
        robot->arcMove(arcSpeed, 1.5*prev);
        return LineFollow;
    }
    const uint16_t delayTime = 100;
    if (sensorValues[RIGHT] < 600) {
        // if (millis() - prevTime > delayTime || prev == 1) {
            prev = 1;
            robot->arcMove(arcSpeed, 1.5);
            prevTime = millis();
        // }
    } else if (sensorValues[LEFT] < 600 || prev == -1) {
        // if (millis() - prevTime > delayTime) {
            prev = -1;
            robot->arcMove(arcSpeed, -1.5);
            prevTime = millis();
        // }
    } else if (sensorValues[LEFT-1] < 4 || prev == -1) {
        // if (millis() - prevTime > delayTime) {
            prev = -1;
            robot->arcMove(arcSpeed, -1.5);
            prevTime = millis();
        // }
    } else if (sensorValues[RIGHT+1] < 4 || prev == -1) {
        // if (millis() - prevTime > delayTime) {
            prev = 1;
            robot->arcMove(arcSpeed, 1.5);
            prevTime = millis();
        // }
    } else if (diff < 100) {
        robot->move(moveSpeed);
    } else if (diff < 500) {
        robot->arcMove(speed, 4*s);
    } else if (diff < 1000) {
        robot->arcMove(speed, 2*s);
    } else if (diff < 1500) {
        robot->arcMove(speed, 1*s);
    } else {
        robot->arcMove(speed, 1*s);
    }
    return LineFollow;
}


int calibrateSensorArray(Robot* robot)
{
    static uint16_t calibrationIdx = 0;
    static int direction = 1;
    if (calibrationIdx == 0) {
        qtr.resetCalibration();
        digitalWrite(13, HIGH);
    }
    if (robot->isNotMoving()) {
        Serial.println("Moving");
        robot->move(4*direction, 6);
        direction = direction < 0 ? 1 : -1;
    }
    if (calibrationIdx < 400) {
        logv(calibrationIdx);
        calibrationIdx++;
        qtr.calibrate();
    } else {
        calibrationIdx = 0;
        digitalWrite(13, LOW);
        return Manual;
    }
    return Calibration;
}

void algorithm(Robot *robot)
{
    static RobotState state = Manual;
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
    case Calibration:
        state = calibrateSensorArray(robot);
        break;
    default:
        state = Manual;
        break;
    }
}

Robot r(algorithm);

ISR(TIMER4_COMPA_vect) {
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
    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
    Serial.begin(115200);
    pinMode(13, OUTPUT);
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    digitalWrite(13, LOW);
    
    
    cli();//stop interrupts

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

    attachInterrupt(digitalPinToInterrupt(echo), isr, CHANGE);
}

int pos = 0;

void loop()
{
    // digitalWrite(trig, 1);
    r.tick();
}
