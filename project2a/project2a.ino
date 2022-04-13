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
        robot->move(5);
        break;
    case Dab::Down:
        robot->move(-5);
        break;
    case Dab::Left:
        robot->rotate(-90);
        break;
    case Dab::Right:
        robot->rotate(90);
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

    if (millis() - updateTime > 100) {
        int16_t position = qtr.readLineBlack(sensorValues);

        int16_t difference = position - 3500;
        const double radiusMax = 25;
        // double radius = sign(difference) * (((radiusMax - 1.0) / 3500.0) * abs(difference) + radiusMax);
        const double speed = 3;

        uint16_t diff = abs(difference);
        int s = sign(difference);

        if (diff < 100) {
            radius += 1;
        } else if (diff < 500) {
            radius -= 1;
        } else if (diff < 1000) {
            radius -= 2;
        } else if (diff < 1500) {
            radius -= 3;
        } else {
            radius = 0.1;
        }
        if (radius > radiusMax) {
            radius = radiusMax;
        } else if (radius < 1) {
            radius = 1;
        }
        robot->arcMove(speed, radius*s);

        updateTime = millis();
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
    logv(distance);
    r.tick();
}
