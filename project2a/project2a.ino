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
#include <Servo.h>
#include <QTRSensors.h>
#include "src/CustomPID.hpp"

#define logn(arg) Serial.println(arg)
#define logv(arg)             \
    Serial.print(#arg " = "); \
    Serial.println(arg)
#define log(arg) Serial.print(arg)

#include <stdlib.h>

#include "motor.hpp"
#include "dabble.hpp"
#include "robot.hpp"

Dab dabble;

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
    }
    if (c != Dab::Up && c != Dab::Down && c != Dab::Right && c != Dab::Left)
    {
        robot->stop();
    }
    return Manual;
}

int lineFollowing(Robot *robot)
{
    if (!robot->isMoving())
    {
        robot->constantArcMove(5, 10);
    }
    return LineFollow;
}


int calibrateSensorArray(Robot* robot)
{
    Serial.println("Calibrating");
    static uint16_t calibrationIdx = 0;
    if (calibrationIdx < 400) {
        qtr.calibrate();
    } else {
        calibrationIdx = 0;
        return LineFollow;
    }
}

void algorithm(Robot *robot)
{
    static RobotState state = LineFollow;
    Serial.println(state);
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
    default:
        state = Manual;
        break;
    }
}

Robot r(algorithm);

QTRSensors qtr;

const int buttonPin = 53;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

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
}

int pos = 0;

void loop()
{
    r.tick();
}
