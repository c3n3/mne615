/*
   Gamepad module provides three different mode namely Digital, JoyStick and Accerleometer.
   You can reduce the size of library compiled by enabling only those modules that you want to
   use. For this first define CUSTOM_SETTINGS followed by defining INCLUDE_modulename.
   Explore more on: https://thestempedia.com/docs/dabble/game-pad-module/
*/
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <Dabble.h>
#include <stdio.h>
#include <stdarg.h>

#include <PID_v1.h>

void _printf ( const char * format, ... )
{
  char buffer[500];
  va_list args;
  va_start (args, format);
  vsprintf (buffer,format, args);
  Serial.print(buffer);
  va_end (args);
}

// Controls a motor with two direction pins
template<int ENC_PIN>
class PidMotor {
    uint8_t dirPin_;
    uint8_t pwmPin_;
    PID pid_;
    const uint8_t updateInterval = 100;
    unsigned long prevTime_;
    uint32_t prevCount_;
    static uint32_t count_;
    double pidIn_;
    double pidOut_;
    double pidValue_;

public:
    PidMotor(uint8_t dirPin, uint8_t pwmPin)
        : dirPin_(dirPin), pwmPin_(pwmPin), pid_(&pidIn_, &pidOut_, &pidValue_, 0.013725, 0.08235, 0, DIRECT)
    {
        attachInterrupt(digitalPinToInterrupt(ENC_PIN),[&count_](){count_++;},RISING);
        pinMode(dirPin_, OUTPUT);
        pinMode(pwmPin_, OUTPUT);
        pid_.SetMode(AUTOMATIC);
    }

    // set dir
    void setForward()
    {
        digitalWrite(dirPin_, LOW);
    }

    // set dir
    void setBackward()
    {
        digitalWrite(dirPin_, HIGH);
    }

    // set speed based on float
    void setSpeed(double cps)
    {
        double set = ((cps < 0 ? -cps : cps)) ;

        if (cps < 0) {
            setBackward();
        } else {
            setForward();
        }
        pidValue_ = set;
    }

    void tick()
    {
        if (millis() - prevTime_ > updateInterval) {
            pidIn_ = ((double)(count_ - prevCount_) / (double)(millis() - prevTime_)) * 1000.0;
            pid_.Compute();
            Serial.println("-----------------");
            Serial.println(pidIn_);
            Serial.println(pidOut_);
            Serial.println(count_);
            Serial.println(pidValue_);
            analogWrite(pwmPin_, pidOut_);
            prevTime_ += updateInterval;
            prevCount_ = count_;
        }
    }
};

template<int ENC_PIN>
uint32_t PidMotor<ENC_PIN>::count_ = 0;


long eracount;
long elacount;

const uint8_t eraPin = 20;
const uint8_t elaPin = 21;
const uint8_t elaCPR = 3;
const uint8_t eraCPR = 6;


unsigned long time;

float countsToInches(long counts, uint8_t cpr)
{
    // 120 : 1 gear ratio
    // Circumference
    float pi = 3.1415;
    float circumferenceIn = 2.7 * pi;

    long revolutions = (float)counts / (120.0 *(float)cpr);
    return circumferenceIn * revolutions;
}

void setup() {
    Serial.begin(115200);
    Dabble.begin(9600);
    eracount = 0;
    elacount = 0;
    pinMode(eraPin, INPUT_PULLUP);
    pinMode(elaPin, INPUT_PULLUP);
    time = millis();
    attachInterrupt(digitalPinToInterrupt(elaPin),[&elacount]{elacount++;},FALLING); 
}

// class Robot {
// public:
//     PID leftPid;
//     PID rightPid;

//     Motor left(41, 45);
//     Motor right(42, 46);

//     double leftOut;
//     double rightOut;
//     double rightIn;
//     double leftIn;
//     unsigned long prevTime;

//     Robot()
//     {
//         prevTime = millis();
//     }
//     void move(float mps, float meters);
//     void move(float mps);
//     void rotate(float dps, float degrees);
//     void rotate(float dps);
//     void tick()
//     {

//     }
// };

// PID myPID(&Input, &Output, &Setpoint,2,5,1, DIRECT);

PidMotor<20> left(41, 45);
PidMotor<3> right(42, 46);

void loop() {

    left.setSpeed(500);
    right.setSpeed(500);
    left.tick();
    right.tick();
    return;
//     Dabble.processInput();
//     if (millis() - time > 200) {
//         time += 200;
//     }
//     if (GamePad.isStartPressed())
//     {
//         eracount = 0;
//         elacount = 0;
//     }
//     if (countsToInches(eracount, eraCPR) >= 120) {
//         right.setSpeed(0);
//         left.setSpeed(0);
//         return;
//     }
//     if (countsToInches(elaCPR, elaCPR) >= 120) {
//         right.setSpeed(0);
//         left.setSpeed(0);
//         return;
//     }
//     if (GamePad.isUpPressed())
//     {
//         right.setSpeed(.25);
//         left.setSpeed(.25);
//     }

//     else if (GamePad.isDownPressed())
//     {
//         right.setSpeed(-.25);
//         left.setSpeed(-.25);
//     }

//     else if (GamePad.isLeftPressed())
//     {
//         right.setSpeed(.25);
//         left.setSpeed(-.25);
//     }

//     else if (GamePad.isRightPressed())
//     {
//         right.setSpeed(-.25);
//         left.setSpeed(.25);
//     } else {
//         right.setSpeed(0);
//         left.setSpeed(0);
//     }

//     // For future reference:
// //  {
//         if (GamePad.isSquarePressed())
//         {
//         }

//         if (GamePad.isCirclePressed())
//         {
//         }

//         if (GamePad.isCrossPressed())
//         {
//         }

//         if (GamePad.isTrianglePressed())
//         {
//         }


//         if (GamePad.isSelectPressed())
//         {
//         }
// //  }
}

