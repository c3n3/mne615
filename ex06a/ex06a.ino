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
class Motor {
    uint8_t dirPin_;
    uint8_t pwmPin_;
public:
    Motor(uint8_t dirPin, uint8_t pwmPin)
        : dirPin_(dirPin), pwmPin_(pwmPin)
    {
        pinMode(dirPin_, OUTPUT);
        pinMode(pwmPin_, OUTPUT);
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
    void setSpeed(float speed)
    {
        uint8_t set = ((speed < 0 ? -speed : speed) * 255.0);
        if (speed < 0) {
            setBackward();
        } else {
            setForward();
        }
        
        analogWrite(pwmPin_, set);
    }
};


Motor left(41, 45);
Motor right(42, 46);

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
    attachInterrupt(digitalPinToInterrupt(eraPin),[&eracount]{eracount++;},CHANGE); 
}


void loop() {
    Dabble.processInput();
    if (millis() - time > 200) {
        _printf("ela=%d", elacount);
        _printf(" era=%d\n", eracount);
        time += 200;
    }
    if (GamePad.isStartPressed())
    {
        eracount = 0;
        elacount = 0;
    }
    if (countsToInches(eracount, eraCPR) >= 120) {
        right.setSpeed(0);
        left.setSpeed(0);
        return;
    }
    if (countsToInches(elaCPR, elaCPR) >= 120) {
        right.setSpeed(0);
        left.setSpeed(0);
        return;
    }
    if (GamePad.isUpPressed())
    {
        right.setSpeed(.25);
        left.setSpeed(.25);
    }

    else if (GamePad.isDownPressed())
    {
        right.setSpeed(-.25);
        left.setSpeed(-.25);
    }

    else if (GamePad.isLeftPressed())
    {
        right.setSpeed(.25);
        left.setSpeed(-.25);
    }

    else if (GamePad.isRightPressed())
    {
        right.setSpeed(-.25);
        left.setSpeed(.25);
    } else {
        right.setSpeed(0);
        left.setSpeed(0);
    }

    // For future reference:
//  {
        if (GamePad.isSquarePressed())
        {
        }

        if (GamePad.isCirclePressed())
        {
        }

        if (GamePad.isCrossPressed())
        {
        }

        if (GamePad.isTrianglePressed())
        {
        }


        if (GamePad.isSelectPressed())
        {
        }
//  }
}

