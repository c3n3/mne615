/*
   Gamepad module provides three different mode namely Digital, JoyStick and Accerleometer.
   You can reduce the size of library compiled by enabling only those modules that you want to
   use. For this first define CUSTOM_SETTINGS followed by defining INCLUDE_modulename.
   Explore more on: https://thestempedia.com/docs/dabble/game-pad-module/
*/
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <Dabble.h>


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
        uint8_t set = (abs(speed) * 255);
        Serial.print("Speed: ");
        Serial.println(set);
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

void setup() {
    Serial.begin(115200);
    Dabble.begin(9600);
}

void loop() {
    Dabble.processInput();
    if (GamePad.isUpPressed())
    {
        right.setSpeed(50);
        left.setSpeed(50);
    }

    else if (GamePad.isDownPressed())
    {
        right.setSpeed(-50);
        left.setSpeed(-50);
    }

    else if (GamePad.isLeftPressed())
    {
        right.setSpeed(50);
        left.setSpeed(-50);
    }

    else if (GamePad.isRightPressed())
    {
        right.setSpeed(-50);
        left.setSpeed(50);
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

        if (GamePad.isStartPressed())
        {
        }

        if (GamePad.isSelectPressed())
        {
        }
//  }
}

