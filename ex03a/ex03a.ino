// Caden Churchman
// MNE 615
// Exercise 3

#include "math.h"
#include <Servo.h>
#include "Arduino.h"

// Pin config
const uint8_t ONE_A_PIN = 41;
const uint8_t TWO_A_PIN = 42;
const uint8_t EN_PIN = 45;
const uint8_t BUTTON_PIN = 32;

void setup()
{
    Serial.begin(115200);
}

// Simple button class to make a non-interrupt button easy to use
class Button {
    uint8_t pin_;
    unsigned long lastTime_;
    uint16_t debounce_;
    bool released_;
public:
    Button(uint8_t pin, uint16_t debounce = 250)
        : pin_(pin), debounce_(debounce), lastTime_(0), released_(true)
    {
        pinMode(pin_, INPUT);
    }

    // True if button is validly pressed
    bool check()
    {
        int result = digitalRead(pin_);
        if (released_ && result && millis() - lastTime_ > debounce_) {
            lastTime_ = millis();
            released_ = false;
            return true;
        }
        released_ = result == LOW;
        return false;
    }
};

// Controls a motor with two direction pins
class Motor {
    uint8_t p1_;
    uint8_t p2_;
    uint8_t en_;
public:
    Motor(uint8_t p1, uint8_t p2, uint8_t en)
        : p1_(p1), p2_(p2), en_(en)
    {
        pinMode(p1_, OUTPUT);
        pinMode(p2_, OUTPUT);
        pinMode(en_, OUTPUT);
    }

    // set dir
    void setForward()
    {
        digitalWrite(p1_, HIGH);
        digitalWrite(p2_, LOW);
    }

    // set dir
    void setBackward()
    {
        digitalWrite(p1_, LOW);
        digitalWrite(p2_, HIGH);
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
        
        analogWrite(en_, set);
    }
};

uint32_t time = millis();
uint32_t interval = 250;
double r = 0;

Motor m(ONE_A_PIN, TWO_A_PIN, EN_PIN);

Button b(BUTTON_PIN);

void loop()
{
    // Check button, and delay 250ms
    if (b.check() && millis() - time > interval) {
        // Set motor speeds according to assignment
        delay(1000);
        m.setSpeed(1);
        delay(1500);
        m.setSpeed(-1);
        delay(1500);
        m.setSpeed(0.5);
        delay(2000);
        m.setSpeed(-0.5);
        delay(2000);
        m.setSpeed(0);
        time += interval;
    }
}
