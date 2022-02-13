// Caden Churchman
// MNE 615
// Exercise 2

#include "math.h"
#include <Servo.h>
#include "Arduino.h"

Servo servo;

const uint8_t BUTTON_PIN = 38;
const uint8_t LED_PIN = 40;
const uint8_t SERVO_CONTROL_PIN = 34;
const uint8_t SERVO_FEEDBACK_PIN = A3;

uint32_t micro = 800;

void setup()
{
    Serial.begin(115200);
    servo.attach(SERVO_CONTROL_PIN);
    pinMode(BUTTON_PIN, INPUT);
    pinMode(SERVO_FEEDBACK_PIN, INPUT);
    pinMode(SERVO_CONTROL_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);

    // Init in low state
    servo.writeMicroseconds(800);
}

uint32_t time = millis();
uint32_t interval = 200;
double r = 0;

enum State {
    Nothing,
    Down,
    Up
};

State state = Nothing;
State oldState = Up;

// Loop function
void loop()
{
    // Write cur microseconds
    servo.writeMicroseconds(micro);
    bool buttonPressed = digitalRead(BUTTON_PIN);
    if (buttonPressed && state == Nothing) {
        state = oldState; // go back to old state
    }
    if (!buttonPressed && state != Nothing) {
        oldState = state; // Clean states
        state = Nothing;
    }

    // With interval
    if (millis() - time >= interval) {
        int read = analogRead(SERVO_FEEDBACK_PIN);
        double volts = ((double)read / 1023.0) * 5.0;
        digitalWrite(LED_PIN, volts > 1.8); // led if > 1.8V

        // Go up
        if (state == Down) {
            micro -= 50;
            if (micro < 800) {
                micro = 800;
                state = Up;
            }
        }
        // Go down
        if (state == Up) {
            micro += 50;
            if (micro > 2400) {
                micro = 2400;
                state = Down;
            }
        }
        // Increment
        time += interval;
    }
}
