// Caden Churchman
// MNE 615
// Exercise 4

#include "math.h"
#include "Arduino.h"

const uint8_t POT_PIN = A2;
const uint8_t DIR_PIN = 31;
const uint8_t STEP_PIN = 33;

// Class to control stepper
class StepperMotor {
public:
    // Direction enum to clarify direction
    enum Direction {
        Forward = HIGH,
        Backward = LOW
    };
private:
    const uint8_t dirPin_;
    const uint8_t stepPin_;
    const uint16_t stepsPerRev_;
    uint16_t msInterval_;
    Direction direction_;
    unsigned long time_;

    // Simple calculation function
    uint16_t rpmToStepsPerSecond(float rpm)
    {
        float rps = abs(rpm / 60);
        return (rps * stepsPerRev_);
    } 

    uint16_t dpsToStepsPerSecond(float dps)
    {
        const float degreesPerStep = 360 / stepsPerRev_;
        return abs(dps / degreesPerStep);
    } 
public:


    // Construct a stepper motor
    StepperMotor(uint8_t dirPin, uint8_t stepPin, uint16_t stepsPerRev = 200)
        : dirPin_(dirPin), stepPin_(stepPin), stepsPerRev_(stepsPerRev), msInterval_(0), direction_(LOW)
    {
        pinMode(dirPin_, OUTPUT);
        pinMode(stepPin_, OUTPUT);
        digitalWrite(dirPin_, LOW);
        digitalWrite(stepPin_, LOW);
    }


    // In revolutions per minute
    void setSpeedRpm(float rpm)
    {
        if (rpm == 0) {
            msInterval_ = 0;
            return;
        }
        direction_ =  rpm < 0 ? LOW : HIGH;
        // 1000ms / 1s *  (second / step)
        msInterval_ = 1000 / rpmToStepsPerSecond(rpm);
    }

    // In degrees per second
    void setSpeedDps(float dps)
    {
        if (dps == 0) {
            msInterval_ = 0;
            return;
        }
        direction_ = dps < 0 ? LOW : HIGH;
        // 1000ms / 1s *  (second / step)
        msInterval_ = 1000 / dpsToStepsPerSecond(dps);
    }

    // In steps per second
    void setSpeedSps(float sps)
    {
        if (sps == 0) {
            msInterval_ = 0;
            return;
        }
        // 1000ms / 1s *  (second / step)
        msInterval_ = 1000 / sps;
    }

    // In millisecond interval
    void setSpeedMsi(float msi)
    {
        msInterval_ = abs(msi);
        direction_ = msi < 0 ? Backward : Forward;
    }

    // Step the motor
    void step()
    {
        digitalWrite(dirPin_, direction_);
        digitalWrite(stepPin_,HIGH); 
        delayMicroseconds(500); 
        digitalWrite(stepPin_,LOW); 
        delayMicroseconds(500); 
    }

    // Step the motor for a direction
    void step(Direction dir)
    {
        msInterval_ = 0;
        direction_ = dir;
        step();
    }

    // Must be called continuously
    void run()
    {
        if (msInterval_ == 0) return;
        if (millis() - time_ > msInterval_) {
            step();
            time_ += msInterval_;
        }
    }
};

StepperMotor m(DIR_PIN, STEP_PIN);

void setup()
{
    Serial.begin(115200);
    pinMode(POT_PIN, INPUT);
    m.setSpeedMsi(200);
}

uint32_t time = millis();
uint32_t interval = 250;
double r = 0;


void loop()
{
    if (millis() - time >= interval) {
        // Read in to calculate voltage and resistance
        double v2 = ((double)analogRead(POT_PIN) / 1023.0) * 5.0;
        r = (5 / (v2)) * 10000.0 - 10000.0 ;

        // Print results
        Serial.print("R="); Serial.print(r, 2); Serial.println(" Ohm");
        if ((r < 5200 && r >= 4800) || (r < 4000 || r > 6000)) {
            m.setSpeedMsi(0);
        } else if (r < 5400 && r >= 5200) {
            m.setSpeedMsi(2);
        } else if (r < 6000 && r >= 5400) {
            m.setSpeedMsi(10);
        } else if (r < 4800 && r >= 4600) {
            m.setSpeedMsi(-2);
        } else if (r < 4600 && r >= 4000) {
            m.setSpeedMsi(-10);
        }
        time += interval;
    }
    m.run();
}
