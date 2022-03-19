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
#include <Servo.h>
#include <PID_v1.h>


 
void serPrintf(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
 
    while (*fmt != '\0') {
        if (*fmt == '%' && *(fmt+1) == 'd') {
            int i = va_arg(args, int);
            Serial.print(i);
        } else if (*fmt == '%' && *(fmt+1) == 'c') {
            // note automatic conversion to integral type
            int c = va_arg(args, int);
            Serial.print(static_cast<char>(c));
        } else if (*fmt == '%' && *(fmt+1) == 'f') {
            double d = va_arg(args, double);
            Serial.print(d);
        } else {
            Serial.print(*fmt);
        }
        ++fmt;
    }
    va_end(args);
}

#define _PRINT(arg, ...) Serial.println() 

#define PRINT(arg, ...) Serial 

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
        // Use best fit as starting value
        // cps = 31.5162 * x + 30.3788
        double start = (cps - 30.3788) / 31.5162;
        analogWrite(pwmPin_, start);
        pidValue_ = set;
    }

    void tick()
    {
        if (millis() - prevTime_ > updateInterval) {
            pidIn_ = ((double)(count_ - prevCount_) / (double)(millis() - prevTime_)) * 1000.0;
            pid_.Compute();
            analogWrite(pwmPin_, pidOut_);
            prevTime_ += updateInterval;
            prevCount_ = count_;
        }
    }
};

template<int ENC_PIN>
uint32_t PidMotor<ENC_PIN>::count_ = 0;


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
    uint16_t toStep_;
    bool coutingSteps_;
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
        : dirPin_(dirPin), stepPin_(stepPin), stepsPerRev_(stepsPerRev), msInterval_(0), direction_(LOW), toStep_(0), coutingSteps_(false)
    {
        pinMode(dirPin_, OUTPUT);
        pinMode(stepPin_, OUTPUT);
        digitalWrite(dirPin_, LOW);
        digitalWrite(stepPin_, LOW);
    }

    // In degrees per second
    void setSpeedDps(float dps)
    {
        coutingSteps_ = false;
        if (dps == 0) {
            msInterval_ = 0;
            return;
        }
        direction_ = dps < 0 ? LOW : HIGH;
        // 1000ms / 1s *  (second / step)
        msInterval_ = 1000 / dpsToStepsPerSecond(dps);
        time_ = millis();
    }

    // In steps per second
    void setSpeedSps(float sps)
    {
        coutingSteps_ = false;
        if (sps == 0) {
            msInterval_ = 0;
            return;
        }
        direction_ = sps < 0 ? LOW : HIGH;

        // 1000ms / 1s *  (second / step)
        msInterval_ = 1000 / sps;
        time_ = millis();
    }

    void stop()
    {
        msInterval_ = 0;
        coutingSteps_ = false;
        toStep_ = 0;
    }

    void move(float dps, float degrees)
    {
        const float stepsPerDegree = stepsPerRev_ / 360.0;
        const float sign = degrees < 0 ? -1 : 1;
        toStep_ = sign * degrees * stepsPerDegree;
        coutingSteps_ = true;
        setSpeedDps(sign * dps);
    }

    // Step the motor
    void step()
    {
        digitalWrite(dirPin_, direction_);
        digitalWrite(stepPin_,HIGH); 
        delayMicroseconds(100); 
        digitalWrite(stepPin_,LOW); 
        delayMicroseconds(100); 
    }

    // Step the motor for a direction
    void step(Direction dir)
    {
        msInterval_ = 0;
        direction_ = dir;
        step();
    }

    // Must be called continuously
    void tick()
    {
        if (msInterval_ == 0) return;
        if (millis() - time_ > msInterval_) {
            step();
            if (toStep_-- == 0) {
                stop();
            }
            time_ += msInterval_;
        }
    }
};

class Lift {
    static const uint8_t DIR_PIN = 31;
    static const uint8_t STEP_PIN = 33;
    StepperMotor screw;
public:
    Lift() : screw(DIR_PIN, STEP_PIN)
    {}

    void move(float inches, float ips)
    {
        const float dpi = 18305.0;
        float degrees = dpi * inches;
        float dps = ips * dpi;
        screw.move(dps, degrees);
    }

    void move(float ips)
    {
        const float dpi = 18305.0;
        float dps = ips * dpi;
        screw.setSpeedDps(dps);
    }

    void tick()
    {
        screw.tick();
    }
};


class Gripper {
    static constexpr uint8_t SERVO_CONTROL_PIN = 44;
    Servo servo;
    float targetUs_;
    float currentUs_;
    static constexpr uint16_t msPerInterval_ = 100;
    float usInterval_;
    float sign_;
    unsigned long time_;
public:
    Gripper()
    {
        pinMode(SERVO_CONTROL_PIN, OUTPUT);
        servo.attach(SERVO_CONTROL_PIN);
        targetUs_ = 800;
        currentUs_ = 800;
        servo.writeMicroseconds(targetUs_);
    }

    void move(int us)
    {
        currentUs_ = us;
        targetUs_ = us;
        servo.writeMicroseconds(us);
    }

    void move(int us, float usps)
    {
        targetUs_ = us;
        sign_ = targetUs_ - currentUs_ < 0 ? -1 : 1;
        usInterval_ = sign_ * (usps / 1000.0) * msPerInterval_;
        time_ = millis();
    }

    void tick()
    {
        // Float check for non equality using within 1
        if ((sign_ > 0 && currentUs_ < targetUs_) || (sign_ < 0 && currentUs_ > targetUs_)) {
            if (millis() - time_ > usInterval_) {
                currentUs_ += usInterval_;
                servo.writeMicroseconds(currentUs_);
                time_ += msPerInterval_;
            }
        }
    }
};

class Robot {
    typedef void(*RobotAlgorithm)(Robot*);
    PidMotor<20> left_;
    PidMotor<3> right_;
    unsigned long timeToKill_;
    bool setToKill_;
    bool isMoving_;
    RobotAlgorithm algo_;

    float ipsToCps(float ips)
    {
        // inches per rotation (circumference) / counts per rotation
        const float countsPerInch =  360.0 / 8.4823;
        return ips * countsPerInch;
    }    

    float dpsToCps(float dps)
    {
        // Calculated based on axle distance and wheel circumference
        // (17.56 / 360.0) * (360.0 / 8.4823)
        // (axle circumference / degrees per rotation) * (counts per rotation / wheel circumference)
        return dps * 2.07;
    }
public:
    Lift lift;
    Gripper gripper;

    Robot(RobotAlgorithm algo)
        : left_(41, 45), right_(42, 46),
        isMoving_(false),
        timeToKill_(0),
        setToKill_(false),
        algo_(algo)
    {
    }

    void move(float ips, float inches)
    {
        float cps = ipsToCps(ips);
        right_.setSpeed(cps);
        left_.setSpeed(cps);
        isMoving_ = true;
        setToKill_ = true;
        timeToKill_ = millis() + ((inches / ips) * 1000);
    }

    void move(float ips)
    {
        float cps = ipsToCps(ips);
        right_.setSpeed(cps);
        left_.setSpeed(cps);
        isMoving_ = true;
    }

    void rotate(float dps, float degrees)
    {
        float cps = dpsToCps(dps);
        right_.setSpeed(cps * -1);
        left_.setSpeed(cps);
        isMoving_ = true;
        setToKill_ = true;
        timeToKill_ = millis() + ((degrees / dps) * 1000);
    }

    void rotate(float dps)
    {
        float speed = dpsToCps(dps);
        right_.setSpeed(speed * -1);
        left_.setSpeed(speed);
        isMoving_ = true;
    }

    void stop()
    {
        right_.setSpeed(0);
        left_.setSpeed(0);
        isMoving_ = false;
    }

    bool isMoving()
    {
        return isMoving_;
    }

    void tick()
    {
        lift.tick();
        gripper.tick();
        right_.tick();
        left_.tick();
        if (setToKill_ && (millis() - timeToKill_ >= 0)) {
            stop();
            setToKill_ = false;
        }
        algo_(this);
    }
};

class Dab {
public:
    enum Button {
        Start,
        Up,
        Down,
        Left,
        Right,
        Square,
        Circle,
        Triangle,
        X,
        Select,
        None
    };
    void tick()
    {
        Dabble.processInput();
        if (GamePad.isStartPressed()) latest = Start;
        else if (GamePad.isUpPressed()) latest = Up;
        else if (GamePad.isDownPressed()) latest = Down;
        else if (GamePad.isLeftPressed()) latest = Left;
        else if (GamePad.isRightPressed()) latest = Right;
        else if (GamePad.isSquarePressed()) latest = Square;
        else if (GamePad.isCirclePressed()) latest = Circle;
        else if (GamePad.isCrossPressed()) latest = X;
        else if (GamePad.isTrianglePressed()) latest = Triangle;
        else if (GamePad.isSelectPressed()) latest = Select;
        else latest = None;
    }

    Button input()
    {
        return latest;
    }
    void init()
    {
        Dabble.begin(9600);
    }
    Dab()
    {
        latest = None;
    }
private:
    Button latest;
};


Dab dabble;

void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("Setup");
    dabble.init();
    serPrintf("hello %d %c %f\n", 10, 'A', 90.876);
}

void algorithm(Robot* robot)
{
    Dab* d = &dabble;
    d->tick();
    switch (d->input()) {
        case Dab::Up:
            robot->move(2);
            break;
        case Dab::Down:
            robot->move(-2);
            break;
        case Dab::Left:
            robot->rotate(-90);
            break;
        case Dab::Right:
            robot->rotate(90);
            break;
        default:
            robot->stop();
            switch (d->input())
            {
                case Dab::Triangle:
                    robot->lift.move(0.1);
                    break;
                case Dab::Circle:
                    robot->gripper.move(800, 300);
                    break;
                case Dab::X:
                    robot->lift.move(-0.1);
                    break;
                case Dab::Square:
                    robot->gripper.move(2400, 300);
                    break;
                case Dab::Start:
                    break;
                case Dab::Select:
                    break;
                default: break;
            }
            break;
    }
}

Robot r(algorithm);

void loop() {
    dabble.tick();
    r.tick();
}

