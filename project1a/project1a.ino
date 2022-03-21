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
#include "src/CustomPID.hpp"

#define logn(arg) Serial.println(arg)
#define logv(arg) Serial.print(#arg " = "); Serial.println(arg)
#define log(arg) Serial.print(arg)
 
// void serPrintf(const char* fmt, ...)
// {
//     va_list args;
//     va_start(args, fmt);
 
//     while (*fmt != '\0') {
//         if (*fmt == '%' && *(fmt+1) == 'd') {
//             int i = va_arg(args, int);
//             Serial.print(i);
//         } else if (*fmt == '%' && *(fmt+1) == 'c') {
//             // note automatic conversion to integral type
//             int c = va_arg(args, int);
//             Serial.print(static_cast<char>(c));
//         } else if (*fmt == '%' && *(fmt+1) == 'f') {
//             double d = va_arg(args, double);
//             Serial.print(d);
//         } else {
//             Serial.print(*fmt);
//         }
//         ++fmt;
//     }
//     va_end(args);
// }

// Controls a motor with two direction pins
template<int ENC_PIN>
class PidMotor {
public:
    uint8_t dirPin_;
    uint8_t pwmPin_;
    CustomPID pid_;
    const uint8_t updateInterval = 100;
    unsigned long prevTime_;
    uint32_t prevCount_;
    static uint32_t count_;
    double pidIn_;
    double pidOut_;
    double pidValue_;
    double prevSet_;
    double realOut_;
    uint32_t killCount_;
    bool killOnCount_;
    bool delayPid_;
    unsigned long pidStartTime_;
    static constexpr uint16_t DELAY_PID_TIME = 150;
public:
    PidMotor(uint8_t dirPin, uint8_t pwmPin)
        : dirPin_(dirPin), pwmPin_(pwmPin), pid_(&pidIn_, &pidOut_, &pidValue_, 0.013725, 0.08235, 0, DIRECT)
    {
        killOnCount_ = false;
        delayPid_ = false;
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

    double getSpeed()
    {
        return pidValue_;
    }

    double batteryVoltage()
    {
        return ((double)analogRead(A9) / 1024.0) * 10;
    }

    double bestFitPwm(double cps)
    {
        return (((cps) / 31.5162) * (4.8*9 / batteryVoltage()));
    }

    double bestFitCps(double pwm)
    {
        return ((pwm / (4.8*9 / batteryVoltage())) * 31.5162);
    }

    // set speed based on float
    void setSpeed(double cps)
    {
        killOnCount_ = false;
        double set = ((cps < 0 ? -cps : cps)) ;

        if (cps < 0) {
            setBackward();
        } else {
            setForward();
        }
        // Use best fit as starting value
        if (cps != prevSet_) {
            // cps = 31.5162 * x + 30.3788
            double pwmStart = bestFitPwm(set);
            analogWrite(pwmPin_, pwmStart);
            realOut_ = pwmStart;
            pidStartTime_ = millis();
            delayPid_ = true;
        }
        // log("Writing motor speed to "); logn(start);
        pidValue_ = set;
        prevSet_ = cps;
    }

    void move(double cps, double counts)
    {
        killCount_ = count_ + counts;
        setSpeed(cps);
        killOnCount_ = true;
    }

    void tick()
    {
        const double cps = ((double)(count_ - prevCount_) / (double)(millis() - prevTime_)) * 1000.0;
        // log("Cps = "); log(cps); log(" PWM = "); log(realOut_); log(" Volage = "); logn(batteryVoltage());
        if (killOnCount_ && count_ >= killCount_) {
            setSpeed(0);
            return;
        }

        if (millis() - prevTime_ > updateInterval) {
            if (!delayPid_) {
                pidIn_ = cps;
                pid_.Compute(millis());
                realOut_ = pidOut_;
                analogWrite(pwmPin_, pidOut_);
            }
            prevTime_ += updateInterval;
            prevCount_ = count_;
        }

        // Used for initial move
        if (delayPid_) {
            for (int i = 0; i < 100; i++) {
                pidIn_ = bestFitCps(pidOut_);
                pid_.Compute(i * 100);
                // log("Delaying pid, fake cps = "); log(pidIn_); log(" PWM = "); log(pidOut_); log(" MIllis "); logn(millis());
            }
            pid_.SetLastTime(millis());
            // delayPid_ = false;
            if (millis() - pidStartTime_ > DELAY_PID_TIME) {
                delayPid_ = false;
            }
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
    float prevDps_;
    bool coutingSteps_;
    Direction direction_;
    unsigned long time_;

    // Simple calculation function
    uint16_t rpmToStepsPerSecond(float rpm)
    {
        float rps = abs(rpm / 60);
        return (rps * stepsPerRev_);
    } 

    float dpsToStepsPerSecond(float dps)
    {
        const float degreesPerStep = 360.0 / (float)stepsPerRev_;
        return abs(dps) / degreesPerStep;
    }
public:


    // Construct a stepper motor
    StepperMotor(uint8_t dirPin, uint8_t stepPin, uint16_t stepsPerRev = 20)
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
        if (dps == prevDps_) {
            return;
        }
        if (dps == 0) {
            prevDps_ = 0;
            msInterval_ = 0;
            return;
        }
        direction_ = dps < 0 ? LOW : HIGH;
        // 1000ms / 1s *  (second / step)
        msInterval_ = 1000.0 / dpsToStepsPerSecond(dps);
        time_ = millis();
        prevDps_ = dps;
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
        prevDps_ = 0;
        toStep_ = 0;
    }

    void move(float dps, float degrees)
    {
        const float stepsPerDegree = stepsPerRev_ / 360.0;
        const float sign = degrees < 0 ? -1 : 1;
        setSpeedDps(dps);
        toStep_ = sign * degrees * stepsPerDegree;
        coutingSteps_ = true;
        logv(dps);
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

    bool isMoving()
    {
        return msInterval_ != 0;
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

    bool isMoving()
    {
        return screw.isMoving();
    }
};


class Gripper {
public:
    static constexpr uint8_t SERVO_CONTROL_PIN = 46;
    Servo servo;
    float targetUs_;
    float currentUs_;
    static constexpr uint16_t msPerInterval_ = 15;
    float usInterval_;
    float sign_;
    unsigned long time_;
public:
    Gripper()
    {
        pinMode(46, OUTPUT);
        targetUs_ = 850;
        currentUs_ = 850;
        sign_ = 1;
    }

    void init()
    {
        servo.attach(46);
        servo.writeMicroseconds(800);
    }

    void move(int us)
    {
        currentUs_ = us;
        targetUs_ = us;
        servo.writeMicroseconds(us);
    }

    void stop()
    {
        targetUs_ = currentUs_;
    }

    void move(int us, float usps)
    {
        targetUs_ = us;
        sign_ = targetUs_ - currentUs_ < 0 ? -1 : 1;
        usInterval_ = sign_ * (usps / 1000.0) * (float)msPerInterval_;
        // time_ = millis();
    }

    bool isMoving()
    {
        return (sign_ > 0 && currentUs_ < targetUs_) || (sign_ < 0 && currentUs_ > targetUs_);
    }

    void tick()
    {
        if ((sign_ > 0 && currentUs_ < targetUs_) || (sign_ < 0 && currentUs_ > targetUs_)) {
            if (millis() - time_ > msPerInterval_) {
                currentUs_ += usInterval_;
                servo.writeMicroseconds(currentUs_);
                time_ = millis();
            }
        }
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

#include <stdlib.h>

class SDCommandReader {
    File f;
    static constexpr uint16_t chipSelect = 53;
public:
    enum CommandType {
        FORWARD, // DISTANCE IN INCHES
        REVERSE, // DISTANCE IN INCHES
        RIGHT, // ANGLE IN DEGREES
        LEFT, // ANGLE IN DEGREES
        UP, // LEAD SCREW UP DISTANCE IN INCHES
        DOWN, // LEAD SCREW UP DISTANCE IN INCHES
        SERVO, // PULSE HEIGHT IN MICROSECONDS
        PAUSE, // MILLISECONDS
        NONE, // Not a command
        END, // No more commands
    };

    struct Command {
        CommandType t;
        double value;
    };

    CommandType stringToCommand(const char* comp)
    {
        #define CHECK(val) if (String(#val) == comp) return val;
        CHECK(FORWARD);
        CHECK(REVERSE);
        CHECK(RIGHT);
        CHECK(LEFT);
        CHECK(UP);
        CHECK(DOWN);
        CHECK(SERVO);
        CHECK(PAUSE);
        return NONE;
        #undef CHECK
    }

    Command next()
    {
        char buffer[100];
        uint16_t i = 0;
        if (f && f.available())
        {
            do {
                buffer[i++] = (char)f.read();
            } while (f.available() && buffer[i - 1] != '\n');
            buffer[i] = '\0';
            char command[40];
            char value[40];
            for (i = 0; buffer[i] != ','; i++) {
                command[i] = buffer[i];
            }
            command[i] = '\0';
            i++; // Because the comma
            i++; // Because there is a space after the comma
            int vi = 0;
            for (; buffer[i] != '\0'; i++) {
                value[vi++] = buffer[i];
            }
            value[vi] = '\0';
            return {
                .t = stringToCommand(command),
                .value = atof(value)
            };
        }
        return {
            .t = SDCommandReader::NONE,
            .value = 0
        };

    }

    void init()
    {
        Serial.print("Initializing SD card...");

        if (!SD.begin(chipSelect))
        {
            Serial.println("initialization failed. Things to check:");
            Serial.println("1. is a card inserted?");
            Serial.println("2. is your wiring correct?");
            Serial.println("3. did you change the chipSelect pin to match your shield or module?");
            Serial.println("Note: press reset or reopen this Serial Monitor after fixing your issue!");
            while (true)
                ;
        }
    }

    void restart()
    {
        f.close();
        f = SD.open("datafile.txt");
    }
};

Dab dabble;

class Robot {
public:
    enum State {
        Rotate,
        Move,
        Other
    };
    typedef void(*RobotAlgorithm)(Robot*);
    PidMotor<20> left_;
    PidMotor<3> right_;
    unsigned long timeToKill_;
    bool setToKill_;
    bool isMoving_;
    State state_;
    RobotAlgorithm algo_;

    float iToC(float i)
    {
        // inches per rotation (circumference) / counts per rotation
        const float countsPerInch =  360.0 / 8.4823;
        return i * countsPerInch;
    }    

    float dToC(float dps)
    {
        // Calculated based on axle distance and wheel circumference
        // (17.56 / 360.0) * (360.0 / 8.4823)
        // (axle circumference / degrees per rotation) * (counts per rotation / wheel circumference)
        return dps * 2.07;
    }
public:

    Lift lift;
    Gripper gripper;
    SDCommandReader commander;

    void init()
    {
        gripper.init();
    }

    Robot(RobotAlgorithm algo)
        : left_(41, 5), right_(42, 2),
        isMoving_(false),
        timeToKill_(0),
        setToKill_(false),
        algo_(algo)
    {
        commander.init();
        state_ = Other;
    }

    void move(float ips, float inches)
    {
        float cps = iToC(ips);
        float counts = iToC(inches);
        right_.move(cps, counts);
        left_.move(cps, counts);
        isMoving_ = true;
        state_ = Move;
    }

    void move(float ips)
    {
        float cps = iToC(ips);
        right_.setSpeed(cps);
        left_.setSpeed(cps);
        isMoving_ = true;
    }

    void rotate(float dps, float degrees)
    {
        float cps = dToC(dps);
        float counts = dToC(degrees);
        right_.move(-cps, counts);
        left_.move(cps, counts);
        isMoving_ = true;
        state_ = Rotate;
    }

    void rotate(float dps)
    {
        float speed = dToC(dps);
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
        right_.tick();
        left_.tick();
        lift.tick();
        gripper.tick();
        switch (state_)
        {
        case Move:
            if (left_.getSpeed() == 0) {
                right_.setSpeed(0);
                isMoving_ = false;
                state_ = Other;
            } else if (right_.getSpeed() == 0) {
                left_.setSpeed(0);
                isMoving_ = false;
                state_ = Other;
            }
        case Rotate:
            if (left_.getSpeed() == 0 && right_.getSpeed() == 0) {
                isMoving_ = false;
            }
            break;
        default: break;
        }
        if (setToKill_ && (millis() > timeToKill_)) {
            stop();
            setToKill_ = false;
        }
        algo_(this);
    }
};


enum RobotState {
    Manual = 0,
    SdCard = 1,
};

int manual(Robot* robot)
{
    auto c = dabble.input();
    switch (c) {
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
        case Dab::Triangle:
            robot->lift.move(0.1);
            break;
        case Dab::Circle:
            robot->gripper.move(800, 1000);
            break;
        case Dab::X:
            robot->lift.move(-0.1);
            break;
        case Dab::Square:
            robot->gripper.move(2400, 1000);
            break;
        case Dab::Start:
            break;
        case Dab::Select:
            robot->commander.restart();
            return SdCard;
    }
    if (c != Dab::Up && c != Dab::Down && c != Dab::Right && c != Dab::Left) {
        robot->stop();
    }
    if (c != Dab::Triangle && c != Dab::X) {
        robot->lift.move(0);
    }
    if (c != Dab::Square && c != Dab::Circle) {
        robot->gripper.stop();
    }
    return Manual;
}

int sdCard(Robot* robot)
{
    auto in = dabble.input();
    if (in == Dab::Start) {
        return Manual;
    }
    if (robot->isMoving() || robot->lift.isMoving() || robot->gripper.isMoving()) {
        return SdCard;
    }
    auto c = robot->commander.next();
    switch (c.t)
    {
        case SDCommandReader::FORWARD: log("Running "); log(c.value); log(" "); logn(__STRINGIFY(FORWARD));
            robot->move(10, c.value);
            break;
        case SDCommandReader::REVERSE: log("Running "); log(c.value); log(" "); logn(__STRINGIFY(REVERSE));
            robot->move(-10, c.value);
            break;
        case SDCommandReader::RIGHT: log("Running "); log(c.value); log(" "); logn(__STRINGIFY(RIGHT));
            robot->rotate(180, c.value);
            break;
        case SDCommandReader::LEFT: log("Running "); log(c.value); log(" "); logn(__STRINGIFY(LEFT));
            robot->rotate(-180, c.value);
            break;
        case SDCommandReader::UP: log("Running "); log(c.value); log(" "); logn(__STRINGIFY(UP));
            robot->lift.move(c.value, 0.1);
            break;
        case SDCommandReader::DOWN: log("Running "); log(c.value); log(" "); logn(__STRINGIFY(DOWN));
            robot->lift.move(c.value, -0.1);
            break;
        case SDCommandReader::SERVO: log("Running "); log(c.value); log(" "); logn(__STRINGIFY(SERVO));
            robot->gripper.move(c.value, 300);
            break;
        case SDCommandReader::PAUSE: log("Running "); log(c.value); log(" "); logn(__STRINGIFY(PAUSE));
            delay(c.value);
            break;
    }
    return SdCard;
}

void algorithm(Robot* robot)
{
    static RobotState state = Manual;
    dabble.tick();

    switch (state)
    {
    case Manual:
        state = manual(robot);
        break;
    case SdCard:
        state = sdCard(robot);;
        break;
    default:
        state = Manual;
        break;
    }
}

Robot r(algorithm);

void setup()
{
    Serial.begin(115200);
    delay(100);
    r.init();
    pinMode(A9, INPUT);
    Serial.println("Setup");
    dabble.init();
}


int pos = 0;


void loop() {
    r.tick();
    // unsigned long startTime = millis();
    // r.left_.setSpeed(500);
    // r.right_.setSpeed(500);
    // // r.move(2);
    // while (millis() < 5000 + startTime) {
    //     // r.left_.tick();
    //     // r.right_.tick();
    //     r.tick();
    // }
    // r.move(5);
    //  startTime = millis();
    // // r.left_.setSpeed(200);
    // // r.right_.setSpeed(200);
    // while (millis() < 4000 + startTime) {
    //     // r.left_.tick();
    //     // r.right_.tick();
    //     r.tick();
    // }
    // // r.move(0);
    //  startTime = millis();
    // r.left_.setSpeed(0);
    // r.right_.setSpeed(0);
    // while (millis() < 6000 + startTime) {
    //     // r.left_.tick();
    //     // r.right_.tick();
    //     r.tick();
    // }
    // // dabble.tick();
    // // r.move(6);
    // // r.tick();

}

