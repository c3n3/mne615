
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
