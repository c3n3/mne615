
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
    static constexpr double axilDiameter = 5.58968645551;
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

    void arcMove(double speed, double radius) 
    {
        double omega = speed * 2 * 3.1415 / (2*radius*3.1415);
        double Vl = omega* (radius + axilDiameter/2);
        double Vr = omega* (radius - axilDiameter/2);
        left_.setSpeed(iToC(Vl));
        right_.setSpeed(iToC(Vr));
        isMoving_ = true;
    }

    void arcMove(double speed, double radius, double distance)
    {
        arcMove(speed, radius);
        timeToKill_ = millis() + distance / abs(speed);
        isMoving_ = true;
    }

    void diffMove(double speed, double offset)
    {
        if (offset > 0) {
            left_.setSpeed(iToC(speed));
            right_.setSpeed(iToC(speed) - iToC(offset));
        } else {
            left_.setSpeed(iToC(speed) + iToC(offset));
            right_.setSpeed(iToC(speed));
        }
        isMoving_ = true;
    }

    void init()
    {
    }

    Robot(RobotAlgorithm algo)
        : left_(41, 5), right_(42, 2),
        isMoving_(false),
        timeToKill_(0),
        setToKill_(false),
        algo_(algo)
    {
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

    bool isNotMoving()
    {
        return !isMoving();
    }

    void tick()
    {
        right_.tick();
        left_.tick();
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
    LineFollow = 2,
    Calibration = 3,
    MazeSolve = 4,
};
