
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
