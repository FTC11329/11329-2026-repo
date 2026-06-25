package org.firstinspires.ftc.teamcode.util;

public enum BallColor {
    Red, Orange, Yellow, Green, Blue, Purple, None, Any;

    public int toInt() {
        int col = -1;
        if (this == Red){col = 1;}
        else if (this == Orange){col = 2;}
        else if (this == Yellow){col = 3;}
        else if (this == Green){col = 4;}
        else if (this == Blue){col = 5;}
        else if (this == Purple){col = 6;}
        return col;
    }

    static public BallColor toBall(int col) {
        BallColor ball = Any;
        if (col == 1){ball = Red;}
        else if (col == 2){ball = Orange;}
        else if (col == 3){ball = Yellow;}
        else if (col == 4){ball = Green;}
        else if (col == 5){ball = Blue;}
        else if (col == 6){ball = Purple;}
        return ball;
    }

    static public int numberColors() {
        return 6;
    }
}
