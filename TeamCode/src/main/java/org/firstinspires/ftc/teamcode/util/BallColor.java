package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.Prism.Color;

public enum BallColor {
    Red, Orange, Yellow, Green, Blue, Purple, None, Any;
    public static Color getColor(BallColor ballColor){
        switch (ballColor){
            case Any:
                return new Color(52, 153, 255);
            case None:
                return new Color(0, 0, 0);
            case Red:
                return new Color(182, 0, 0);
            case Orange:
                return new Color(200, 59, 0);
            case Yellow:
                return new Color(182, 182, 0);
            case Green:
                return new Color(0, 182, 0);
            case Blue:
                return new Color(0, 0, 182);
            case Purple:
                return new Color(148, 8, 251);
            default:
                return new Color(1, 0, 0);

        }
    }
}
