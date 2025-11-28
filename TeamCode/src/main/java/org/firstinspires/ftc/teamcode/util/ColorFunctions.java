package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Constants;

public class ColorFunctions {
    public static BallColor toColor(NormalizedRGBA rgba, double distance) {
        double[] colorList = {rgba.red, rgba.green, rgba.blue, rgba.alpha};
        double purpleDistance = Math.hypot(Math.hypot(colorList[0] - Constants.Color.purple[0], colorList[1] - Constants.Color.purple[1]), colorList[2] - Constants.Color.purple[2]);
        double greenDistance  = Math.hypot(Math.hypot(colorList[0] - Constants.Color.green[0] , colorList[1] - Constants.Color.green[1]) , colorList[2] - Constants.Color.green[2] );
        double purpleHoleDistance = Math.hypot(Math.hypot(colorList[0] - Constants.Color.purpleHole[0], colorList[1] - Constants.Color.purpleHole[1]), colorList[2] - Constants.Color.purpleHole[2]);
        double greenHoleDistance  = Math.hypot(Math.hypot(colorList[0] - Constants.Color.greenHole[0] , colorList[1] - Constants.Color.greenHole[1]) , colorList[2] - Constants.Color.greenHole[2] );
        double noneDistance   = Math.hypot(Math.hypot(colorList[0] - Constants.Color.none[0]  , colorList[1] - Constants.Color.none[1])  , colorList[2] - Constants.Color.none[2] );
        double none2Distance  = Math.hypot(Math.hypot(colorList[0] - Constants.Color.none2[0] , colorList[1] - Constants.Color.none2[1]) , colorList[2] - Constants.Color.none2[2] );

        noneDistance = Math.min(noneDistance, none2Distance);

        BallColor viewedColor = BallColor.None;

        if (greenDistance <= purpleDistance && greenDistance <= noneDistance) {
            viewedColor = BallColor.Green;
        } else if (purpleDistance <= greenDistance && purpleDistance <= noneDistance) {
            viewedColor = BallColor.Purple;
        }

        if (distance < Constants.Color.fronDst){
            return viewedColor;
        }else if (distance < Constants.Color.backDst){
            return BallColor.None;
        }else {
            if (greenHoleDistance <= purpleHoleDistance && greenHoleDistance <= noneDistance) {
                return BallColor.Green;
            } else if (purpleHoleDistance <= greenHoleDistance && purpleHoleDistance <= noneDistance) {
                return BallColor.Purple;
            }else {
                return BallColor.None;
            }
        }

    }
}
