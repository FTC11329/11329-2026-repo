package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Constants;

public class ColorFunctions {
    public static BallColor toColor(NormalizedRGBA rgba, double distance) {
        double[] colorList = {rgba.red, rgba.green, rgba.blue, rgba.alpha};
        double purpleCloseDistance = Math.hypot(Math.hypot(colorList[0] - Constants.Color.purple[0], colorList[1] - Constants.Color.purple[1]), colorList[2] - Constants.Color.purple[2]);
        double purpleFarDistance = Math.hypot(Math.hypot(colorList[0] - Constants.Color.purpleFar[0], colorList[1] - Constants.Color.purpleFar[1]), colorList[2] - Constants.Color.purpleFar[2]);
        double purpleWeirdDistance = Math.hypot(Math.hypot(colorList[0] - Constants.Color.purpleWeird[0], colorList[1] - Constants.Color.purpleWeird[1]), colorList[2] - Constants.Color.purpleWeird[2]);

        double greenCloseDistance  = Math.hypot(Math.hypot(colorList[0] - Constants.Color.green[0] , colorList[1] - Constants.Color.green[1]) , colorList[2] - Constants.Color.green[2] );
        double greenFarDistance  = Math.hypot(Math.hypot(colorList[0] - Constants.Color.greenFar[0] , colorList[1] - Constants.Color.greenFar[1]) , colorList[2] - Constants.Color.greenFar[2] );
        double greenWeirdDistance  = Math.hypot(Math.hypot(colorList[0] - Constants.Color.greenWeird[0] , colorList[1] - Constants.Color.greenWeird[1]) , colorList[2] - Constants.Color.greenWeird[2] );
        double greenFHoleDistance  = Math.hypot(Math.hypot(colorList[0] - Constants.Color.greenFakeHole[0] , colorList[1] - Constants.Color.greenFakeHole[1]) , colorList[2] - Constants.Color.greenFakeHole[2] );

        double none1Distance   = Math.hypot(Math.hypot(colorList[0] - Constants.Color.none[0]  , colorList[1] - Constants.Color.none[1])  , colorList[2] - Constants.Color.none[2] );
        double none2Distance  = Math.hypot(Math.hypot(colorList[0] - Constants.Color.none2[0] , colorList[1] - Constants.Color.none2[1]) , colorList[2] - Constants.Color.none2[2] );


        double noneDistance = Math.min(none1Distance, none2Distance);
        double purpleDistance = Math.min(purpleCloseDistance, Math.min(purpleFarDistance, purpleWeirdDistance));
        double greenDistance = Math.min(Math.min(greenCloseDistance, greenFHoleDistance), Math.min(greenFarDistance, greenWeirdDistance));

        BallColor viewedColor;

        if (greenDistance <= purpleDistance && greenDistance <= noneDistance) {
            viewedColor = BallColor.Green;
        } else if (purpleDistance <= greenDistance && purpleDistance <= noneDistance) {
            viewedColor = BallColor.Purple;
        } else {
            viewedColor = BallColor.None;
        }

        if (distance > Constants.Color.backDst){
            return BallColor.None;
        } else {
            return viewedColor;
        }

    }
}
