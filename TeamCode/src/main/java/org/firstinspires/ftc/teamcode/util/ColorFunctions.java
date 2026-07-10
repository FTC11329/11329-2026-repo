package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Constants;

public class ColorFunctions {
    public static BallColor toColor(double[] list, double distance) {
        NormalizedRGBA rgba = new NormalizedRGBA();
        rgba.red   = (float)( list[0] / list[2]);
        rgba.green = (float)( list[1] / list[2]);
        rgba.blue  = 1f;
        rgba.alpha = (float)( list[3]);
        return toColor(rgba, distance);
    }
    public static BallColor toColor(NormalizedRGBA rgba, double distance) {
        double[] colorList = {rgba.red, rgba.green, rgba.blue, rgba.alpha};

        double redDistance =    getDistFromColor(colorList, Constants.Color.red);
        double orangeDistance = getDistFromColor(colorList, Constants.Color.orange);
        double yellowDistance = getDistFromColor(colorList, Constants.Color.yellow);
        double greenDistance  = getDistFromColor(colorList, Constants.Color.green);
        double blueDistance  =  getDistFromColor(colorList, Constants.Color.blue);
        double purpleDistance = getDistFromColor(colorList, Constants.Color.purple);

        double closestDistance = Math.min(Math.min(Math.min(redDistance, orangeDistance), Math.min(yellowDistance, greenDistance)), Math.min(blueDistance, purpleDistance));

        BallColor viewedColor;

        if (redDistance == closestDistance) {
            viewedColor = BallColor.Red;
        } else if (orangeDistance == closestDistance) {
            viewedColor = BallColor.Orange;
        } else if (yellowDistance == closestDistance) {
            viewedColor = BallColor.Yellow;
        } else if (greenDistance == closestDistance) {
            viewedColor = BallColor.Green;
        } else if (blueDistance == closestDistance) {
            viewedColor = BallColor.Blue;
        } else if (purpleDistance == closestDistance) {
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

    public static double getDistFromColor(double[] colorList, double[] testColor) {
        return Math.hypot(Math.hypot(colorList[0] - testColor[0], colorList[1] - testColor[1]), colorList[2] - testColor[2]);

    }
}
