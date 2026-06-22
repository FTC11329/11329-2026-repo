package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.sun.source.tree.Tree;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.Dictionary;
import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

public class ColorFunctions {
    public static BallColor toColor(NormalizedRGBA rgba, double distance) {
        double[] colorList = {rgba.red, rgba.green, rgba.blue, rgba.alpha};

        double noneDistance = colorDistance(colorList, Constants.Color.none, Constants.Color.none2);
        double purpleDistance = colorDistance(colorList, Constants.Color.purple, Constants.Color.purpleFar, Constants.Color.purpleWeird);
        double greenDistance = colorDistance(colorList, Constants.Color.green, Constants.Color.greenFar, Constants.Color.greenWeird, Constants.Color.greenFakeHole);

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

    public static double colorDistance(double[] colorList, double[]... params) {
        double dst = 0;
        for (double[] game: params) {
            double gameDistance  = Math.hypot(Math.hypot(colorList[0] - game[0] , colorList[1] - game[1]) , colorList[2] - game[2] );
            dst = ((dst == 0) ? gameDistance : Math.min(gameDistance, dst));
        }
        return  dst;
    }

    public static BallColor toGemColor(NormalizedRGBA rgba, double distance) {
        double[] colorList = {rgba.red, rgba.green, rgba.blue, rgba.alpha};

        Map<BallColor, Double> distances= Map.of(
            BallColor.None, colorDistance(colorList, Constants.Color.none, Constants.Color.none2),
            BallColor.Red, colorDistance(colorList, Constants.Color.red),
            BallColor.Orange, colorDistance(colorList, Constants.Color.orange),
            BallColor.Yellow, colorDistance(colorList, Constants.Color.yellow),
            BallColor.Green, colorDistance(colorList, Constants.Color.green),
            BallColor.Blue, colorDistance(colorList, Constants.Color.blue),
            BallColor.Purple, colorDistance(colorList, Constants.Color.purple)
        );

        return distances.entrySet()
                        .stream()
                        .min(Map.Entry.comparingByValue())
                        .orElseThrow()
                        .getKey(); // Sorts the map by value, which is the distances, then returns the Key (Gem) with minimum distance
    }
}
