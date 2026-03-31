package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public enum RGBColors {
    Red, Green, Blue;

    public static RGBColors[] sortByMagnitude(NormalizedRGBA rgba) {
        List<RGBColors> colors = new ArrayList<>(Arrays.asList(
                RGBColors.Red,
                RGBColors.Green,
                RGBColors.Blue
        ));

        colors.sort((c1, c2) -> {
            double val1 = getValueForColor(rgba, c1);
            double val2 = getValueForColor(rgba, c2);
            // Sorts descending: largest value first
            return Double.compare(val2, val1);
        });

        return colors.toArray(new RGBColors[0]);
    }

    private static double getValueForColor(NormalizedRGBA rgba, RGBColors type) {
        switch (type) {
            case Red: return rgba.red;
            case Green: return rgba.green;
            case Blue: return rgba.blue;
            default: return 0.0;
        }
    }
}
