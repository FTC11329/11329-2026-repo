package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;

public class Constants {
    public static class Vision {
        public static Pose redTag = new Pose(55.64, -58.34);
        public static Pose blueTag = new Pose(55.64, 58.34);

        public static Pose cameraPos = new Pose(0, 4.47);

        public static double pitch = 80;
    }
    public static class Indexer {
        public static double spindexPower = 0;
    }
    public static class Color {
        public static double[] green = {0, 0, 0, 0};
        public static double[] purple = {0, 0, 0, 0};
        public static double[] none = {0, 0, 0, 0};
        public static double[] none2 = {0, 0, 0, 0};
    }
}
