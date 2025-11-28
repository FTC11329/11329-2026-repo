package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;

public class Constants {
    // todo replace all placeholders with real constants
    public static double PlacholdereDouble = 0;
    public static boolean PlaceHOLDERboooLEAN = false;

    public static class Indexer {
        public static double spindexPower = 0;
        public static double primingServoUp = 0;
        public static double primingServoDown = 0;
    }
    public static class Intake {
        public static double intakePower = 0;
    }

    public static class Vision {
        public static Pose redTag = new Pose(55.64, -58.34);
        public static Pose blueTag = new Pose(55.64, 58.34);

        public static Pose cameraPos = new Pose(0, 4.47);
        public static Pose3D cameraPosForInfoForGlacierInIN = new Pose3D(new Position(DistanceUnit.INCH, 0, -7, 15.75, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 28.3, 0, 0));

        public static double pitch = 80;
        public static double tagAngle = 80;
    }
    public static class Color {

        public static double[] green = {0.0142, 0.0564, 0.0452, 0.9899};
        public static double[] purple = {0.0149, 0.0177, 0.0371, 0.9726};
        public static double[] none = {0, 0, 0, 0};
        public static double[] none2 = {0, 0, 0, 0};
    }

    public static class ShooterParamaters {
        // --- Fixed geometry ---
        public static double LAUNCHER_HEIGHT_IN = 12.063;   // inches: how high the ball leaves the shooter
        public static double TARGET_HEIGHT_IN   = 29.0;   // inches: height of the target off the ground
        public static double TAG_TO_TARGET_IN   = 19.0;   // inches: distance from AprilTag to middle of the targeted point

        // --- Physical constants ---
        public static double G = 9.81;                   // gravity (m/s^2)
        public static double IN_TO_M = 0.0254;           // inches to meters conversion

        // --- Empirical tuning constants ---
        public static double K_DRAG = 0.06;              // drag correction per meter (higher = more required speed)
        public static double K_SPIN = 0.28; //0.12     // lift correction factor from backspin
        public static double SPIN_RPM = 0;               // expected backspin of the ball
        public static double EFF = 0.95;                 // wheel-to-ball efficiency (0.9–1.0 typical)
        public static double K_TUNE = 1.0;               // final tuning multiplier (easy field tuning)

        // --- Shooter hardware parameters ---
        public static double R_WHEEL_IN = 2.0;           // wheel radius (inches)
        public static double H_WHEEL_IN = 1.5;           // wheel radius (inches)
        public static double MotorToWheel = 1.0;           // wheel radius (inches)
        public static double phi_deg = 50;               // 5-50 Angle of the shot (degrees) // todo
        public static double R_WHEEL_M = R_WHEEL_IN * IN_TO_M; // wheel radius in meters
    }
}
