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
        public static double spindexPower = 1;
        public static double transferPower = 1;
        public static double primingServoUp = 0;
        public static double primingServoDown = 0;

        public static double secondsFor2 = 1.6;
        public static double secondsFor1 = 0.8;
        public static double secondsForHole = 0.1;
    }
    public static class Intake {
        public static double intakePower = 1;
        public static double spitPower = -0.75;
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

        public static double[] green = {0.0097, 0.0421, 0.0318, 0.9807}; //0.2503
        public static double[] purple = {0.0253, 0.0336, 0.0632, 0.9909}; //0.2503
        public static double[] none = {0.0004, 0.0008, 0.0007, 0.0234}; // 1.67
        public static double[] none2 = {0.0005, 0.001, 0.0009, 0.0396}; //1.52

        public static double backDst = 0.7;
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
