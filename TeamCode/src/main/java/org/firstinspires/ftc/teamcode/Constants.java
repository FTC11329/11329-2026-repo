package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.control.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;

public class Constants {
    // todo replace all placeholders with real constants
    public static double PlacholdereDouble = 0;
    public static boolean PlaceHOLDERboooLEAN = false;

    @com.bylazar.ftcontrol.panels.configurables.annotations.Configurable
    public static class Indexer {
        public static double spindexPower = 0.9;

        public static double farSpindexPower = 0.3;
        public static double farDistance = 90;

        public static double scanningPower = 0.5;
        public static double transferPower = 1;

        public static double secondsFor2 = 1.6; // todo remove after 2nd comp
        public static double secondsFor1 = 0.8;
        public static double secondsForHole = 0.1;
    }

    @Configurable
    public static class Intake {
        public static double intakePower = 1;
        public static double spitPower = -0.75;
    }

    public static class Vision {
        public static Pose redGoal = new Pose(60, -60);
        public static Pose blueGoal = new Pose(60, 60);

        public static Pose blueReset = new Pose(7.5, 39.7);
        public static Pose redReset = new Pose(7.5, -39.7);



        public static Pose redTag = new Pose(55.64, -58.34);
        public static Pose blueTag = new Pose(55.64, 58.34);

        public static Pose cameraPos = new Pose(0, 4.47);
        public static Pose3D cameraPosForInfoForGlacierInIN = new Pose3D(new Position(DistanceUnit.INCH, 0, -7, 15.75, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 28.3, 0, 0));

        public static double pitch = 80;
        public static double tagAngle = 80;
    }

    public static class Turret {
        public static double closeEnough = 10; //Inches
        public static double turretOffset = 1;


        public static double P = 0.0207;
        public static double I = 0.000085;
        public static double D = 0.00056;
        public static double leftF = -0.03;
        public static double rightF = 0.11;

        public static PIDFCoefficients turretPID = new PIDFCoefficients(P, I, D, 1);
//        public static double P = 0;
//        public static double I = 0;
//        public static double D = 0;
//        public static double F = 0;
    }
    public static class Shooter {
        public static double maxHoodAngle = 53.5;
        public static double closeEnoughRPM = 1000;
        public static double ticksPerRevolution = 28;
        public static double P = 0.015;
        public static double I = 0.00008;
        public static double D = 0.000001;
        public static double F = 0.085;
        public static PIDFCoefficients shooterVelocityPID = new PIDFCoefficients(P, I, D, F);
    }
    public static class Color {

        public static double[] green = {0.0097, 0.0421, 0.0318, 0.9807}; //0.2503
        public static double[] greenFar = {0, 0, 0, 0};
        public static double[] purple = {0.0253, 0.0336, 0.0632, 0.9909}; //0.2503
        public static double[] purpleFar = {0, 0, 0, 0};
        public static double[] none = {0.0004, 0.0008, 0.0007, 0.0234}; // 1.67
        public static double[] none2 = {0.0005, 0.001, 0.0009, 0.0396}; //1.52

        public static double backDst = 1;
    }

    public static class ShootingZone {

            public static Pose bigCenter = new Pose(0, 0);
            public static Pose bigRight = new Pose(-72, 72);
            public static Pose bigLeft = new Pose(-72, -72);
            public static Pose smallCenter = new Pose(48, 0);
            public static Pose smallRight = new Pose(72, 24);
            public static Pose smallLeft = new Pose(72, -24);
    }

    public static class ShooterParamaters {
        // --- Fixed geometry ---
        public static double LAUNCHER_HEIGHT_IN = 12.063;   // inches: how high the ball leaves the shooter
        public static double TARGET_HEIGHT_IN   = 40.0;   // inches: height of the target off the ground
        public static double TAG_TO_TARGET_IN   = 9;   // inches: distance from AprilTag to middle of the targeted point

        // --- Physical constants ---
        public static double IN_TO_M = 0.0254;           // inches to meters conversion
        public static double G = 9.81 / IN_TO_M;                   // gravity (m/s^2)

        // --- Shooter hardware parameters ---
        public static double R_WHEEL_IN = 2.0;           // wheel radius (inches)
        public static double H_WHEEL_IN = 1.5;           // wheel radius (inches)
        public static double MotorToWheel = 1.25;           // wheel radius (inches)
        public static double phi_deg = 50;               // 5-50 Angle of the shot (degrees) // todo
        public static double R_WHEEL_M = R_WHEEL_IN * IN_TO_M; // wheel radius in meters
    }
}
