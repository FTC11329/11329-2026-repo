package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.control.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;

import kotlin.jvm.JvmField;

@Configurable
public class Constants {
    public static class Indexer {

        public static double indexerTolerance = 0.07;  // has to be big for smart shooting, has to be small for spit timing
        public static double wideIndexerTolerance = 0.14;  // has to be big for smart shooting, has to be small for spit timing

        public static double spindexPower = 0.45;

        public static double transferPower = 1;

        public static double smartFeedSec = 0.2;
        public static double smartShootSpacingSec = 1;
        public static double spitTime = 0.15;
    }

    public static class Intake {
        public static double intakePower = 1;
        public static double spitPower = -0.67; //67 67 67 67 67 67 67 67 67 67
        public static double intakeOffPower = 0.2;
        public static double spitTime = 0.3;
    }

    public static class Vision {
        public static Pose redGoal = new Pose(60, -60);
        public static Pose blueGoal = new Pose(60, 60);
        public static Pose redGoalAimOffset = new Pose(4, -9);
        public static Pose blueGoalAimOffset = new Pose(6, 4);

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
        public static double closeEnough = 8; //Inches
        public static double turretOffset = 0;


//        public static double P = 0.025;
//        public static double I = 0.001;
//        public static double D = 0.001;
//        public static double P = 0.005; //.011
        public static double P = 0.015; //.011
        public static double I = 0;
        public static double D = 0;


        public static double CCW_F = -0.045;
        public static double CW_F = 0.1;
        public static double kV = .095;
        public static double kA = .012;

        public static final double cableHangL = 248.8;

        public static PIDFCoefficients turretPID = new PIDFCoefficients(P, I, D, 1);
//        public static double P = 0;
//        public static double I = 0;
//        public static double D = 0;
//        public static double F = 0;
    }
    @Configurable
    public static class Shooter {
        @Sorter(sort = 0)
        @JvmField
        public static PIDFCoefficients shooterVelocityPID = new PIDFCoefficients(.005, 0, 0, 0);
        @Sorter(sort = 1)
        @JvmField
        public static double kV = 4456;
        @Sorter(sort =2)
        @JvmField
        public static double RPMoffset = 0;
        @Sorter(sort = 3)
        public static double closeEnoughRPM = 50;
        @Sorter(sort = 4)
        public static double ticksPerRevolution = 28;
        @Sorter(sort = 5)
        public static double minHoodAngle = 0;
        @Sorter(sort = 6)
        public static double maxHoodAngle = 53.5;
    }
    public static class Color {
//            old values
//        public static double[] green = {0.0059, 0.0242, 0.0196, 0.947}; //0.311
//        public static double[] greenFar = {0.0007, 0.0027, 0.0023, 0.1934}; //0.85
//        public static double[] greenWeird = {0.0029, 0.0121, 0.0101, 0.82}; //0.43
//        public static double[] greenFakeHole = {0.0015, 0.0067, 0.0054, 0.575}; //0.73
//
//        public static double[] purple = {0.0104, 0.013, 0.0276, 0.95}; //0.36
//        public static double[] purpleFar = {0.0035, 0.0045, 0.0094, 0.689}; //0.64
//        public static double[] purpleWeird = {0.0018, 0.0023, 0.0049, 0.376}; //0.77
//
//        public static double[] none = {0.0003, 0.0007, 0.0006, 0.0187}; //1.97
        public static double[] green = {0.0269, 0.0792, 0.06, 0.9551}; //0.743
        public static double[] greenFar = {0.0147, 0.0361, 0.0274, 0.978}; //1.95
        public static double[] greenWeird = {0.0157, 0.0414, 0.0313, 0.9828}; //4.7
        public static double[] greenFakeHole = {-1, -1, -1, -1}; //0.0

        public static double[] purple = {0.062, 0.0719, 0.0898, 0.9973}; //0.48
        public static double[] purpleFar = {0.0229, 0.0302, 0.0324, 0.9816}; //1.73
        public static double[] purpleWeird = {0.0666, 0.0763, 0.0966, 0.9976}; //0.62

        public static double[] none = {0.012, 0.0271, 0.02, 0.9622}; //6
        public static double[] none2 = {-1, -1, -1, -1}; // unused

        public static double backDst = 2.3;
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
