package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.pedroPathing.control.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;

public class Constants {

    public static class Indexer {

        public static double indexerTolerance = 0.07;
        public static double wideIndexerTolerance = 0.14;
        public static double transferPower = 1;

        public static double smartFeedSec = 0.2;
        public static double smartShootSpacingSec = 0.3;

        public static double indexerPlugDistance = 8; //inches
    }

    public static class Intake {
        public static double intakePower = 1;
        public static double spitPower = -0.67; //67 67 67 67 67 67 67 67 67 67
        public static double intakeOffPower = 0.08;
        public static double spitTime = 0.3;
    }

    public static class Vision {
        public static int configTest = 0;
        public static Pose redGoal = new Pose(60, -60);
        public static Pose blueGoal = new Pose(60, 60);

        public static Pose redGoalPhysics = new Pose(64, -67);
        public static Pose blueGoalPhysics = new Pose(67, 67);

        public static Pose redGoalAimOffset = new Pose(0, -3);
        public static Pose blueGoalAimOffset = new Pose(1, 2.5);
    }

    public static class Turret {
        public static double closeEnough = 8; //Inches
        public static double turretOffset = 0;
        public static double P = 0.015; //todo: see if we can't up this too make the turret a little snappier
        public static double I = 0;
        public static double D = 0;


        public static double CCW_F = -0.045;
        public static double CW_F = 0.1;
        public static double kV = .095;
        public static double kA = .012;


        public static PIDFCoefficients turretPID = new PIDFCoefficients(P, I, D, 1);
    }
    public static class Shooter {
        public static PIDFCoefficients shooterVelocityPID = new PIDFCoefficients(.005, 0, 0, 0);
        public static double kV = 4545;
        public static double RPMoffset = 0;
        public static double closeEnoughRPM = 50;
        public static double ticksPerRevolution = 28;
        public static double minHoodAngle = 5;
        public static double maxHoodAngle = 53.5;

        public static double entryAngle = Math.toRadians(-30);
        public static double entryHeight = 34;
    }
    public static class Color {
        public static double[] green = {0.0269, 0.0792, 0.06, 0.9551}; //0.743
        public static double[] greenFar = {0.0147, 0.0361, 0.0274, 0.978}; //1.95
        public static double[] greenWeird = {0.0157, 0.0414, 0.0313, 0.9828}; //4.7
        public static double[] greenFakeHole = {-1, -1, -1, -1}; //0.0

        public static double[] purple = {0.062, 0.0719, 0.0898, 0.9973}; //0.48
        public static double[] purpleFar = {0.0229, 0.0302, 0.0324, 0.9816}; //1.73
        public static double[] purpleWeird = {0.0666, 0.0763, 0.0966, 0.9976}; //0.62

        public static double[] none = {0.012, 0.0271, 0.02, 0.9622}; //6
        public static double[] none2 = {-1, -1, -1, -1}; // unused

//      with orange tape
//        public static double[] green = {0.209, 0.4111, 0.1952, 0.9998}; //0.583
//        public static double[] greenFar = {0.1937, 0.3655, 0.182, 0.9998}; //1.95
//        public static double[] greenFakeHole = {0.1997, 0.3869, 0.189, 0.9998}; //0.63
//        public static double[] greenWeird = {-1, -1, -1, -1}; //
//
//        public static double[] purple = {0.223, 0.384, 0.1855, 0.9998}; //0.88
//        public static double[] purpleFar = {0.199, 0.365, 0.1813, 0.9998}; //2.07
//        public static double[] purpleWeird = {0.1954, 0.362, 0.1813, 0.9997}; //2.22
//
//        public static double[] none = {0.1922, 0.359, 0.1792, 0.9997}; //3.1
//        public static double[] none2 = {-1, -1, -1, -1}; // unused

        public static double backDst = 2.2;
    }
    public static class Climber {
        public static double climbedPosition = 0.4;
        public static double storedPosition = 1;
    }
}
