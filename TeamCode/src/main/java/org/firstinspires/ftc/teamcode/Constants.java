package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.pedroPathing.control.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.control.PIDFController;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.RGBColors;

public class Constants {

    public static class Drivetrain {

        public static double notTurboPower = 0.6;
        public static double turboPower = 1;

        public static PIDFController stopPID = new PIDFController(new PIDFCoefficients(
                0.035,   // P (strong push against motion)
                0.0,   // I (DON’T use I for stopping)
                0.0,   // D (damping, prevents oscillation)
                0.0    // F
        ));    }

    public static class Indexer {

        public static double indexerTolerance = 0.07;
        public static double wideIndexerTolerance = 0.14;
        public static double transferPower = 1;
        public static double smartShootSpacingSec = 0.45;

        public static double indexerPlugDistance = 8; //inches
    }

    public static class Intake {
        public static double intakePower = 1;
        public static double spitPower = -0.67; //67 67 67 67 67 67 67 67 67 67
        public static double intakeOffPower = 0.08;
        public static double spitTime = 0.3;
        public static double shootPower = .6;
    }

    public static class Vision {
        public static int configTest = 0;
        public static Pose redGoal = new Pose(65, -64);
        public static Pose blueGoal = new Pose(67, 64);

        public static Pose redGoalPhysics = new Pose(64, -67);
        public static Pose blueGoalPhysics = new Pose(67, 67);

        public static Pose redGoalAimOffset = new Pose(0, 0);
        public static Pose blueGoalAimOffset = new Pose(0, 0);
    }

    @Configurable
    public static class Turret {
        public static double closeEnough = 8; //Inches
        public static double turretOffset = 0;

        public static double P = .008;
        public static double I = 0;
        public static double D = 0.0005;

        public static double CCW_F = -0.035;
        public static double CW_F = 0.087;
        public static double CableCW_F = 0.13;
        public static double kV = .045;
        public static double kA = -.07;


        public static PIDFCoefficients turretPID = new PIDFCoefficients(P, I, D, 1);
//        public static PIDFCoefficients turretPID = new PIDFCoefficients(.018, 0, 0.001, 1);
    }

    @Configurable
    public static class Shooter {
        public static PIDFCoefficients shooterVelocityPID = new PIDFCoefficients(.0015, 0, 0, .118599876923);
//        public static PIDFCoefficients shooterVelocityPID = new PIDFCoefficients(0, 0, 0, 0);
        public static double kV = 5368.59375;
        public static double RPMoffset = 0;
        public static double closeEnoughRPM = 50;
        public static double ticksPerRevolution = 28;
        public static double minHoodAngle = 14.3;
        public static double maxHoodAngle = 46;

        public static double entryAngle = Math.toRadians(-30);
        public static double entryHeight = 34;
        public static double setRPM = 0;
        public static double setHood = 0;
    }
    public static class Climber {
        public static double climbedPosition = 0.466;
        public static double storedPosition = 0.1;
    }
    public static class Color {
        // High to low
        public static RGBColors[] greenColorOrder  = {RGBColors.Green, RGBColors.Blue, RGBColors.Red};
        public static RGBColors[] purpleColorOrder = {RGBColors.Blue, RGBColors.Green, RGBColors.Red};

        public static double brushLandsDist = 20;
        public static double i2cDist = 1.2;

//        Prev Method (Dont use)
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

}















