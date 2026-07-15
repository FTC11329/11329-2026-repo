package org.firstinspires.ftc.teamcode.modularAutos;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.RobotSide;

public class CommonCRI {
    // All poses should be on blue side
    // Close is close to goal (so usually higher x)
    // Outer is closer to driver wall (so usually higher y)

    public static Pose toRedOffset = new Pose(0,0); // red offset remember y is reversed
    public static boolean wasLastRed = false;
    public static class TValues {
        public static double fastInterpolationIntakeStart = 0.8;

        public static double fastInterpolationSpikeShootStart = 0.6;
        public static double fastInterpolationSpikeShootEnd = 0.75;

        public static double fastInterpolationLeverStart = 0.63;
        public static double fastInterpolationLeverEnd = 0.73;

        public static double fastInterpolationPreloadStart = 0.65;
        public static double fastInterpolationPreloadEnd = 0.8;

    }
    public static class Timings {

        public static double unjamTimeOut = 1.5;
        public static double unjamTimeOutFar = 2;
        public static double unjamTimeOutSort = 2.5;
        public static double unjamTimeOutFarSort = 3;

        public static double spikeIntakeTimeOut = 1.4;

        public static double shortLeverPressTime = 0.3;
        public static double longLeverPressTime = 2.5;
        public static double shortSTunnelIntakeTimeOut = 1.2;
        public static double longSTunnelIntakeTimeOut = 0;


        public static double notSpike1HightOut = 85;

    }
    public static class DrivePower {
        public static double intake = 1;
        public static double shootOnThFly = 0.25;
    }
    public static class StartPoses {
        public static Pose closeInner = new Pose(88.5, 59.8, Math.toRadians(-90)); // this is against the goal and wall, facing the obelisk
        public static Pose closeOuter = new Pose(64.55, 77.5, Math.toRadians(90)); // this is up against the goal and secret tunnel
        public static Pose far = new Pose(-88, 39.8, Math.toRadians(0));

        static void convert(boolean toRed) {
            closeInner = convertToRed(closeInner, toRed, true);
            closeOuter = convertToRed(closeOuter, toRed, true);
            far = convertToRed(far, toRed, true);
        }
    }

    public static class IntakeBallPoses {
        public static Pose intakeSpike1ControlPoint = new Pose(20.2,40.5, Math.toRadians(90));
        public static Pose intakeSpike1Start = new Pose(11.6,50, Math.toRadians(90));
        public static Pose intakeSpike1End = new Pose(9,84, Math.toRadians(90));

        public static Pose intakeSpike2ControlPoint = new Pose(0.5,35.5, Math.toRadians(90));
        public static Pose intakeSpike2Start = new Pose(-5.7,54, Math.toRadians(90));
        public static Pose intakeSpike2End = new Pose(-9.4,84, Math.toRadians(90));

        public static Pose intakeSpike3ControlPoint = new Pose(-24.9,36, Math.toRadians(90));
        public static Pose intakeSpike3StartClose = new Pose(-29.6,54.4, Math.toRadians(90));
        public static Pose intakeSpike3StartFar = new Pose(-35,54.4, Math.toRadians(90));
        public static Pose intakeSpike3End = new Pose(-32.9,84, Math.toRadians(90));


        public static Pose movingToPushLeverControlPoint = new Pose(11.6,59.9);
        public static Pose pushLeverAfterSpike1 = new Pose(16.1,75.8, Math.toRadians(90));
        public static Pose pushLever = new Pose(12,83.25, Math.toRadians(63));
        public static Pose intakeFromSTunnel = new Pose(6, 75, Math.toRadians(45)); // pointing at ramp


        public static Pose intakeHumanDiagonal = new Pose(-46,80, Math.toRadians(154));
        public static Pose intakeHumanDiagonalToStrait = new Pose(-82,84.5, Math.toRadians(180));

        static void convert(boolean toRed) {
            intakeSpike1ControlPoint = convertToRed(intakeSpike1ControlPoint, toRed);
            intakeSpike2ControlPoint = convertToRed(intakeSpike2ControlPoint, toRed);
            intakeSpike3ControlPoint = convertToRed(intakeSpike3ControlPoint, toRed);
            intakeSpike1Start = convertToRed(intakeSpike1Start, toRed);
            intakeSpike1End = convertToRed(intakeSpike1End, toRed);
            intakeSpike2Start = convertToRed(intakeSpike2Start, toRed);
            intakeSpike2End = convertToRed(intakeSpike2End, toRed);
            intakeSpike3StartClose = convertToRed(intakeSpike3StartClose, toRed);
            intakeSpike3StartFar = convertToRed(intakeSpike3StartFar, toRed);
            intakeSpike3End = convertToRed(intakeSpike3End, toRed);
            pushLeverAfterSpike1 = convertToRed(pushLeverAfterSpike1, toRed);
            pushLever = convertToRed(pushLever, toRed);
            movingToPushLeverControlPoint = convertToRed(movingToPushLeverControlPoint, toRed);
            intakeFromSTunnel = convertToRed(intakeFromSTunnel, toRed);
            intakeHumanDiagonal = convertToRed(intakeHumanDiagonal, toRed);
            intakeHumanDiagonalToStrait = convertToRed(intakeHumanDiagonalToStrait, toRed);
        }
    }

    public static class ShootPoses {
        public static Pose parkShoot =  new Pose(72, 36, Math.toRadians(135));
        public static Pose midShoot =   new Pose(32.9,42.5, Math.toRadians(90));
        public static Pose distantShoot =   new Pose(2.7,9.9, Math.toRadians(90));
        public static Pose farShoot =   new Pose(-28.3,13.1, Math.toRadians(85));

        public static Pose optimalSpike1Start = new Pose(30.9,35.3, Math.toRadians(150));
        public static Pose optimalSpike2Start = new Pose(24.7,29.5, Math.toRadians(160));
        public static Pose optimalSpike3Start = new Pose(24.1,25.9, Math.toRadians(170));
        public static Pose optimalRampStart = new Pose(36.9,42.6, Math.toRadians(140));
        public static Pose optimalHumanPlayerStart = new Pose(33.24,37.25, Math.toRadians(155));
        public static Pose optimalVisionStart = new Pose(33.24,38.25, Math.toRadians(155));

        static void convert(boolean toRed) {
            parkShoot = convertToRed(parkShoot, toRed);
            midShoot = convertToRed(midShoot, toRed);
            farShoot = convertToRed(farShoot, toRed);
            optimalSpike1Start = convertToRed(optimalSpike1Start, toRed);
            optimalSpike2Start = convertToRed(optimalSpike2Start, toRed);
            optimalSpike3Start = convertToRed(optimalSpike3Start, toRed);
            optimalRampStart = convertToRed(optimalRampStart, toRed);
            optimalHumanPlayerStart = convertToRed(optimalHumanPlayerStart, toRed);
            optimalVisionStart = convertToRed(optimalVisionStart, toRed);
        }
    }

    /**
     * Call this ONCE at init
     */
    public static void init(Robot robot) {
        init(robot.robotSide);
    }

    public static void init(RobotSide robotSide) {
        init(robotSide == RobotSide.Red);
    }

    public static void init(boolean toRed) {
        // if wrong
        if (toRed && StartPoses.closeInner.getY() > 0) {
            StartPoses.convert(true);
            ShootPoses.convert(true);
            IntakeBallPoses.convert(true);
        } else if (!toRed && StartPoses.closeInner.getY() < 0) {
            StartPoses.convert(false);
            ShootPoses.convert(false);
            IntakeBallPoses.convert(false);
        }
        wasLastRed = toRed;
    }

    public static Pose convertToRed(Pose convertPose, boolean toRed) {
        return convertToRed(convertPose, toRed, false);
    }

    public static Pose convertToRed(Pose convertPose, boolean toRed, boolean startPose) {
        if (startPose) {
            if (toRed != wasLastRed) {
                return new Pose(
                        convertPose.getX(),
                        (-convertPose.getY()),
                        (-convertPose.getHeading())
                );
            } else {
                return convertPose;
            }
        }

        if (toRed && !wasLastRed) {
            convertPose = convertPose.plus(toRedOffset);
        } else if (!toRed && wasLastRed) {
            convertPose = convertPose.minus(toRedOffset);
        }

        if (toRed != wasLastRed) {
            return new Pose(
                    convertPose.getX(),
                    (-convertPose.getY()),
                    (-convertPose.getHeading())
            );
        } else {
            return convertPose;
        }
    }
}
