package org.firstinspires.ftc.teamcode.modularAutos;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.RobotSide;

public class CommonCRI {
    // All poses should be on blue side
    // Close is close to goal (so usually higher x)
    // Outer is closer to driver wall (so usually higher y)

    public static Pose toRedOffset = new Pose(2.5,-2.5); // red offset remember y is reversed
    public static boolean wasLastRed = false;
    public static class TValues {
        public static double fastInterpolationIntakeStart = 0.8;

        public static double fastInterpolationIntakeStartFar = 0.5;
        public static double fastInterpolationIntakeEndFar = 0.65;

        public static double fastInterpolationSpikeShootStart = 0.6;
        public static double fastInterpolationSpikeShootEnd = 0.75;

        public static double fastInterpolationLeverStart = 0.6;
        public static double fastInterpolationLeverEnd = 0.7;

        public static double fastInterpolationPreloadStart = 0.65;
        public static double fastInterpolationPreloadEnd = 0.8;

    }
    public static class Timings {

        public static double unjamTimeOut = 1.5;
        public static double unjamTimeOutFar = 2;
        public static double unjamTimeOutSort = 2.5;
        public static double unjamTimeOutFarSort = 3;

        public static double spikeIntakeTimeOut = 0.5;

        public static double shortLeverPressTime = 0.3;
        public static double longLeverPressTime = 2.5;
        public static double shortSTunnelIntakeTimeOut = 1.2;
        public static double longSTunnelIntakeTimeOut = 0;

        public static double humanIntakeTime = 0.5;
        public static double farShootWaitUntil = 23;

        public static double spikeHightOut = 85;


    }
    public static class DrivePower {
        public static double intake = 1;
        public static double shootOnThFly = 0.25;
    }
    public static class StartPoses {
        public static Pose closeInner = new Pose(88.5, 59.8, Math.toRadians(-90)); // this is against the goal and wall, facing the obelisk
        public static Pose perpPrism = new Pose(83,27, Math.toRadians(180)); // this is against the prism, facing the gate
        public static Pose closeOuter = new Pose(64.55, 77.5, Math.toRadians(90)); // this is up against the goal and secret tunnel
        public static Pose far = new Pose(-83.9,13, Math.toRadians(0));

        static void convert(boolean toRed) {
            closeInner = convertToRed(closeInner, toRed, true);
            closeOuter = convertToRed(closeOuter, toRed, true);
            perpPrism = convertToRed(perpPrism, toRed, true);
            far = convertToRed(far, toRed, true);
        }
    }

    public static class IntakeBallPoses {
        public static Pose intakeSpike1ControlPointClose = new Pose(20.2,40.5, Math.toRadians(90));
        public static Pose intakeSpike1ControlPointFar = new Pose(15.3,31.6, Math.toRadians(90));
        public static Pose intakeSpike1Start = new Pose(11.6,49.7, Math.toRadians(90));
        public static Pose intakeSpike1End = new Pose(9,87, Math.toRadians(90));

        public static Pose intakeSpike2ControlPointClose = new Pose(-9.2,28, Math.toRadians(90));
        public static Pose intakeSpike2ControlPointFar = new Pose(-6.9,23.2, Math.toRadians(90));
        public static Pose intakeSpike2Start = new Pose(-5.7,49.7, Math.toRadians(90));
        public static Pose intakeSpike2End = new Pose(-9.4,87, Math.toRadians(90));

        public static Pose intakeSpike3ControlPoint = new Pose(-24.9,36, Math.toRadians(90));
        public static Pose intakeSpike3StartClose = new Pose(-29.6,49.7, Math.toRadians(90));
        public static Pose intakeSpike3StartFar = new Pose(-33.2,49.7, Math.toRadians(90));
        public static Pose intakeSpike3End = new Pose(-32.9,87, Math.toRadians(90));


        public static Pose movingToPushLeverControlPoint = new Pose(11.6,59.9);
        public static Pose pushLeverAfterSpike1 = new Pose(16.1,75.8, Math.toRadians(90));
        public static Pose pushLever = new Pose(9,83.25, Math.toRadians(63));
        public static Pose pushLeverFromSTunnel = new Pose(4,80, Math.toRadians(63));
        public static Pose intakeFromSTunnel = new Pose(6, 75, Math.toRadians(45)); // pointing at ramp

        public static Pose intakeSTunnelDiagonalStart = new Pose(-62.9,78.3, Math.toRadians(45));
        public static Pose intakeSTunnelDiagonalEnd = new Pose(-11,78.3, Math.toRadians(45));

        public static Pose intakeHumanDiagonal = new Pose(-75,78, Math.toRadians(130));
        public static Pose intakeHumanStrait = new Pose(-81.5,81.7, Math.toRadians(90));
        public static Pose intakeHumanStraitControlPoint = new Pose(-87.8,23.5, Math.toRadians(90));

        static void convert(boolean toRed) {
            intakeSpike1ControlPointClose = convertToRed(intakeSpike1ControlPointClose, toRed);
            intakeSpike1ControlPointFar = convertToRed(intakeSpike1ControlPointFar, toRed);
            intakeSpike2ControlPointClose = convertToRed(intakeSpike2ControlPointClose, toRed);
            intakeSpike2ControlPointFar = convertToRed(intakeSpike2ControlPointFar, toRed);
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
            intakeSTunnelDiagonalStart = convertToRed(intakeSTunnelDiagonalStart, toRed);
            intakeSTunnelDiagonalEnd = convertToRed(intakeSTunnelDiagonalEnd, toRed);
            intakeHumanDiagonal = convertToRed(intakeHumanDiagonal, toRed);
            intakeHumanStrait = convertToRed(intakeHumanStrait, toRed);
        }
    }

    public static class ShootPoses {
        public static Pose parkShoot =  new Pose(72, 36, Math.toRadians(135));
        public static Pose midShoot =   new Pose(32.9,42.5, Math.toRadians(135));
        public static Pose distantShoot = new Pose(2.7,9.9, Math.toRadians(135));
        public static Pose farShoot =   new Pose(-28.3,13.1, Math.toRadians(85));
        public static Pose distantFarShoot = new Pose(-58,28.5, Math.toRadians(0));

        public static Pose optimalSpike1StartClose = new Pose(30.9,35.3, Math.toRadians(150));
        public static Pose optimalSpike2StartClose = new Pose(24.7,29.5, Math.toRadians(160));
        public static Pose optimalSpike3StartClose = new Pose(24.1,25.9, Math.toRadians(170));
        public static Pose optimalSpike1StartFar = new Pose(-27.7,12.5, Math.toRadians(30));
        public static Pose optimalSpike2StartFar = new Pose(-27.7,12.5, Math.toRadians(30));
        public static Pose optimalSpike3StartFar = new Pose(-33.2,16.4, Math.toRadians(90));

        public static Pose optimalRampStart = new Pose(36.9,42.6, Math.toRadians(140));
        public static Pose optimalHumanPlayerStart = new Pose(-67.9,26.4, Math.toRadians(100));
        public static Pose optimalVisionStartClose = new Pose(52,52.5, Math.toRadians(45));
        public static Pose optimalVisionStart = new Pose(-40,15, Math.toRadians(120));

        static void convert(boolean toRed) {
            parkShoot = convertToRed(parkShoot, toRed);
            midShoot = convertToRed(midShoot, toRed);
            distantShoot = convertToRed(distantShoot, toRed);
            farShoot = convertToRed(farShoot, toRed);
            distantFarShoot = convertToRed(distantFarShoot, toRed);
            optimalSpike1StartClose = convertToRed(optimalSpike1StartClose, toRed);
            optimalSpike2StartClose = convertToRed(optimalSpike2StartClose, toRed);
            optimalSpike3StartClose = convertToRed(optimalSpike3StartClose, toRed);
            optimalSpike1StartFar = convertToRed(optimalSpike1StartFar, toRed);
            optimalSpike2StartFar = convertToRed(optimalSpike2StartFar, toRed);
            optimalSpike3StartFar = convertToRed(optimalSpike3StartFar, toRed);
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
