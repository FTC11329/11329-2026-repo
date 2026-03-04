package org.firstinspires.ftc.teamcode.modularAutos;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.RobotSide;

public class Common {
    // All poses should be on blue side
    // Close is close to goal (so usually higher x)
    // Outer is closer to driver wall (so usually higher y)

    public static Pose toRedOffset = new Pose(-2.25,-0.75,0); // red offset remember y is reversed
    public static boolean wasLastRed = false;
    public static class Timings {
        public static int moveAwayRampAmount = 3; // balls

        public static double spikeIntakeTimeOut = 1.25;
        public static double fastSpikeIntakeTimeOut = 1.25;

        public static double shortLeverPressTime = 0.3;
        public static double longLeverPressTime = 1.5;
        public static double shortSTunnelIntakeTimeOut = 1.2;
        public static double longSTunnelIntakeTimeOut = 0;

        public static double humanIntakeTime = 1;
        public static double farShootWaitUntil = 23;

        public static double shootTimeOut = 0.7;
        public static double sortShootTimeOut = 3.5;

        public static double shootVelocity = 5; // in / s
    }
    public static class DrivePower {
        public static double intake = 1;
        public static double shootOnThFly = 0.5;
    }
    public static class StartPoses {
        public static Pose closeInner = new Pose(62.5, 39, Math.toRadians(-90));
        public static Pose closeOuter = new Pose(39.6,57.3, Math.toRadians(-90));
        public static Pose far = new Pose(-63.5, 15.75, Math.toRadians(0));
        public static Pose reZeroAtCorner = new Pose(-65, -61, Math.toRadians(-90));

        static void convert(boolean toRed) {
            closeInner = convertToRed(closeInner, toRed, true);
            far = convertToRed(far, toRed, true);
            reZeroAtCorner = convertToRed(reZeroAtCorner, toRed, true);
        }
    }

    public static class IntakeBallPoses {
        public static Pose intakeSpike1ControlPoint = new Pose(12.2, 20.9, Math.toRadians(90));
        public static Pose intakeSpike1Start = new Pose(13.5, 32, Math.toRadians(90));
        public static Pose intakeSpike1End = new Pose(13.5, 53, Math.toRadians(90));

        public static Pose intakeSpike2ControlPoint = new Pose(-11.7,19.8, Math.toRadians(90));
        public static Pose intakeSpike2Start = new Pose(-10.5, 32, Math.toRadians(90));
        public static Pose intakeSpike2End = new Pose(-10.5, 60, Math.toRadians(90));

        public static Pose intakeSpike3ControlPoint = new Pose(10.1, 14.9, Math.toRadians(90));
        public static Pose intakeSpike3Start = new Pose(-30.5, 32, Math.toRadians(90));
        public static Pose intakeSpike3End = new Pose(-36, 60, Math.toRadians(90));

        public static Pose intakeSpike1FastControlPoint = new Pose(26.7,47.5);
        public static Pose intakeSpike1Fast = new Pose(16.8,47.5);

        public static Pose intakeSpike2FastControlPoint = new Pose(2.6,48);
        public static Pose intakeSpike2Fast = new Pose(-7.9,48);

        public static Pose intakeSpike3FastControlPoint = new Pose(-9,47.9);
        public static Pose intakeSpike3Fast = new Pose(-31.1,47.9);

        public static Pose movingToPushLeverControlPoint = new Pose(-3.8, 25.3);
        public static Pose pushLeverAfterSpike = new Pose(-0.4,56.1, Math.toRadians(60));
        public static Pose pushLever = new Pose(-9.5, 58.8, Math.toRadians(70));
        public static Pose pushLeverFast = new Pose(-11.4,59.7, Math.toRadians(70));
        public static Pose intakeFromSTunnel = new Pose(-17, 58.25, Math.toRadians(45)); // pointing at ramp

        public static Pose intakeHuman = new Pose(-62,62, Math.toRadians(90));
        public static Pose intakeSTunnelAfterHumanControl = new Pose(-56.4,55.8);
        public static Pose intakeSTunnelAfterHuman = new Pose(-49.6,62.9, Math.toRadians(0));

        static void convert(boolean toRed) {
            intakeSpike1ControlPoint = convertToRed(intakeSpike1ControlPoint, toRed);
            intakeSpike2ControlPoint = convertToRed(intakeSpike2ControlPoint, toRed);
            intakeSpike3ControlPoint = convertToRed(intakeSpike3ControlPoint, toRed);
            intakeSpike1Start = convertToRed(intakeSpike1Start, toRed);
            intakeSpike1End = convertToRed(intakeSpike1End, toRed);
            intakeSpike2Start = convertToRed(intakeSpike2Start, toRed);
            intakeSpike2End = convertToRed(intakeSpike2End, toRed);
            intakeSpike3Start = convertToRed(intakeSpike3Start, toRed);
            intakeSpike3End = convertToRed(intakeSpike3End, toRed);
            pushLeverAfterSpike = convertToRed(pushLeverAfterSpike, toRed);
            pushLever = convertToRed(pushLever, toRed);
            pushLeverFast = convertToRed(pushLeverFast, toRed);
            movingToPushLeverControlPoint = convertToRed(movingToPushLeverControlPoint, toRed);
            intakeFromSTunnel = convertToRed(intakeFromSTunnel, toRed);
            intakeHuman = convertToRed(intakeHuman, toRed);
            intakeSTunnelAfterHumanControl = convertToRed(intakeSTunnelAfterHumanControl, toRed);
            intakeSTunnelAfterHuman = convertToRed(intakeSTunnelAfterHuman, toRed);
            intakeSpike1FastControlPoint = convertToRed(intakeSpike1FastControlPoint, toRed);
            intakeSpike1Fast = convertToRed(intakeSpike1Fast, toRed);
            intakeSpike2FastControlPoint = convertToRed(intakeSpike2FastControlPoint, toRed);
            intakeSpike2Fast = convertToRed(intakeSpike2Fast, toRed);
            intakeSpike3FastControlPoint = convertToRed(intakeSpike3FastControlPoint, toRed);
            intakeSpike3Fast = convertToRed(intakeSpike3Fast, toRed);
        }
    }

    public static class ShootPoses {
        public static Pose parkShoot = new Pose(50, 12, Math.toRadians(90));
        public static Pose closeShoot = new Pose(36, 36, Math.toRadians(90));
        public static Pose midShoot = new Pose(8.9,18.5, Math.toRadians(90));
        public static Pose farShoot = new Pose(-54.7,9.9, Math.toRadians(90));

        public static Pose fromStartCloseToMidShootControl = new Pose(45.3,49.6);

        public static Pose panicShoot = new Pose(24, 24, Math.toRadians(0));

        public static Pose optimalSpike1Start = new Pose(32.4,38.4, Math.toRadians(123));
        public static Pose optimalSpike2Start = new Pose(15,26.5, Math.toRadians(120));
        public static Pose optimalSpike3Start = new Pose(14,24, Math.toRadians(133));

        public static Pose optimalLeverStart = new Pose(10.7, 21.6, Math.toRadians(122));


        static void convert(boolean toRed) {
            parkShoot = convertToRed(parkShoot, toRed);
            closeShoot = convertToRed(closeShoot, toRed);
            midShoot = convertToRed(midShoot, toRed);
            farShoot = convertToRed(farShoot, toRed);
            fromStartCloseToMidShootControl = convertToRed(fromStartCloseToMidShootControl, toRed);
            panicShoot = convertToRed(panicShoot, toRed);
            optimalSpike1Start = convertToRed(optimalSpike1Start, toRed);
            optimalSpike2Start = convertToRed(optimalSpike2Start, toRed);
            optimalSpike3Start = convertToRed(optimalSpike3Start, toRed);
            optimalLeverStart = convertToRed(optimalLeverStart, toRed);
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
