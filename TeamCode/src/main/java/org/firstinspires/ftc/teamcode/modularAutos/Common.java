package org.firstinspires.ftc.teamcode.modularAutos;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.RobotSide;

public class Common {
    // All poses should be on blue side
    // Close is close to goal (so usually higher x)
    // Outer is closer to driver wall (so usually higher y)

    public static Pose toRedOffset = new Pose(-1,0,0); // red offset remember y is reversed
    public static boolean wasLastRed = false;
    public static class Timings {
        public static int moveAwayRampAmount = 3; // balls

        public static double spikeIntakeTimeOut = 1.25;

        public static double shortLeverPressTime = 0.5;
        public static double longLeverPressTime = 2;
        public static double shortSTunnelIntakeTimeOut = 1.5;
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
        public static Pose far = new Pose(-63.5, 15.75, Math.toRadians(0));

        static void convert(boolean toRed) {
            closeInner = convertToRed(closeInner, toRed, true);
            far = convertToRed(far, toRed, true);
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

        public static Pose movingToPushLeverControlPoint = new Pose(-3.8, 25.3);
        public static Pose pushLeverAfterSpike = new Pose(-0.4,56.1, Math.toRadians(60));
        public static Pose pushLever = new Pose(-9.5, 58.8, Math.toRadians(70));
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
            movingToPushLeverControlPoint = convertToRed(movingToPushLeverControlPoint, toRed);
            intakeFromSTunnel = convertToRed(intakeFromSTunnel, toRed);
            intakeHuman = convertToRed(intakeHuman, toRed);
            intakeSTunnelAfterHumanControl = convertToRed(intakeSTunnelAfterHumanControl, toRed);
            intakeSTunnelAfterHuman = convertToRed(intakeSTunnelAfterHuman, toRed);
        }
    }

    public static class ShootPoses {
        public static Pose parkShoot = new Pose(50, 12, Math.toRadians(90));
        public static Pose closeShoot = new Pose(36, 36, Math.toRadians(90));
        public static Pose midShoot = new Pose(8.9,18.5, Math.toRadians(90));
        public static Pose farShoot = new Pose(-54.7,9.9, Math.toRadians(90));

        public static Pose panicShoot = new Pose(25,25, Math.toRadians(45));

        static void convert(boolean toRed) {
            parkShoot = convertToRed(parkShoot, toRed);
            closeShoot = convertToRed(closeShoot, toRed);
            midShoot = convertToRed(midShoot, toRed);
            farShoot = convertToRed(farShoot, toRed);
            panicShoot = convertToRed(panicShoot, toRed);
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
