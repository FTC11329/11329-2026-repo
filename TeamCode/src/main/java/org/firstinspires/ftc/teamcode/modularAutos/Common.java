package org.firstinspires.ftc.teamcode.modularAutos;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.RobotSide;

public class Common {
    // All poses should be on blue side
    // Close is close to goal (so usually higher x)
    // Outer is closer to driver wall (so usually higher y)

    public static class Timings {
        public static int moveAwayRampAmount = 3; // balls
        public static double rampIntakeTimeOut = 2;
        public static double spikeIntakeTimeOut = 1;
        public static double longLeverPressTime = 0.5;
        public static double shortLeverPressTime = 0.3;
        public static double shootVelocity = 10; // in / s
    }
    public static class DrivePower {
        public static double intake = 1;
        public static double shootOnThFly = 0.5;
    }
    public static class StartPoses {
        public static Pose closeInner = new Pose(62.5, 36, Math.toRadians(90));
        public static Pose farOuter = new Pose(62.5, 36, Math.toRadians(90));

        static void convert(boolean toRed) {
            closeInner = convertToRed(closeInner, toRed);
            farOuter = convertToRed(farOuter, toRed);
        }
    }

    public static class IntakeBallPoses {
        public static Pose intakeSpike1ControlPoint = new Pose(12.2, 20.9, Math.toRadians(90));
        public static Pose intakeSpike1Start = new Pose(12, 32, Math.toRadians(90));
        public static Pose intakeSpike1End = new Pose(12, 53, Math.toRadians(90));

        public static Pose intakeSpike2ControlPoint = new Pose(-11.7,19.8, Math.toRadians(90));
        public static Pose intakeSpike2Start = new Pose(-12, 32, Math.toRadians(90));
        public static Pose intakeSpike2End = new Pose(-12, 60, Math.toRadians(90));

        public static Pose intakeSpike3ControlPoint = new Pose(10.1, 14.9, Math.toRadians(90));
        public static Pose intakeSpike3Start = new Pose(-32, 32, Math.toRadians(90));
        public static Pose intakeSpike3End = new Pose(-36, 60, Math.toRadians(90));

        public static Pose movingToPushLeverControlPoint = new Pose(-3.8, 25.3);
        public static Pose pushLeverAfterSpike = new Pose(-0.4,56.1, Math.toRadians(60));
        public static Pose pushLever = new Pose(-8.8,59.3, Math.toRadians(60));
        public static Pose intakeFromSTunnel = new Pose(-17, 58.25, Math.toRadians(45)); // pointing at ramp

        public static Pose startSTunnelPose = new Pose(-27, 59.42, Math.toRadians(155)); //pointing at human
        public static Pose endSTunnelPose = new Pose(-53, 63, Math.toRadians(180));

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
            startSTunnelPose = convertToRed(startSTunnelPose, toRed);
            endSTunnelPose = convertToRed(endSTunnelPose, toRed);
        }
    }

    public static class ShootPoses {
        public static Pose parkShoot = new Pose(50, 12, Math.toRadians(90));
        public static Pose closeShoot = new Pose(36, 36, Math.toRadians(90));
        public static Pose midShoot = new Pose(8.9,19.5, Math.toRadians(90));
        public static Pose farShoot = new Pose(-60, 12, Math.toRadians(90));

        static void convert(boolean toRed) {
            parkShoot = convertToRed(parkShoot, toRed);
            closeShoot = convertToRed(closeShoot, toRed);
            midShoot = convertToRed(midShoot, toRed);
            farShoot = convertToRed(farShoot, toRed);
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
        StartPoses.convert(toRed);
        IntakeBallPoses.convert(toRed);
        ShootPoses.convert(toRed);
    }

    public static Pose convertToRed(Pose convertPose, boolean toRed) {
        if (!toRed) return convertPose;
        return new Pose(
                convertPose.getX(),
                -convertPose.getY(),
                -convertPose.getHeading()
        );
    }
}
