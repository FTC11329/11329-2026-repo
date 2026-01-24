package org.firstinspires.ftc.teamcode.modularAutos;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.RobotSide;

public class CommonPoses {
    // All poses should be on blue side
    // Close is close to goal (so usually higher x)
    // Outer is closer to driver wall (so usually higher y)

    public static class StartPoses {
        public static Pose closeInner = new Pose(62.5, 36, Math.toRadians(90));
        public static Pose farOuter = new Pose(62.5, 36, Math.toRadians(90));

        static void convert(boolean toRed) {
            closeInner = convertToRed(closeInner, toRed);
            farOuter = convertToRed(farOuter, toRed);
        }
    }

    public static class IntakeBallPoses {
        public static Pose intakeSpike1Start = new Pose(12, 32, Math.toRadians(90));
        public static Pose intakeSpike1End = new Pose(12, 48, Math.toRadians(90));

        public static Pose intakeSpike2Start = new Pose(-12, 32, Math.toRadians(90));
        public static Pose intakeSpike2End = new Pose(-12, 54, Math.toRadians(90));

        public static Pose intakeSpike3Start = new Pose(-36, 32, Math.toRadians(90));
        public static Pose intakeSpike3End = new Pose(-36, 54, Math.toRadians(90));

        public static Pose pushLever = new Pose(-4.5,55.5, Math.toRadians(90));
        public static Pose movingToIntakeSTunnelControlPoint = new Pose(-16.5,51);
        public static Pose intakeFromSTunnel = new Pose(-22,59.25, Math.toRadians(45)); // pointing at ramp

        public static Pose startSTunnelPose = new Pose(-27, 59.42, Math.toRadians(155)); //pointing at human
        public static Pose endSTunnelPose = new Pose(-53, 63, Math.toRadians(180));

        static void convert(boolean toRed) {
            intakeSpike1Start = convertToRed(intakeSpike1Start, toRed);
            intakeSpike1End = convertToRed(intakeSpike1End, toRed);
            intakeSpike2Start = convertToRed(intakeSpike2Start, toRed);
            intakeSpike2End = convertToRed(intakeSpike2End, toRed);
            intakeSpike3Start = convertToRed(intakeSpike3Start, toRed);
            intakeSpike3End = convertToRed(intakeSpike3End, toRed);
            pushLever = convertToRed(pushLever, toRed);
            movingToIntakeSTunnelControlPoint = convertToRed(movingToIntakeSTunnelControlPoint, toRed);
            intakeFromSTunnel = convertToRed(intakeFromSTunnel, toRed);
            startSTunnelPose = convertToRed(startSTunnelPose, toRed);
            endSTunnelPose = convertToRed(endSTunnelPose, toRed);
        }
    }

    public static class ShootPoses {
        public static Pose parkShoot = new Pose(32, 12, Math.toRadians(90));
        public static Pose closeShoot = new Pose(36, 36, Math.toRadians(90));
        public static Pose midShoot = new Pose(12, 12, Math.toRadians(90));
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
