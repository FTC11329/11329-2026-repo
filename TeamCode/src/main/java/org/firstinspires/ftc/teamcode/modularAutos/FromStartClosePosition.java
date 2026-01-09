package org.firstinspires.ftc.teamcode.modularAutos;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class FromStartClosePosition {
    // allows for either close inner or outer startPositions
    public static class ShootAndGoToMidShootPos implements PathPlanner{
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;

        public ShootAndGoToMidShootPos(Robot robot, Pose startPose) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
        }

        @Override
        public void buildPaths() {

        }

        @Override
        public boolean run() {
            return false;
        }

        @Override
        public Pose getEndPoseEst() {
            return null;
        }
    }
}
