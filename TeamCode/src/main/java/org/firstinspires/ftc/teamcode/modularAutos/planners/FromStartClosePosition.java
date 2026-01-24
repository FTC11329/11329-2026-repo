package org.firstinspires.ftc.teamcode.modularAutos.planners;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.modularAutos.CommonPoses.*;
import org.firstinspires.ftc.teamcode.modularAutos.PathPlanner;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class FromStartClosePosition {
    // allows for either close inner or outer startPositions
    public static class ShootAndGoToMidShootPos implements PathPlanner {
        /// Starts Close and goes to shoot mid pos while shooting
        // Variables
        Pose offset = new Pose();
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

        //Path initialization
        Path toShootPosition;

        @Override
        public void buildPaths() {
            // Path creation
            toShootPosition = robot.follower.linearPathBuilder(startPose, ShootPoses.midShoot);
        }

        @Override
        public Pose getEndPoseEst() {
            return ShootPoses.midShoot;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(toShootPosition);
                    setPathState(1);
                    break;
                case 1:
                    if (robot.follower.getPose().getX() < 48) {
                        robot.indexer.shootAll();
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.indexer.isHasBallsEmpty() && robot.follower.getCurrentTValue() > 0.75) {
                        isFinished = true;
                    }
                    break;
            }

            return isFinished;
        }

        private void setPathState(int state) {
            this.state = state;
            pathTimer.resetTimer();
        }

        @NonNull
        @Override
        public String toString() {
            return "FromStartClosePosition ShootAndGoToMidShootPos, step: " + state;
        }
    }
}
