package org.firstinspires.ftc.teamcode.modularAutos.modules;


import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.modularAutos.Common.*;
import org.firstinspires.ftc.teamcode.modularAutos.PathPlanner;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class FromStartFarPos {

    public static class ShootPreloads implements PathPlanner {
        /// shoots preloads

        Pose offset = new Pose();
        // Variables
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private boolean sort;
        private Pose lastPose;

        public ShootPreloads(Robot robot, PathPlanner prevPlanner, boolean sort) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = prevPlanner.getEndPoseEst();
            this.sort = sort;
            lastPose = ShootPoses.farShoot;
        }

        @Override
        public boolean hasComms() {
            return true;
        }

        @Override
        public void setOptimalEndPose(Pose optimalEndPose) {
            lastPose = optimalEndPose;
        }

        @Override
        public Pose getOptimalStartPose() {
            return StartPoses.far;
        }


        //Path initialization
        PathChain toShootPose;

        @Override
        public void buildPaths() {
            // Path creation
            toShootPose = robot.follower.linearPathChainBuilder(startPose, lastPose);
        }

        @Override
        public Pose getEndPoseEst() {
            return ShootPoses.farShoot;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(toShootPose);
                    setPathState(1);
                    break;
                case 1:
                    if (robot.readyToShootMotors()) {
                        setPathState(2);
                    }
                    break;
                case 2:
                    if ((robot.inShootingZone() || !robot.follower.isBusy()) && robot.movingSlowEnoughToShoot(false)) {
                        if (sort) {
                            robot.doSmartShoot(true);
                            robot.indexer.setQueueGivenAttemptedRampOrder(robot.getMotif());
                        } else {
                            robot.indexer.shootAll();
                        }
                        setPathState(3);
                    }
                    break;
                case 3:
                    if (!robot.isIndexerUnjamming() && (robot.indexer.isHasBallsEmpty() || robot.indexer.autoFastEnd()) || (sort && robot.indexer.isQueuedBallsEmpty())) {
                        robot.doSmartShoot(false);
                        isFinished = true;
                    }
                    if (pathTimer.getElapsedTimeSeconds() > (!sort ? Timings.unjamTimeOutFar : Timings.unjamTimeOutFarSort)) {
                        robot.indexerUnjam();
                    }
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
            return "From start Far to shoot preloads, state: " + state;
        }
    }
}