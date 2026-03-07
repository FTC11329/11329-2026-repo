package org.firstinspires.ftc.teamcode.modularAutos.modules;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.modularAutos.Common;
import org.firstinspires.ftc.teamcode.modularAutos.Common.*;
import org.firstinspires.ftc.teamcode.modularAutos.PathPlanner;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class FromStartClosePos {
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
        private boolean prevHasComms;

        public ShootAndGoToMidShootPos(Robot robot, PathPlanner prevPlanner) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = prevPlanner.getEndPoseEst();
            prevHasComms = prevPlanner.hasComms();
        }

        //Path initialization
        PathChain toShootPosition;

        @Override
        public void buildPaths() {
            // Path creation
            toShootPosition = robot.follower.linearPathChainBuilder(startPose, ShootPoses.midShoot, 0.7);
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
                    if (!robot.follower.isBusy()) {
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                        robot.indexer.shootAll();
                        setPathState(3);
                    }
                    break;
                case 3:
                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                        robot.indexerUnjam();
                    }
                    if (robot.indexer.isHasBallsEmpty()) {
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
    public static class ShootAndGoToMidShootPosFast implements PathPlanner {
        /// Starts Close and goes to shoot mid pos while shooting
        // Variables
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private Pose lastPose;

        public ShootAndGoToMidShootPosFast(Robot robot, PathPlanner prevPlanner) {
            pathTimer = new Timer();
            this.robot = robot;
            this.lastPose = ShootPoses.midShoot;

            if (!prevPlanner.hasComms()) {
                startPose = getOptimalStartPose();
            } else {
                startPose = prevPlanner.getEndPoseEst();
            }
        }

        @Override
        public boolean hasComms() {
            return true;
        }

        @Override
        public void setOptimalEndPose(Pose optimalEndPose) {
            lastPose = optimalEndPose;
        }

        //Path initialization
        PathChain shootOnTheFlyPath;

        @Override
        public void buildPaths() {
            if (startPose == null) {
                throw new RuntimeException("start");
            }
            if (lastPose == null) {
                throw new RuntimeException("end");
            }
            // Path creation
            shootOnTheFlyPath = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(startPose, ShootPoses.fromStartCloseToMidShootControl, lastPose))
                    .setLinearHeadingInterpolation(startPose, lastPose)
                    .build();
        }

        @Override
        public Pose getEndPoseEst() {
            return lastPose;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.setMaxPower(DrivePower.shootOnThFly);
                    robot.follower.followPath(shootOnTheFlyPath);
                    setPathState(1);
                    break;
                case 1:
                    if (pathTimer.getElapsedTimeSeconds() > 3) {
                        robot.indexerUnjam();
                    }
                    if (robot.indexer.isHasBallsEmpty() && !robot.follower.isBusy()) {
                        robot.follower.setMaxPower(1);
                        isFinished = true;
                    } else if (robot.indexer.isHasBallsEmpty()) {
                        robot.follower.setMaxPower(1);
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
            return "FromStartClosePosition ShootAndGoToMidShootPosFast, step: " + state;
        }
    }
}
