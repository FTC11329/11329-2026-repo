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
import org.firstinspires.ftc.teamcode.util.RobotSide;
import org.opencv.core.Mat;

public class FromStartClosePos {
    // allows for either close inner or outer startPositions
    @Deprecated
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
        private Pose lastPose;

        public ShootAndGoToMidShootPos(Robot robot, PathPlanner prevPlanner) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = prevPlanner.getEndPoseEst();
            this.lastPose = ShootPoses.midShoot;
        }

        @Override
        public boolean hasComms() {
            return true;
        }

        @Override
        public boolean useSOTF() {
            return false;
        }

        @Override
        public void setOptimalEndPose(Pose optimalEndPose) {
            lastPose = optimalEndPose;
        }

        @Override
        public Pose getOptimalStartPose() {
            return StartPoses.closeOuter;
        }

        //Path initialization
        PathChain toShootPosition;

        @Override
        public void buildPaths() {
            // Path creation
            toShootPosition = robot.follower.fastPathChainBuilder(startPose, lastPose, TValues.fastInterpolationPreloadStart, TValues.fastInterpolationPreloadEnd, true);
        }

        @Override
        public Pose getEndPoseEst() {
            return lastPose;
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
                        robot.indexer.shootAll();
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (pathTimer.getElapsedTimeSeconds() > Timings.unjamTimeOutFar && pathTimer.getElapsedTimeSeconds() < Timings.unjamTimeOutFar + 0.5) {
                        robot.indexerUnjam();
                    }
                    if ((robot.indexer.isHasBallsEmpty())) {
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
    /// TIMES  3.64 - 0.84
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
            this.startPose = prevPlanner.getEndPoseEst();
            this.lastPose = ShootPoses.midShoot;
        }

        @Override
        public boolean hasComms() {
            return true;
        }

        @Override
        public boolean useSOTF() {
            return true;
        }

        @Override
        public void setOptimalEndPose(Pose optimalEndPose) {
            lastPose = optimalEndPose;
        }

        @Override
        public Pose getOptimalStartPose() {
            return StartPoses.closeOuter;
        }


        //Path initialization
        PathChain toShootPosition;

        @Override
        public void buildPaths() {
            // Path creation
            toShootPosition = robot.follower.fastPathChainBuilder(startPose, lastPose, TValues.fastInterpolationPreloadStart, TValues.fastInterpolationPreloadEnd, true);
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
                    if (robot.follower.getCurrentTValue() > 0.12) {
                        robot.follower.setMaxPower(DrivePower.shootOnThFly);
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.readyToShootMotors()) {
                        robot.indexer.shootAll();
                        setPathState(3);
                    }
                    break;
                case 3:
                    if (!robot.follower.isBusy() || (robot.indexer.isHasBallsEmpty())) {
                        robot.follower.setMaxPower(1);
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (pathTimer.getElapsedTimeSeconds() > Timings.unjamTimeOut) {
                        robot.indexerUnjam();
                    }
                    if ((robot.indexer.isHasBallsEmpty()) && robot.follower.getErrorDistance(lastPose) < 200) {
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
