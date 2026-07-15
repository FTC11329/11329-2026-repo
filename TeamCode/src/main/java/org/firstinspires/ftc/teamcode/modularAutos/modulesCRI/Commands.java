package org.firstinspires.ftc.teamcode.modularAutos.modulesCRI;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.modularAutos.PathPlanner;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;


public class Commands {
    public static class WaitSeconds implements PathPlanner {
        /// Waits the seconds passed in
        // Variables
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private double waitSec;
        public WaitSeconds(Robot robot, PathPlanner prevPlanner) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = prevPlanner.getEndPoseEst();
            this.waitSec = waitSec;
        }

        @Override
        public void buildPaths() {
        }

        @Override
        public Pose getEndPoseEst() {
            return startPose;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    setPathState(1);
                    break;
                case 1:
                    if (pathTimer.getElapsedTimeSeconds() > waitSec) {
                        isFinished = true;
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
            return "Wait Time: " + waitSec;
        }
    }
    public static class WaitUntilRemainingTime implements PathPlanner {
        /// Waits until time remaing in auto
        // Variables
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private double endTime;
        public WaitUntilRemainingTime(Robot robot, PathPlanner prevPlanner, double endTime) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = prevPlanner.getEndPoseEst();
            this.endTime = endTime;
        }

        @Override
        public void buildPaths() {
        }

        @Override
        public Pose getEndPoseEst() {
            return startPose;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    setPathState(1);
                    break;
                case 1:
                    if (robot.getOpmodeTimeSeconds() > endTime) {
                        isFinished = true;
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
            return "Wait Until Remaining Time: " + endTime;
        }
    }
    public static class goToPose implements PathPlanner {
        /// goes to pose
        // Variables
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private Pose endPose;
        public goToPose(Robot robot, PathPlanner prevPlanner, Pose endPose) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = prevPlanner.getEndPoseEst();
            this.endPose = endPose;
        }

        PathChain toPose;
        @Override
        public void buildPaths() {
            toPose = robot.follower.linearPathChainBuilder(startPose, endPose);
        }

        @Override
        public Pose getEndPoseEst() {
            return new Pose(0,0, Math.toRadians(-90));
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    setPathState(1);
                    robot.follower.followPath(toPose);
                    break;
                case 1:
                    if (!robot.follower.isBusy()) {
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (pathTimer.getElapsedTimeSeconds() > 2) {
                        isFinished = true;
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
            return "to Pose " + endPose;
        }
    }

    public static class nullPlanner implements PathPlanner {

        private Pose lastPose;

        public nullPlanner(Pose lastPose) {
            this.lastPose = lastPose;
        }

        @Override
        public void buildPaths() {}

        @Override
        public boolean run() {
            return false;
        }

        @Override
        public Pose getEndPoseEst() {
            return lastPose;
        }

        @Override
        public boolean hasComms() {
            return true;
        }

        @NonNull
        @Override
        public String toString() {
            return "Null Planner";
        }
    }
}
