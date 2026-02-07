package org.firstinspires.ftc.teamcode.modularAutos.modules;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.modularAutos.PathPlanner;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
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
        public WaitSeconds(Robot robot, Pose startPose, double waitSec) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
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
        public WaitUntilRemainingTime(Robot robot, Pose startPose, double endTime) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
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
}
