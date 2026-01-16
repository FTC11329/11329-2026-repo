package org.firstinspires.ftc.teamcode.modularAutos.planners;

import org.firstinspires.ftc.teamcode.modularAutos.PathPlanner;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import static org.firstinspires.ftc.teamcode.modularAutos.CommonPoses.*;

import androidx.annotation.NonNull;


public class Example {
    public static class NAME implements PathPlanner {
        //todo
        /// DESCRIPTION
        // Variables
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        // todo
        public NAME(Robot robot, Pose startPose) {
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
            //todo
            return ShootPoses.midShoot;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    setPathState(1);
                case 1:
            }

            return isFinished;
        }

        private void setPathState(int state) {
            this.state = state;
            pathTimer.resetTimer();
        }

        @NonNull
        @Override
        //todo
        public String toString() {
            return "NAME";
        }
    }
}
