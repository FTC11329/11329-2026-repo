package org.firstinspires.ftc.teamcode.modularAutos.planners;

import org.firstinspires.ftc.teamcode.modularAutos.PathPlanner;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import static org.firstinspires.ftc.teamcode.modularAutos.CommonPoses.*;

import androidx.annotation.NonNull;

public class FromShootMidPos {
    public static class ToIntakeSpike1 implements PathPlanner {
        /// intakes 3 from the close spike mark
        /// ends as soon as the robot intakes 2 so you need to continue intaking for a little
        // Variables
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;

        public ToIntakeSpike1(Robot robot, Pose startPose) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
        }

        //Path initialization
        PathChain pathChain;

        @Override
        public void buildPaths() {
            // Path creation
            pathChain = robot.follower.pathBuilder()
                    .addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike1Start, IntakeBallPoses.intakeSpike1End))
                    .addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike1End, ShootPoses.midShoot))
                    .build();
        }

        @Override
        public Pose getEndPoseEst() {
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
        public String toString() {
            return "NAME";
        }
    }
}