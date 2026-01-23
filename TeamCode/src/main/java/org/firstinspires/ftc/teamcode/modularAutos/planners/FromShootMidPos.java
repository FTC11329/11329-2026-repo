package org.firstinspires.ftc.teamcode.modularAutos.planners;

import org.firstinspires.ftc.teamcode.modularAutos.PathPlanner;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import static org.firstinspires.ftc.teamcode.modularAutos.CommonPoses.*;

import androidx.annotation.NonNull;

public class FromShootMidPos {
    public static class ToIntakeSpike1 implements PathPlanner {
        /// intakes 3 from the close spike mark
        // Variables
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private boolean lever;

        public ToIntakeSpike1(Robot robot, Pose startPose, boolean lever) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.lever = lever;
        }

        //Path initialization
        PathBuilder pathChainBuilder;
        PathChain pathChain;
        @Override
        public void buildPaths() {
            // Path creation
            pathChainBuilder = robot.follower.pathBuilder()
                    .addPath(robot.follower.linearPathBuilder(startPose, IntakeBallPoses.intakeSpike1Start))
                    .addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike1Start, IntakeBallPoses.intakeSpike1End));
            if (lever) {
                pathChainBuilder.addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike1End, IntakeBallPoses.pushLever));
                pathChainBuilder.addPath(robot.follower.linearPathBuilder(IntakeBallPoses.pushLever, ShootPoses.midShoot));
            } else {

                pathChainBuilder.addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike1End, ShootPoses.midShoot));
            }
            pathChain = pathChainBuilder.build();
        }

        @Override
        public Pose getEndPoseEst() {
            return ShootPoses.midShoot;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.spinIntake();
                    robot.follower.followPath(pathChain);
                    setPathState(1);
                    break;
                case 1:
                    if (robot.inShootingZone()) {
                        robot.indexer.shootAll();
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.indexer.isHasBallsEmpty()) {
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
            return "From shoot mid to intake spike 1";
        }
    }
    public static class ToIntakeSpike2 implements PathPlanner {
        /// intakes 3 from the second spike mark

        Pose offset = new Pose();
        // Variables
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private boolean lever;

        public ToIntakeSpike2(Robot robot, Pose startPose, boolean lever) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.lever = lever;
        }

        //Path initialization
        PathBuilder pathChainBuilder;
        PathChain pathChain;
        @Override
        public void buildPaths() {
            // Path creation
            pathChainBuilder = robot.follower.pathBuilder()
                    .addPath(robot.follower.linearPathBuilder(startPose, IntakeBallPoses.intakeSpike2Start))
                    .addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike2Start, IntakeBallPoses.intakeSpike2End));
            if (lever) {
                pathChainBuilder.addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike2End, IntakeBallPoses.pushLever));
                pathChainBuilder.addPath(robot.follower.linearPathBuilder(IntakeBallPoses.pushLever, ShootPoses.midShoot));
            } else {

                pathChainBuilder.addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike2End, ShootPoses.midShoot));
            }
            pathChain = pathChainBuilder.build();
        }

        @Override
        public Pose getEndPoseEst() {
            return ShootPoses.midShoot;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.spinIntake();
                    robot.follower.followPath(pathChain);
                    setPathState(1);
                    break;
                case 1:
                    if (robot.inShootingZone()) {
                        robot.indexer.shootAll();
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.indexer.isHasBallsEmpty()) {
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
            return "From shoot mid to intake spike 2";
        }
    }
    public static class ToIntakeSpike3 implements PathPlanner {
        /// intakes 3 from the second spike mark

        Pose offset = new Pose();
        // Variables
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        public ToIntakeSpike3(Robot robot, Pose startPose) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
        }

        //Path initialization
        PathBuilder pathChainBuilder;
        PathChain pathChain;
        @Override
        public void buildPaths() {
            // Path creation
            pathChainBuilder = robot.follower.pathBuilder()
                    .addPath(robot.follower.linearPathBuilder(startPose, IntakeBallPoses.intakeSpike3Start))
                    .addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike3Start, IntakeBallPoses.intakeSpike3End))
                    .addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike3End, ShootPoses.midShoot));
            pathChain = pathChainBuilder.build();
        }

        @Override
        public Pose getEndPoseEst() {
            return ShootPoses.midShoot;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.spinIntake();
                    robot.follower.followPath(pathChain);
                    setPathState(1);
                    break;
                case 1:
                    if (robot.inShootingZone()) {
                        robot.indexer.shootAll();
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.indexer.isHasBallsEmpty()) {
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
            return "From shoot mid to intake spike 3";
        }
    }

}