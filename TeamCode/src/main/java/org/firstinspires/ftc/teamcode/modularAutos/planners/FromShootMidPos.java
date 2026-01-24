package org.firstinspires.ftc.teamcode.modularAutos.planners;

import org.firstinspires.ftc.teamcode.modularAutos.PathPlanner;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.BallColor;

import static org.firstinspires.ftc.teamcode.modularAutos.CommonPoses.*;

import androidx.annotation.NonNull;

public class FromShootMidPos {
    public static class ToIntakeSpike1 implements PathPlanner {
        /// intakes 3 from the close spike mark
        /// then goes back and shoots them
        // Variables
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private boolean lever;
        private boolean sort;
        private boolean parkAfter;

        public ToIntakeSpike1(Robot robot, Pose startPose, boolean lever, boolean sort, boolean parkAfter) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.lever = lever;
            this.sort = sort;
            this.parkAfter = parkAfter;
        }

        //Path initialization
        PathBuilder pathChainBuilder;
        PathChain pathChain;
        Pose lastPose;
        @Override
        public void buildPaths() {
            lastPose = parkAfter ? ShootPoses.parkShoot : ShootPoses.midShoot;
            // Path creation
            pathChainBuilder = robot.follower.pathBuilder()
                    .addPath(robot.follower.linearPathBuilder(startPose, IntakeBallPoses.intakeSpike1Start))
                    .addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike1Start, IntakeBallPoses.intakeSpike1End));
            if (lever) {
                pathChainBuilder.addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike1End, IntakeBallPoses.pushLever));
                pathChainBuilder.addPath(robot.follower.linearPathBuilder(IntakeBallPoses.pushLever, lastPose));
            } else {

                pathChainBuilder.addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike1End, lastPose));
            }
            pathChain = pathChainBuilder.build();
        }

        @Override
        public Pose getEndPoseEst() {
            return lastPose;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(pathChain);
                    setPathState(1);
                    break;
                case 1:
                    if (robot.inShootingZone()) {
                        robot.indexer.setHasBalls(new BallColor[]{BallColor.Purple, BallColor.Purple, BallColor.Green});
                        if (sort) {
                            robot.doSmartShoot(true);
                            robot.indexer.setQueuedBalls(robot.getMotif());
                        } else {
                            robot.indexer.shootAll();
                        }
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.indexer.isHasBallsEmpty() || (sort && robot.indexer.isQueuedBallsEmpty())) {
                        robot.doSmartShoot(false);
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
        /// then goes back and shoots them

        Pose offset = new Pose();
        // Variables
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private boolean lever;
        private boolean sort;
        private boolean parkAfter;

        public ToIntakeSpike2(Robot robot, Pose startPose, boolean lever, boolean sort, boolean parkAfter) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.lever = lever;
            this.sort = sort;
            this.parkAfter = parkAfter;
        }

        //Path initialization
        PathBuilder pathChainBuilder;
        PathChain pathChain;
        Pose lastPose;
        @Override
        public void buildPaths() {
            lastPose = parkAfter ? ShootPoses.parkShoot : ShootPoses.midShoot;
            // Path creation
            pathChainBuilder = robot.follower.pathBuilder()
                    .addPath(robot.follower.linearPathBuilder(startPose, IntakeBallPoses.intakeSpike2Start))
                    .addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike2Start, IntakeBallPoses.intakeSpike2End));
            if (lever) {
                pathChainBuilder.addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike2End, IntakeBallPoses.pushLever));
                pathChainBuilder.addPath(robot.follower.linearPathBuilder(IntakeBallPoses.pushLever, lastPose));
            } else {

                pathChainBuilder.addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike2End, lastPose));
            }
            pathChain = pathChainBuilder.build();
        }

        @Override
        public Pose getEndPoseEst() {
            return lastPose;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(pathChain);
                    setPathState(1);
                    break;
                case 1:
                    if (robot.inShootingZone()) {
                        robot.indexer.setHasBalls(new BallColor[]{BallColor.Purple, BallColor.Green, BallColor.Purple});
                        if (sort) {
                            robot.doSmartShoot(true);
                            robot.indexer.setQueuedBalls(robot.getMotif());
                        } else {
                            robot.indexer.shootAll();
                        }
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.indexer.isHasBallsEmpty() || (sort && robot.indexer.isQueuedBallsEmpty())) {
                        robot.doSmartShoot(false);
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
        /// then goes back and shoots them

        Pose offset = new Pose();
        // Variables
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private boolean sort;
        private boolean parkAfter;
        public ToIntakeSpike3(Robot robot, Pose startPose, boolean sort, boolean parkAfter) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.sort = sort;
            this.parkAfter = parkAfter;
        }

        //Path initialization
        PathBuilder pathChainBuilder;
        PathChain pathChain;
        Pose lastPose;
        @Override
        public void buildPaths() {
            lastPose = parkAfter ? ShootPoses.parkShoot : ShootPoses.midShoot;
            // Path creation
            pathChainBuilder = robot.follower.pathBuilder()
                    .addPath(robot.follower.linearPathBuilder(startPose, IntakeBallPoses.intakeSpike3Start))
                    .addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike3Start, IntakeBallPoses.intakeSpike3End))
                    .addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike3End, lastPose));
            pathChain = pathChainBuilder.build();
        }

        @Override
        public Pose getEndPoseEst() {
            return lastPose;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(pathChain);
                    setPathState(1);
                    break;
                case 1:
                    if (robot.inShootingZone()) {
                        robot.indexer.setHasBalls(new BallColor[]{BallColor.Green, BallColor.Purple, BallColor.Purple});
                        if (sort) {
                            robot.doSmartShoot(true);
                            robot.indexer.setQueuedBalls(robot.getMotif());
                        } else {
                            robot.indexer.shootAll();
                        }
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.indexer.isHasBallsEmpty() || (sort && robot.indexer.isQueuedBallsEmpty())) {
                        robot.doSmartShoot(false);
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

    public static class ToIntakeFromRamp implements PathPlanner {
        /// intakes 3 from the ramp
        /// then goes back and shoots them

        // Variables
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private boolean sort;
        private boolean parkAfter;

        public ToIntakeFromRamp(Robot robot, Pose startPose, boolean sort, boolean parkAfter) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.sort = sort;
            this.parkAfter = parkAfter;
        }

        //Path initialization
        PathChain pathChain;
        Path toShootPose;
        Pose lastPose;
        @Override
        public void buildPaths() {
            lastPose = parkAfter ? ShootPoses.parkShoot : ShootPoses.midShoot;
            // Path creation
            pathChain = robot.follower.pathBuilder()
                    .addPath(robot.follower.linearPathBuilder(startPose, IntakeBallPoses.pushLever))
                    .addPath(new BezierCurve(IntakeBallPoses.pushLever, IntakeBallPoses.movingToIntakeSTunnelControlPoint, IntakeBallPoses.intakeFromSTunnel))
                    .build();
            toShootPose = robot.follower.linearPathBuilder(IntakeBallPoses.intakeFromSTunnel, lastPose);
        }

        @Override
        public Pose getEndPoseEst() {
            return lastPose;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(pathChain);
                    setPathState(1);
                    break;
                case 1:
                    if (!robot.follower.isBusy()) {
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.indexer.numberOfBallsInBallCells() >= 2 || pathTimer.getElapsedTimeSeconds() > 1) {
                        robot.follower.followPath(toShootPose);
                        setPathState(3);
                    }
                    break;
                case 3:
                    if (robot.inShootingZone()) {
                        if (sort) {
                            robot.doSmartShoot(true);
                            robot.indexer.setQueuedBalls(robot.getMotif());
                        } else {
                            robot.indexer.shootAll();
                        }
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (robot.indexer.isHasBallsEmpty() || (sort && robot.indexer.isQueuedBallsEmpty())) {
                        robot.doSmartShoot(false);
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

}