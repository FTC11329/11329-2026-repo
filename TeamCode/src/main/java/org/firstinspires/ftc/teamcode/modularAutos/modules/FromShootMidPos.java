package org.firstinspires.ftc.teamcode.modularAutos.modules;

import org.firstinspires.ftc.teamcode.modularAutos.PathPlanner;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import static org.firstinspires.ftc.teamcode.modularAutos.Common.*;

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
        Pose lastPose;

        public ToIntakeSpike1(Robot robot, PathPlanner prevPlanner, boolean sort, boolean parkAfter, boolean lever) {
            pathTimer = new Timer();
            this.robot = robot;
            this.lever = lever;
            this.sort = sort;
            this.parkAfter = parkAfter;
            lastPose = parkAfter ? ShootPoses.parkShoot : ShootPoses.midShoot;
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
        public boolean useSOTF() {
                return true;
        }

        @Override
        public void setOptimalEndPose(Pose optimalEndPose) {
            lastPose = optimalEndPose;
        }

        @Override
        public Pose getOptimalStartPose() {
            return ShootPoses.optimalSpike1Start;
        }

        //Path initialization
        PathBuilder pathChainBuilder;
        PathChain pathChain;
        Path toShootPose;
        @Override
        public void buildPaths() {
            // Path creation
            pathChainBuilder = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(startPose, IntakeBallPoses.intakeSpike1ControlPoint, IntakeBallPoses.intakeSpike1Start))
                    .setFastHeadingInterpolation(TValues.fastInterpolationIntakeStart)
                    .addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike1Start, IntakeBallPoses.intakeSpike1End));
            if (lever) {
                pathChainBuilder.addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike1End, IntakeBallPoses.pushLever));
                toShootPose = new Path(new BezierCurve(IntakeBallPoses.intakeSpike1End, IntakeBallPoses.intakeSpike1ControlPoint, lastPose));
                toShootPose.setLinearHeadingInterpolation(IntakeBallPoses.intakeSpike1End, lastPose);
            } else {
                toShootPose = new Path(new BezierLine(IntakeBallPoses.intakeSpike1End, lastPose));
            }
            pathChain = pathChainBuilder.build();
            if (parkAfter) {
                toShootPose.setTangentHeadingInterpolation();
                toShootPose.reverseHeadingInterpolation();
            } else {
                toShootPose.setFastHeadingInterpolation(TValues.fastInterpolationSpikeShootStart, TValues.fastInterpolationSpikeShootEnd, true);
            }
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
                    if (!robot.follower.isBusy() || robot.basicallyHas3() || robot.indexer.isHasBallsFull()) {
                        if (!lever) {
                            robot.follower.followPath(toShootPose);
                        }
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.indexer.isHasBallsFull() || robot.basicallyHas3() || pathTimer.getElapsedTimeSeconds() > Timings.spikeIntakeTimeOut) {
                        setPathState(3);
                    }
                    break;
                case 3:
                    // time pushing the lever
                    if (!lever || pathTimer.getElapsedTimeSeconds() > Timings.longLeverPressTime) {
                        if (lever) {
                            robot.follower.followPath(toShootPose);
                        }
                        if (sort) {
                            robot.doSmartShoot(true);
                        }
                        setPathState(4);
                    }
                case 4:
                    if ((robot.inShootingZone() || !robot.follower.isBusy()) && (parkAfter || robot.movingSlowEnoughToShoot(true))) {
                        if (parkAfter) {
                            robot.follower.setMaxPower(DrivePower.shootOnThFly);
                        }
                        if (sort) {
                            robot.indexer.setQueuedBalls(robot.getMotif());
                        } else {
                            robot.indexer.shootAll();
                        }
                        setPathState(5);
                    }
                    break;
                case 5:
                    if (pathTimer.getElapsedTimeSeconds() > Timings.unjamTimeOutFar && pathTimer.getElapsedTimeSeconds() < Timings.unjamTimeOutFar + 0.5 && !sort) {
                        robot.indexerUnjam();
                    }
                    if (!robot.isIndexerUnjamming() && robot.indexer.isHasBallsEmpty() || (sort && robot.indexer.isQueuedBallsEmpty())) {
//                        robot.setShootFromPose(null);
                        robot.follower.setMaxPower(1);
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
            return "From shoot mid to intake spike 1, state: " + state;
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
        Pose lastPose;

        public ToIntakeSpike2(Robot robot, PathPlanner prevPlanner, boolean sort, boolean parkAfter, boolean lever) {
            pathTimer = new Timer();
            this.robot = robot;
            this.lever = lever;
            this.sort = sort;
            this.parkAfter = parkAfter;
            lastPose = parkAfter ? ShootPoses.parkShoot : ShootPoses.midShoot;
            if (!prevPlanner.hasComms()) {
                startPose = getOptimalStartPose();
            } else {
                startPose = prevPlanner.getEndPoseEst();
            }
        }

        @Override
        public boolean useSOTF() {
            return true;
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
            return ShootPoses.optimalSpike2Start;
        }

        //Path initialization
        PathBuilder pathChainBuilder;
        PathChain pathChain;
        Path toShootPose;
        @Override
        public void buildPaths() {
            // Path creation
            pathChainBuilder = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(startPose, IntakeBallPoses.intakeSpike2ControlPoint, IntakeBallPoses.intakeSpike2Start))
                    .setConstantHeadingInterpolation(startPose);
            if (lever) {
                pathChainBuilder.addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike2Start, IntakeBallPoses.pushLever));
                toShootPose = new Path(new BezierCurve(IntakeBallPoses.pushLever, IntakeBallPoses.intakeSpike2ControlPoint, lastPose));
            } else {
                pathChainBuilder.addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike2Start, IntakeBallPoses.intakeSpike2End));
                toShootPose = new Path(new BezierCurve(IntakeBallPoses.intakeSpike2End, IntakeBallPoses.intakeSpike2ControlPoint, lastPose));
            }
            pathChain = pathChainBuilder.build();
            if (parkAfter) {
                toShootPose.setTangentHeadingInterpolation();
                toShootPose.reverseHeadingInterpolation();
            } else {
                toShootPose.setFastHeadingInterpolation(TValues.fastInterpolationSpikeShootStart, TValues.fastInterpolationSpikeShootEnd, true);
            }
        }

        @Override
        public Pose getEndPoseEst() {
            return lastPose;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    if (!parkAfter) {
//                        robot.setShootFromPose(lastPose);
                    }
                    robot.follower.followPath(pathChain);
                    setPathState(1);
                    break;
                case 1:
                    if (!robot.follower.isBusy() || robot.basicallyHas3() || robot.indexer.isHasBallsFull()) {
                        if (!lever) {
                            robot.follower.followPath(toShootPose);
                        }
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.indexer.isHasBallsFull() || robot.basicallyHas3() || pathTimer.getElapsedTimeSeconds() > Timings.spikeIntakeTimeOut) {
                        setPathState(3);
                    }
                    break;
                case 3:
                    // time pushing the lever
                    if (!lever || pathTimer.getElapsedTimeSeconds() > Timings.longLeverPressTime) {
                        if (lever) {
                            robot.follower.followPath(toShootPose);
                        }
                        if (sort) {
                            robot.doSmartShoot(true);
                        }
                        setPathState(4);
                    }
                case 4:
                    if ((robot.inShootingZone() || !robot.follower.isBusy()) && (parkAfter || robot.movingSlowEnoughToShoot(true))) {
                        if (parkAfter) {
                            robot.follower.setMaxPower(DrivePower.shootOnThFly);
                        }
                        if (sort) {
                            robot.doSmartShoot(true);
                            robot.indexer.setQueuedBalls(robot.getMotif());
                        } else {
                            robot.indexer.shootAll();
                        }
                        setPathState(5);
                    }
                    break;
                case 5:
                    if (pathTimer.getElapsedTimeSeconds() > Timings.unjamTimeOutFar && pathTimer.getElapsedTimeSeconds() < Timings.unjamTimeOutFar + 0.5 && !sort) {
                        robot.indexerUnjam();
                    }
                    if (!robot.isIndexerUnjamming() && robot.indexer.isHasBallsEmpty() || (sort && robot.indexer.isQueuedBallsEmpty())) {
//                        robot.setShootFromPose(null);
                        robot.follower.setMaxPower(1);
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
            return "From shoot mid to intake spike 2, state: " + state;
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
        Pose lastPose;
        public ToIntakeSpike3(Robot robot, PathPlanner prevPlanner, boolean sort, boolean parkAfter) {
            pathTimer = new Timer();
            this.robot = robot;
            this.sort = sort;
            this.parkAfter = parkAfter;
            lastPose = parkAfter ? ShootPoses.parkShoot : ShootPoses.midShoot;
            if (!prevPlanner.hasComms()) {
                startPose = getOptimalStartPose();
            } else {
                startPose = prevPlanner.getEndPoseEst();
            }
        }

        @Override
        public boolean useSOTF() {
            return true;
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
            return ShootPoses.optimalSpike3Start;
        }

        //Path initialization
        PathChain pathChain;
        Path toShootPose;
        @Override
        public void buildPaths() {
            // Path creation
            pathChain = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(startPose, IntakeBallPoses.intakeSpike3ControlPoint, IntakeBallPoses.intakeSpike3StartClose))
                    .setFastHeadingInterpolation(TValues.fastInterpolationIntakeStart)
                    .addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike3StartClose, IntakeBallPoses.intakeSpike3End))
                    .build();
            toShootPose = robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike3End, lastPose);
            if (parkAfter) {
                toShootPose.setTangentHeadingInterpolation();
                toShootPose.reverseHeadingInterpolation();
            } else {
                toShootPose.setFastHeadingInterpolation(TValues.fastInterpolationSpikeShootStart, TValues.fastInterpolationSpikeShootEnd, true);
            }
        }

        @Override
        public Pose getEndPoseEst() {
            return lastPose;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    if (!parkAfter) {
//                        robot.setShootFromPose(lastPose);
                    }
                    robot.follower.followPath(pathChain);
                    setPathState(1);
                    break;
                case 1:
                    if (!robot.follower.isBusy() || robot.basicallyHas3() || robot.indexer.isHasBallsFull()) {
                        robot.follower.followPath(toShootPose);
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.indexer.isHasBallsFull() || robot.basicallyHas3() || pathTimer.getElapsedTimeSeconds() > Timings.spikeIntakeTimeOut) {
                        if (sort) {
                            robot.doSmartShoot(true);
                        }
                        setPathState(3);
                    }
                    break;
                case 3:
                    if ((robot.inShootingZone() || !robot.follower.isBusy()) && (parkAfter || robot.movingSlowEnoughToShoot(true))) {
                        if (parkAfter) {
                            robot.follower.setMaxPower(DrivePower.shootOnThFly);
                        }
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
                    if (pathTimer.getElapsedTimeSeconds() > Timings.unjamTimeOutFar && pathTimer.getElapsedTimeSeconds() < Timings.unjamTimeOutFar + 0.5 && !sort) {
                        robot.indexerUnjam();
                    }
                    if (!robot.isIndexerUnjamming() && robot.indexer.isHasBallsEmpty() || (sort && robot.indexer.isQueuedBallsEmpty())) {
                        if (!parkAfter) {
//                            robot.setShootFromPose(null);
                        }
                        robot.follower.setMaxPower(1);
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
            return "From shoot mid to intake spike 3, state: " + state;
        }
    }

    public static class ToIntakeSpike3ToFar implements PathPlanner {
        /// intakes 3 from the second spike mark
        /// then goes to far and shoots them

        Pose offset = new Pose();
        // Variables
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private boolean sort;
        Pose lastPose;
        public ToIntakeSpike3ToFar(Robot robot, PathPlanner prevPlanner, boolean sort) {
            pathTimer = new Timer();
            this.robot = robot;
            this.sort = sort;
            lastPose = ShootPoses.farShoot;
            if (!prevPlanner.hasComms()) {
                startPose = getOptimalStartPose();
            } else {
                startPose = prevPlanner.getEndPoseEst();
            }
        }

        //Path initialization
        PathChain pathChain;
        Path toShootPose;
        @Override
        public void buildPaths() {
            // Path creation
            pathChain = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(startPose, IntakeBallPoses.intakeSpike3ControlPoint, IntakeBallPoses.intakeSpike3StartClose))
                    .setFastHeadingInterpolation(TValues.fastInterpolationIntakeStart)
                    .addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike3StartClose, IntakeBallPoses.intakeSpike3End))
                    .build();
            toShootPose = robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike3End, lastPose);
            toShootPose.setFastHeadingInterpolation(TValues.fastInterpolationSpikeShootStart, TValues.fastInterpolationSpikeShootEnd, true);
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
            return ShootPoses.optimalSpike3Start;
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
                    if (!robot.follower.isBusy() || robot.basicallyHas3() || robot.indexer.isHasBallsFull()) {
                        robot.follower.followPath(toShootPose);
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.indexer.isHasBallsFull() || robot.basicallyHas3() || pathTimer.getElapsedTimeSeconds() > Timings.spikeIntakeTimeOut) {
                        if (sort) {
                            robot.doSmartShoot(true);
                        }
                        setPathState(3);
                    }
                    break;
                case 3:
                    if ((robot.inShootingZone() || !robot.follower.isBusy()) && robot.movingSlowEnoughToShoot(false)) {
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
                    if (pathTimer.getElapsedTimeSeconds() > Timings.unjamTimeOutFar && pathTimer.getElapsedTimeSeconds() < Timings.unjamTimeOutFar + 0.5 && !sort) {
                        robot.indexerUnjam();
                    }
                    if (!robot.isIndexerUnjamming() && robot.indexer.isHasBallsEmpty() || (sort && robot.indexer.isQueuedBallsEmpty())) {
                        robot.follower.setMaxPower(1);
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
            return "From shoot mid to intake spike 3, state: " + state;
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
        private boolean longLever;
        double leverTimeOut;
        double rampTimeOut;
        Pose lastPose;

        public ToIntakeFromRamp(Robot robot, PathPlanner prevPlanner, boolean sort, boolean parkAfter, boolean longLever) {
            pathTimer = new Timer();
            this.robot = robot;
            this.sort = sort;
            this.parkAfter = parkAfter;
            this.longLever = longLever;
            if (!prevPlanner.hasComms()) {
                startPose = getOptimalStartPose();
            } else {
                startPose = prevPlanner.getEndPoseEst();
            }

            leverTimeOut = longLever ? Timings.longLeverPressTime         : Timings.shortLeverPressTime;
            rampTimeOut =  longLever ? Timings.longSTunnelIntakeTimeOut : Timings.shortSTunnelIntakeTimeOut;
            lastPose =     parkAfter ? ShootPoses.parkShoot               : ShootPoses.midShoot;
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
            return ShootPoses.optimalRampStart;
        }


        //Path initialization
        PathChain toLever;
        PathChain toIntake;
        PathBuilder toShootBuilder;
        PathChain toShootPose;
        @Override
        public void buildPaths() {
            // Path creation
            toLever = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(startPose, IntakeBallPoses.movingToPushLeverControlPoint, IntakeBallPoses.pushLever))
                    .setFastHeadingInterpolation(TValues.fastInterpolationLeverStart, TValues.fastInterpolationLeverEnd)
                    .build();

            toIntake = robot.follower.linearPathChainBuilder(IntakeBallPoses.pushLever, IntakeBallPoses.intakeFromSTunnel);

            toShootBuilder = robot.follower.pathBuilder();
            if (sort) {
                toShootBuilder.addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeFromSTunnel, IntakeBallPoses.pushLever))
                        .addPath(new BezierCurve(IntakeBallPoses.pushLever, IntakeBallPoses.movingToPushLeverControlPoint, lastPose));
            } else {
                toShootBuilder.addPath(new BezierCurve(IntakeBallPoses.intakeFromSTunnel, IntakeBallPoses.movingToPushLeverControlPoint, lastPose));
            }
            if (parkAfter) {
                toShootBuilder.setTangentHeadingInterpolation()
                        .setReversed();
            } else {
                toShootBuilder.setFastHeadingInterpolation(TValues.fastInterpolationSpikeShootStart, TValues.fastInterpolationSpikeShootEnd - 0.05, true);
            }

            toShootPose = toShootBuilder.build();
        }

        @Override
        public boolean useSOTF() {
            return true;
        }

        @Override
        public Pose getEndPoseEst() {
            return lastPose;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    if (!parkAfter) {
//                        robot.setShootFromPose(lastPose);
                    }
                    robot.follower.followPath(toLever);
                    setPathState(1);
                    break;
                case 1:
                    if (!robot.follower.isBusy()) {
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.basicallyHas3() || pathTimer.getElapsedTimeSeconds() > leverTimeOut) {
                        robot.follower.followPath(toIntake);
                        setPathState(3);
                    }
                    break;
                case 3:
                    if (robot.basicallyHas3() ||  pathTimer.getElapsedTimeSeconds() > rampTimeOut) {
                        if (sort) {
                            robot.doSmartShoot(true);
                        }
                        robot.follower.followPath(toShootPose);
                        setPathState(4);
                    }
                    break;
                case 4:
                    if ((robot.inShootingZone() || !robot.follower.isBusy()) && (parkAfter || robot.movingSlowEnoughToShoot(true))) {
                        if (sort) {
                            robot.doSmartShoot(true);
                            robot.indexer.setQueuedBalls(robot.getMotif());
                        } else {
                            robot.indexer.shootAll();
                        }
                        setPathState(5);
                    }
                    break;
                case 5://       if robot shoot all balls v                                                                                                              if timeout v
                    if (pathTimer.getElapsedTimeSeconds() > Timings.unjamTimeOutFar && pathTimer.getElapsedTimeSeconds() < Timings.unjamTimeOutFar + 0.5 && !sort) {
                        robot.indexerUnjam();
                    }
                    if (!robot.isIndexerUnjamming() && robot.indexer.isHasBallsEmpty() || (sort && robot.indexer.isQueuedBallsEmpty())) {
                        robot.follower.setMaxPower(1);
                        robot.doSmartShoot(false);
//                        robot.setShootFromPose(null);
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
            return "From shoot mid to intake ramp, state: " + state;
        }
    }

    public static class ToIntakeHuman implements PathPlanner {
        /// Intakes from around the human player zone generally
        // Variables
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private Pose lastPose;
        private boolean sort;
        private boolean parkAfter;
        public ToIntakeHuman(Robot robot, PathPlanner prevPlanner, boolean sort, boolean parkAfter) {
            pathTimer = new Timer();
            this.robot = robot;
            this.lastPose = parkAfter ? ShootPoses.parkShoot : ShootPoses.midShoot;
            this.sort = sort;
            this.parkAfter = parkAfter;
            if (!prevPlanner.hasComms()) {
                startPose = getOptimalStartPose();
            } else {
                startPose = prevPlanner.getEndPoseEst();
            }
        }

        @Override
        public boolean useSOTF() {
            return true;
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
            return ShootPoses.optimalHumanPlayerStart;
        }

        @Override
        public Pose getEndPoseEst() {
            return lastPose;
        }

        //Path initialization
        PathChain toIntakeBalls;
        PathChain toShootPose;

        @Override
        public void buildPaths() {
            toIntakeBalls = robot.follower.pathBuilder()
                    .addPath(robot.follower.fastPathBuilder(startPose, IntakeBallPoses.intakeHumanDiagonal, TValues.fastInterpolationIntakeStart))
                    .addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeHumanDiagonal, IntakeBallPoses.intakeHumanDiagonalToStrait))
                    .build();
            if (parkAfter) {
                toShootPose = robot.follower.pathBuilder()
                        .addPath(new BezierLine(IntakeBallPoses.intakeHumanDiagonalToStrait, lastPose))
                        .setReversed()
                        .build();
            } else {
                toShootPose = robot.follower.fastPathChainBuilder(IntakeBallPoses.intakeHumanDiagonalToStrait, lastPose, TValues.fastInterpolationSpikeShootStart, TValues.fastInterpolationSpikeShootStart, true);
            }
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(toIntakeBalls);
                    setPathState(1);
                    break;
                case 1:
                    if (!robot.follower.isBusy() || robot.basicallyHas3() || robot.indexer.isHasBallsFull() || robot.follower.getVelocity().getMagnitude() < 5 && pathTimer.getElapsedTimeSeconds() > 0.75) {
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                        robot.follower.followPath(toShootPose);
                        setPathState(3);
                    }
                    break;
                case 3:
                    if ((robot.inShootingZone() || !robot.follower.isBusy()) && (parkAfter || robot.movingSlowEnoughToShoot(true))) {
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
                    if (pathTimer.getElapsedTimeSeconds() > Timings.unjamTimeOutFar && pathTimer.getElapsedTimeSeconds() < Timings.unjamTimeOutFar + 0.5 && !sort) {
                        robot.indexerUnjam();
                    }
                    if (robot.indexer.isHasBallsEmpty() || (sort && robot.indexer.isQueuedBallsEmpty())) {
                        robot.follower.setMaxPower(1);
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
            return "To Intake Human, state: " + state;
        }
    }

    public static class ToIntakeHumanWVision implements PathPlanner {
        /// Intakes from around the human player with vision
        // Variables
        private Timer pathTimer;
        private int state = -1;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private Pose lastPose;
        private boolean sort;
        private boolean parkAfter;
        private boolean visionFail;
        PathPlanner failSafePath;
        public ToIntakeHumanWVision(Robot robot, PathPlanner prevPlanner, boolean sort, boolean parkAfter) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = prevPlanner.getEndPoseEst();
            this.lastPose = parkAfter ? ShootPoses.parkShoot : ShootPoses.midShoot;
            this.sort = sort;
            this.parkAfter = parkAfter;
            if (!prevPlanner.hasComms()) {
                startPose = getOptimalStartPose();
            } else {
                startPose = prevPlanner.getEndPoseEst();
            }
            failSafePath = new FromShootFarPos.ToIntakeHuman(robot, prevPlanner, sort);
            failSafePath.buildPaths();
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
            return ShootPoses.optimalVisionStart;
        }

        @Override
        public Pose getEndPoseEst() {
            return lastPose;
        }

        //Path initialization
        PathChain toIntakeBalls;
        PathChain toShootPose;

        @Override
        public void buildPaths() {
            // We create these on the fly during runtime
        }

        Pose intakeBallPose;
        @Override
        public boolean run() {
            if (visionFail) {
                return failSafePath.run();
            }
            switch (state) {
                case -1:
                    setPathState(0);
                    break;
                case 0:
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        visionFail = true;
                        robot.follower.breakFollowing();
                        return false;
                    }

                    intakeBallPose = robot.getIntakeBallPoseFromCam();
                    if (intakeBallPose != null) {
                        double dx = intakeBallPose.getX() - startPose.getX();
                        double dy = intakeBallPose.getY() - startPose.getY();
                        double headingRadians = Math.atan2(dy, dx);

                        intakeBallPose = intakeBallPose.setHeading(headingRadians);
                        toIntakeBalls = robot.follower.linearPathChainBuilder(startPose, intakeBallPose);
                        toIntakeBalls.getPath(0).setTangentHeadingInterpolation();
                        robot.follower.followPath(toIntakeBalls);
                        setPathState(1);
                    }
                    break;
                case 1:
                    if (!robot.follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3 || robot.indexer.isHasBallsFull() || robot.basicallyHas3() || robot.follower.getVelocity().getMagnitude() < 2 && pathTimer.getElapsedTimeSeconds() > 0.75) {
                        toShootPose = robot.follower.fastPathChainBuilder(intakeBallPose, lastPose, TValues.fastInterpolationSpikeShootStart, TValues.fastInterpolationSpikeShootEnd, true);

                        robot.follower.followPath(toShootPose);
                    }
                    break;
                case 2:
                    if ((robot.inShootingZone() || !robot.follower.isBusy()) && (parkAfter || robot.movingSlowEnoughToShoot(true))) {
                        if (sort) {
                            robot.doSmartShoot();
                            robot.indexer.setQueuedBalls(robot.getMotif());
                        } else {
                            robot.indexer.shootAll();
                        }
                        setPathState(5);
                    }
                    break;
                case 3:
                    if (pathTimer.getElapsedTimeSeconds() > Timings.unjamTimeOutFar && pathTimer.getElapsedTimeSeconds() < Timings.unjamTimeOutFar + 0.5 && !sort) {
                        robot.indexerUnjam();
                    }
                    if (!robot.isIndexerUnjamming() && robot.indexer.isHasBallsEmpty() || (sort && robot.indexer.isQueuedBallsEmpty())) {
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
            return "From Mid Shoot PosTo Intake Human W Vision, state: " + state;
        }
    }
}