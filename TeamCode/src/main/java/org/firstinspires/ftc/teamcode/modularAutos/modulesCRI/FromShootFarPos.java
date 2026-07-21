package org.firstinspires.ftc.teamcode.modularAutos.modulesCRI;

import static org.firstinspires.ftc.teamcode.modularAutos.CommonCRI.*;
import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.modularAutos.PathPlanner;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.VisionTypes;

public class FromShootFarPos {

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
        Pose lastPose;

        public ToIntakeSpike2(Robot robot, PathPlanner prevPlanner, boolean sort, boolean lever) {
            pathTimer = new Timer();
            this.robot = robot;
            this.lever = lever;
            this.sort = sort;
            lastPose = ShootPoses.farShoot;
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

        @Override
        public Pose getOptimalStartPose() {
            return ShootPoses.optimalSpike2StartFar;
        }

        //Path initialization
        PathBuilder pathChainBuilder;
        PathChain pathChain;
        Path toShootPose;
        @Override
        public void buildPaths() {
            // Path creation
            pathChainBuilder = robot.follower.pathBuilder()
                    .addPath(robot.follower.fastPathBuilder(startPose, IntakeBallPoses.intakeSpike2Start, TValues.fastInterpolationIntakeStartFar, TValues.fastInterpolationIntakeEndFar));
            if (lever) {
                pathChainBuilder.addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike2Start, IntakeBallPoses.pushLever));
                toShootPose = new Path(new BezierLine(IntakeBallPoses.pushLever, lastPose));
            } else {
                pathChainBuilder.addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike2Start, IntakeBallPoses.intakeSpike2End));
                toShootPose = new Path(new BezierLine(IntakeBallPoses.intakeSpike2End, lastPose));
            }
            pathChain = pathChainBuilder.build();
            toShootPose.setFastHeadingInterpolation(TValues.fastInterpolationSpikeShootStart, TValues.fastInterpolationSpikeShootEnd, true);
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
                    if (sort) {
                        robot.doSmartShoot(true);
                    }
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
                        setPathState(4);
                    }
                case 4:
                    if ((robot.inShootingZone() || !robot.follower.isBusy()) && robot.movingSlowEnoughToShoot(false)) {
                        if (sort) {
                            robot.indexer.setQueueGivenAttemptedRampOrder(robot.getMotif());
                        } else {
                            robot.indexer.shootAll();
                        }
                        setPathState(5);
                    }
                    break;
                case 5:
                    if (! robot.isIndexerUnjamming() && (robot.indexer.isHasBallsEmpty()) || (sort && robot.indexer.isQueuedBallsEmpty())) {
//                        robot.setShootFromPose(null);
                        robot.follower.setMaxPower(1);
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
            return "From shoot far to intake spike 2, state: " + state;
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
        private Pose lastPose;
        private boolean sort;
        public ToIntakeSpike3(Robot robot, PathPlanner prevPlanner, boolean sort) {
            pathTimer = new Timer();
            this.robot = robot;
            this.sort = sort;
            if (!prevPlanner.hasComms()) {
                startPose = getOptimalStartPose();
            } else {
                startPose = prevPlanner.getEndPoseEst();
            }

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
            return ShootPoses.optimalSpike3StartFar;
        }

        //Path initialization
        PathChain toIntakeSpike3;
        PathChain toShootPose;
        @Override
        public void buildPaths() {
            // Path creation
            toIntakeSpike3 = robot.follower.pathBuilder()
                    .addPath(robot.follower.linearPathBuilder(startPose, IntakeBallPoses.intakeSpike3StartFar))
                    .addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike3StartFar, IntakeBallPoses.intakeSpike3End))
                    .build();
            toShootPose = robot.follower.fastPathChainBuilder(IntakeBallPoses.intakeSpike3End, lastPose, TValues.fastInterpolationSpikeShootStart, TValues.fastInterpolationSpikeShootStart, true);
        }

        @Override
        public Pose getEndPoseEst() {
            return lastPose;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(toIntakeSpike3);
                    if (sort) {
                        robot.doSmartShoot(true);
                    }
                    setPathState(1);
                    break;
                case 1:
                    if (!robot.follower.isBusy()) {
                        robot.follower.followPath(toShootPose);
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.indexer.isHasBallsFull() || robot.basicallyHas3() || pathTimer.getElapsedTimeSeconds() > Timings.spikeIntakeTimeOut) {
                        setPathState(3);
                    }
                    break;
                case 3:
                    if ((robot.inShootingZone() || !robot.follower.isBusy()) && robot.movingSlowEnoughToShoot(false)) {
                        if (sort) {
                            robot.indexer.setQueueGivenAttemptedRampOrder(robot.getMotif());
                        } else {
                            robot.indexer.shootAll();
                        }
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (!robot.isIndexerUnjamming() && (robot.indexer.isHasBallsEmpty()) || (sort && robot.indexer.isQueuedBallsEmpty())) {
                        robot.follower.setMaxPower(1);
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
            return "From shoot Far to intake spike 3, state: " + state;
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
        private boolean longLever;
        double leverTimeOut;
        double rampTimeOut;
        Pose lastPose;

        public ToIntakeFromRamp(Robot robot, PathPlanner prevPlanner, boolean sort, boolean longLever) {
            pathTimer = new Timer();
            this.robot = robot;
            this.sort = sort;
            this.longLever = longLever;
            lastPose = ShootPoses.farShoot;
            if (!prevPlanner.hasComms()) {
                startPose = getOptimalStartPose();
            } else {
                startPose = prevPlanner.getEndPoseEst();
            }

            leverTimeOut = longLever ? Timings.longLeverPressTime         : Timings.shortLeverPressTime;
            rampTimeOut =  longLever ? Timings.longSTunnelIntakeTimeOut : Timings.shortSTunnelIntakeTimeOut;
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
            return ShootPoses.optimalSpike2StartFar;
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
                    .addPath(new BezierCurve(startPose, IntakeBallPoses.movingToPushLeverControlPoint , IntakeBallPoses.pushLever))
                    .setFastHeadingInterpolation(TValues.fastInterpolationLeverStart, TValues.fastInterpolationLeverEnd)
                    .build();

            toIntake = robot.follower.linearPathChainBuilder(IntakeBallPoses.pushLever, IntakeBallPoses.intakeFromSTunnel);

            toShootBuilder = robot.follower.pathBuilder();
            if (sort) {
                toShootBuilder.addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeFromSTunnel, IntakeBallPoses.pushLever))
                        .addPath(new BezierLine(IntakeBallPoses.pushLever, lastPose));
            } else {
                toShootBuilder.addPath(new BezierLine(IntakeBallPoses.intakeFromSTunnel, lastPose));
            }
            toShootBuilder.setFastHeadingInterpolation(TValues.fastInterpolationSpikeShootStart, TValues.fastInterpolationSpikeShootEnd, true);
            

            toShootPose = toShootBuilder.build();
        }

        @Override
        public Pose getEndPoseEst() {
            return lastPose;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(toLever);
                    if (sort) {
                        robot.doSmartShoot(true);
                    }
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
                        robot.follower.followPath(toShootPose);
                        setPathState(4);
                    }
                    break;
                case 4:
                    if ((robot.inShootingZone() || !robot.follower.isBusy()) && robot.movingSlowEnoughToShoot(false)) {
                        if (sort) {
                            robot.indexer.setQueueGivenAttemptedRampOrder(robot.getMotif());
                        } else {
                            robot.indexer.shootAll();
                        }
                        setPathState(5);
                    }
                    break;
                case 5:
                    if (!robot.isIndexerUnjamming() && (robot.indexer.isHasBallsEmpty()) || (sort && robot.indexer.isQueuedBallsEmpty())) {
                        robot.follower.setMaxPower(1);
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
            return "From shoot far to intake ramp, state: " + state;
        }
    }

    @Deprecated
    public static class ToIntakeWVision implements PathPlanner {
        /// Intakes from around the secret Tunnel with vision
        // Variables
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = -1;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private Pose lastPose;
        private boolean sort;
        private boolean visionFail = false;
        PathPlanner failSafePath;
        public ToIntakeWVision(Robot robot, PathPlanner prevPlanner, boolean sort) {
            pathTimer = new Timer();
            this.robot = robot;
            this.lastPose = ShootPoses.farShoot;
            this.sort = sort;
            if (!prevPlanner.hasComms()) {
                startPose = getOptimalStartPose();
            } else {
                startPose = prevPlanner.getEndPoseEst();
            }
            failSafePath = new ToIntakeHuman(robot, prevPlanner, sort);
            failSafePath.buildPaths();
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
            return ShootPoses.optimalVisionStart;
        }
        //Path initialization
        PathChain toIntakeBalls;
        PathChain toShootPose;
        @Override
        public void buildPaths() {
            // We create these on the fly during runtime
        }

        @Override
        public Pose getEndPoseEst() {
            return lastPose;
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
                    if (sort) {
                        robot.doSmartShoot(true);
                    }
                    break;
                case 0:
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        visionFail = true;
                        robot.follower.breakFollowing();
                        return false;
                    }
                    intakeBallPose = robot.getIntakeBallPoseFromCam(VisionTypes.Spline);
                    if (intakeBallPose != null) {
                        intakeBallPose = intakeBallPose.setHeading(StartPoses.closeOuter.getHeading());
                        toIntakeBalls = robot.follower.fastPathChainBuilder(startPose, intakeBallPose, TValues.fastInterpolationIntakeStartFar, TValues.fastInterpolationIntakeEndFar);
                        robot.follower.followPath(toIntakeBalls);
                        setPathState(1);
                    }
                    break;
                case 1:
                    if (!robot.follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5 || robot.indexer.isHasBallsFull() || robot.basicallyHas3() || (robot.follower.getVelocity().getMagnitude() < 2) && pathTimer.getElapsedTimeSeconds() > 0.75) {
                        toShootPose = robot.follower.fastPathChainBuilder(intakeBallPose, lastPose, TValues.fastInterpolationSpikeShootStart, TValues.fastInterpolationSpikeShootEnd, true);

                        robot.follower.followPath(toShootPose);
                        setPathState(2);
                    }
                    break;
                case 2:
                    if ((robot.inShootingZone() || !robot.follower.isBusy()) && robot.movingSlowEnoughToShoot(false)) {
                        if (sort) {
                            robot.indexer.setQueueGivenAttemptedRampOrder(robot.getMotif());
                        } else {
                            robot.indexer.shootAll();
                        }
                        setPathState(3);
                    }
                    break;
                case 3:
                    if ((robot.indexer.isHasBallsEmpty())) {
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
            return "To Intake With Vision, state: " + state;
        }
    }

    public static class ToIntakeWVisionSpline implements PathPlanner {
        /// Intakes from around the secret Tunnel with vision
        // Variables
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = -1;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private Pose lastPose;
        private boolean sort;
        private boolean visionFail = false;
        PathPlanner failSafePath;
        public ToIntakeWVisionSpline(Robot robot, PathPlanner prevPlanner, boolean sort, boolean lever) {
            this(robot, prevPlanner, new ToIntakeSTunnelDumb(robot, prevPlanner, sort, lever, false), sort);
        }
        public ToIntakeWVisionSpline(Robot robot, PathPlanner prevPlanner, boolean sort) {
            this(robot, prevPlanner, new ToIntakeSTunnelDumb(robot, prevPlanner, sort, false, false), sort);
        }
        public ToIntakeWVisionSpline(Robot robot, PathPlanner prevPlanner, PathPlanner failSafePath, boolean sort) {
            pathTimer = new Timer();
            this.robot = robot;
            this.lastPose = ShootPoses.farShoot;
            this.sort = sort;
            if (!prevPlanner.hasComms()) {
                startPose = getOptimalStartPose();
            } else {
                startPose = prevPlanner.getEndPoseEst();
            }
            this.failSafePath = failSafePath;
            this.failSafePath.buildPaths();
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
            return ShootPoses.optimalVisionStart;
        }
        //Path initialization
        PathChain toIntakeBalls;
        PathChain toShootPose;
        @Override
        public void buildPaths() {
            // We create these on the fly during runtime
        }

        @Override
        public Pose getEndPoseEst() {
            return lastPose;
        }

        PathChain intakeBallPath;
        @Override
        public boolean run() {
            if (visionFail) {
                return failSafePath.run();
            }
            switch (state) {
                case -1:
                    setPathState(0);
                    if (sort) {
                        robot.doSmartShoot(true);
                    }
                    break;
                case 0:
                    if (pathTimer.getElapsedTimeSeconds() > 0.75) {
                        visionFail = true;
                        robot.follower.breakFollowing();
                        return false;
                    }
                    intakeBallPath = robot.getIntakeBallPathFromCam(VisionTypes.Spline, true, true);
                    if (intakeBallPath != null) {
                        robot.follower.followPath(intakeBallPath);
                        setPathState(1);
                    }
                    break;
                case 1:
                    if (!robot.follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5 || robot.indexer.isHasBallsFull() || robot.basicallyHas3() || (robot.follower.getVelocity().getMagnitude() < 2 && pathTimer.getElapsedTimeSeconds() > 0.5)) {
                        toShootPose = robot.follower.fastPathChainBuilder(intakeBallPath.lastPath().endPose(), lastPose, TValues.fastInterpolationSpikeShootStart, TValues.fastInterpolationSpikeShootEnd, true);

                        robot.follower.followPath(toShootPose);
                        setPathState(2);
                    }
                    break;
                case 2:
                    if ((robot.inShootingZone() || !robot.follower.isBusy()) && robot.movingSlowEnoughToShoot(false)) {
                        if (sort) {
                            robot.indexer.setQueueGivenAttemptedRampOrder(robot.getMotif());
                        } else {
                            robot.indexer.shootAll();
                        }
                        setPathState(3);
                    }
                    break;
                case 3:
                    if ((robot.indexer.isHasBallsEmpty())) {
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
            return "To Intake With Vision, state: " + state;
        }
    }

    public static class ToIntakeHuman implements PathPlanner {
        /// intakes 3 from the Human Player zone
        /// then goes back and shoots them

        Pose offset = new Pose();
        // Variables
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private Pose lastPose;
        private boolean sort;
        public ToIntakeHuman(Robot robot, PathPlanner prevPlanner, boolean sort) {
            pathTimer = new Timer();
            this.robot = robot;
            this.sort = sort;
            if (!prevPlanner.hasComms()) {
                startPose = getOptimalStartPose();
            } else {
                startPose = prevPlanner.getEndPoseEst();
            }
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
            return ShootPoses.farShoot;
        }

        //Path initialization
        PathChain toIntakeHuman;
        PathChain toShootPose;
        @Override
        public void buildPaths() {
            // Path creation
            toIntakeHuman = robot.follower.pathBuilder()
                    .addPath(robot.follower.fastPathBuilder(startPose, IntakeBallPoses.intakeHumanDiagonal, TValues.fastInterpolationIntakeStartFar, TValues.fastInterpolationIntakeEndFar))
                    .addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeHumanDiagonal,IntakeBallPoses.intakeHumanStrait))
                    .build();
            toShootPose = robot.follower.fastPathChainBuilder(IntakeBallPoses.intakeHumanStrait, lastPose, TValues.fastInterpolationSpikeShootStart, TValues.fastInterpolationSpikeShootStart, true);
        }

        @Override
        public Pose getEndPoseEst() {
            return lastPose;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(toIntakeHuman);
                    if (sort) {
                        robot.doSmartShoot(true);
                    }
                    setPathState(1);
                    break;
                case 1:
                    if (robot.indexer.isHasBallsFull() || robot.basicallyHas3() || !robot.follower.isBusy() || robot.follower.getVelocity().getMagnitude() < 0.5 && pathTimer.getElapsedTimeSeconds() > 0.75) {
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.basicallyHas3() || robot.indexer.isHasBallsFull() || pathTimer.getElapsedTimeSeconds() > Timings.humanIntakeTime) {
                        robot.follower.followPath(toShootPose);
                        setPathState(3);
                    }
                    break;
                case 3:
                    if (!robot.follower.isBusy()) {
                        if (sort) {
                            robot.indexer.setQueueGivenAttemptedRampOrder(robot.getMotif());
                        } else {
                            robot.indexer.shootAll();
                        }
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (!robot.isIndexerUnjamming() && (robot.indexer.isHasBallsEmpty()) || (sort && robot.indexer.isQueuedBallsEmpty())) {
                        robot.follower.setMaxPower(1);
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
            return "From shoot far to intake spike 3, state: " + state;
        }
    }

    public static class ToIntakeSTunnelDumb implements PathPlanner {
        /// intakes 3 from the Human Player zone
        /// then goes back and shoots them

        Pose offset = new Pose();
        Pose lowPose;
        // Variables
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private Pose lastPose;
        private boolean sort;
        private boolean lever;
        private boolean high;
        private boolean once = false;
        public ToIntakeSTunnelDumb(Robot robot, PathPlanner prevPlanner, boolean sort, boolean lever, boolean high) {
            pathTimer = new Timer();
            this.robot = robot;
            this.sort = sort;
            this.lever = lever;
            this.high = high;
            if (!prevPlanner.hasComms()) {
                startPose = getOptimalStartPose();
            } else {
                startPose = prevPlanner.getEndPoseEst();
            }
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
            return ShootPoses.optimalVisionStart;
        }

        //Path initialization
        PathBuilder toIntakeSTunnelBuilder;
        PathChain toShootPose;
        Path panicPath;
        @Override
        public void buildPaths() {
            // Path creation
            if (lever) {
                toShootPose = robot.follower.pathBuilder()
                        .addPath(robot.follower.fastPathBuilder(IntakeBallPoses.pushLever, IntakeBallPoses.intakeSTunnelDiagonalEnd, 0.8, 0.9, true))
                        .addPath(robot.follower.fastPathBuilder(IntakeBallPoses.intakeSTunnelDiagonalEnd, lastPose, TValues.fastInterpolationSpikeShootStart, TValues.fastInterpolationSpikeShootStart, true))
                        .build();
            } else {
                toShootPose = robot.follower.fastPathChainBuilder(IntakeBallPoses.intakeSTunnelDiagonalEnd, lastPose, TValues.fastInterpolationSpikeShootStart, TValues.fastInterpolationSpikeShootStart, true);
            }
            panicPath = robot.follower.fastPathBuilder(IntakeBallPoses.pushLever, ShootPoses.distantShoot, TValues.fastInterpolationPreloadStart, TValues.fastInterpolationPreloadEnd);
        }

        @Override
        public Pose getEndPoseEst() {
            return lastPose;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    toIntakeSTunnelBuilder = robot.follower.pathBuilder();
                    if (!high) {
                        lowPose = IntakeBallPoses.intakeSTunnelDiagonalStart;
                        toIntakeSTunnelBuilder
                                .addPath(new Path(new BezierCurve(startPose, IntakeBallPoses.intakeHumanStraitControlPoint, IntakeBallPoses.intakeHumanStrait)))
                                .setFastHeadingInterpolation(TValues.fastInterpolationIntakeStartFar, TValues.fastInterpolationIntakeEndFar)
                                .addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeHumanStrait, IntakeBallPoses.intakeSTunnelDiagonalStart))
                                .addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSTunnelDiagonalStart, IntakeBallPoses.intakeSTunnelDiagonalEnd));
                    } else {
                        double height = robot.getLowestBallHeightFromCam();
                        if (height > -36) {
                            height = 48;
                        }
                        lowPose = new Pose(Math.max(height - 12, -80), IntakeBallPoses.intakeSTunnelDiagonalStart.getY(), IntakeBallPoses.intakeSTunnelDiagonalStart.getHeading());
                        toIntakeSTunnelBuilder
                                .addPath(robot.follower.fastPathBuilder(startPose, lowPose, TValues.fastInterpolationIntakeStartFar, TValues.fastInterpolationIntakeEndFar))
                                .addPath(robot.follower.linearPathBuilder(lowPose, IntakeBallPoses.intakeSTunnelDiagonalEnd));
                    }

                    if (lever) {
                        toIntakeSTunnelBuilder
                                .addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSTunnelDiagonalEnd, IntakeBallPoses.pushLeverFromSTunnel));
                    }

                    robot.follower.followPath(toIntakeSTunnelBuilder.build());
                    if (sort) {
                        robot.doSmartShoot(true);
                    }
                    setPathState(1);
                    break;
                case 1:
                    if (robot.follower.getChainIndex() == 1 || robot.indexer.isHasBallsFull() || robot.basicallyHas3() || !robot.follower.isBusy()) {
                        robot.follower.setMaxPower(0.9);
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.indexer.isHasBallsFull() || robot.basicallyHas3() || !robot.follower.isBusy() || robot.follower.getVelocity().getMagnitude() < 1 && pathTimer.getElapsedTimeSeconds() > 0.75) {
                        robot.follower.setMaxPower(1);
                        robot.follower.followPath(toShootPose);
                        setPathState(3);
                    }
                    break;
                case 3:
                    if (pathTimer.getElapsedTimeSeconds() > 2.5 && !once) {
                        robot.follower.followPath(panicPath);
                        once = true;
                    }
                    if (!robot.follower.isBusy()) {
                        if (sort) {
                            robot.indexer.setQueueGivenAttemptedRampOrder(robot.getMotif());
                        } else {
                            robot.indexer.shootAll();
                        }
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (!robot.isIndexerUnjamming() && (robot.indexer.isHasBallsEmpty()) || (sort && robot.indexer.isQueuedBallsEmpty())) {
                        robot.follower.setMaxPower(1);
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
            return "From shoot far to intake spike 3, state: " + state;
        }
    }
}