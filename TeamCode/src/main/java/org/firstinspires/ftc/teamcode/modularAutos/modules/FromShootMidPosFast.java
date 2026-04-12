package org.firstinspires.ftc.teamcode.modularAutos.modules;

import static org.firstinspires.ftc.teamcode.modularAutos.Common.DrivePower;
import static org.firstinspires.ftc.teamcode.modularAutos.Common.IntakeBallPoses;
import static org.firstinspires.ftc.teamcode.modularAutos.Common.ShootPoses;
import static org.firstinspires.ftc.teamcode.modularAutos.Common.Timings;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.modularAutos.PathPlanner;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Deprecated
public class FromShootMidPosFast {
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
        private boolean sort;
        private boolean parkAfter;
        Pose lastPose;

        public ToIntakeSpike1(Robot robot, PathPlanner prevPlanner, boolean sort, boolean parkAfter) {
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
        public boolean hasComms() {
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
        PathChain pathChain;
        PathChain toShootPose;
        @Override
        public void buildPaths() {
            // Path creation
            pathChain = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(startPose, IntakeBallPoses.intakeSpike1FastControlPoint, IntakeBallPoses.intakeSpike1Fast))
                    .build();

            toShootPose = robot.follower.pathBuilder()
                    .addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike1Fast, lastPose))
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
                    robot.follower.followPath(pathChain);
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
                    if ((robot.inShootingZone() || !robot.follower.isBusy()) && (parkAfter || robot.follower.getVelocity().getMagnitude() < Timings.shootVelocityClose)) {
                        if (parkAfter) {
                            robot.follower.setMaxPower(DrivePower.shootOnThFly);
                        }
                        if (sort) {
                            robot.doSmartShoot(true);
                            robot.indexer.setQueueGivenAttemptedRampOrder(robot.getMotif());
                        } else {
                            robot.indexer.shootAll();
                        }
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (pathTimer.getElapsedTimeSeconds() > 1.5 && !sort) {
                        robot.indexerUnjam();
                    }
                    if ((robot.indexer.isHasBallsEmpty() || robot.indexer.autoFastEnd()) || (sort && robot.indexer.isQueuedBallsEmpty())) {
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
        private boolean sort;
        private boolean parkAfter;
        private boolean prevHasComms;
        Pose lastPose;

        public ToIntakeSpike2(Robot robot, PathPlanner prevPlanner, boolean sort, boolean parkAfter) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = prevPlanner.getEndPoseEst();
            this.sort = sort;
            this.parkAfter = parkAfter;
            lastPose = parkAfter ? ShootPoses.parkShoot : ShootPoses.midShoot;
            prevHasComms = prevPlanner.hasComms();
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
        PathChain toShootPose;
        @Override
        public void buildPaths() {
            // Path creation
            pathChain = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(startPose, IntakeBallPoses.intakeSpike2FastControlPoint, IntakeBallPoses.intakeSpike2Fast))
                    .build();
            toShootPose = robot.follower.pathBuilder()
                    .addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike2Fast, lastPose))
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
                    robot.follower.followPath(pathChain);
                    setPathState(1);
                    break;
                case 1:
                    if (!robot.follower.isBusy()) {
                        robot.follower.followPath(toShootPose);
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.indexer.isHasBallsFull() || pathTimer.getElapsedTimeSeconds() > Timings.fastSpikeIntakeTimeOut) {
                        setPathState(3);
                    }
                    break;
                case 3:
                    if ((robot.inShootingZone() || !robot.follower.isBusy()) && (parkAfter || robot.follower.getVelocity().getMagnitude() < Timings.shootVelocityClose)) {
                        if (parkAfter) {
                            robot.follower.setMaxPower(DrivePower.shootOnThFly);
                        }
                        if (sort) {
                            robot.doSmartShoot(true);
                            robot.indexer.setQueueGivenAttemptedRampOrder(robot.getMotif());
                        } else {
                            robot.indexer.shootAll();
                        }
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (pathTimer.getElapsedTimeSeconds() > 1.5 && !sort) {
                        robot.indexerUnjam();
                    }
                    if ((robot.indexer.isHasBallsEmpty() || robot.indexer.autoFastEnd()) || (sort && robot.indexer.isQueuedBallsEmpty())) {
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
        PathChain toShootPose;
        @Override
        public void buildPaths() {
            // Path creation
            pathChain = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(startPose, IntakeBallPoses.intakeSpike3FastControlPoint, IntakeBallPoses.intakeSpike3Fast))
                    .setTangentHeadingInterpolation()
                    .build();
            toShootPose = robot.follower.pathBuilder()
                    .addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike3Fast, lastPose))
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
                    robot.follower.followPath(pathChain);
                    setPathState(1);
                    break;
                case 1:
                    if (!robot.follower.isBusy()) {
                        robot.follower.followPath(toShootPose);
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.indexer.isHasBallsFull() || pathTimer.getElapsedTimeSeconds() > Timings.fastSpikeIntakeTimeOut) {
                        setPathState(3);
                    }
                    break;
                case 3:
                    if ((robot.inShootingZone() || !robot.follower.isBusy()) && (parkAfter || robot.follower.getVelocity().getMagnitude() < Timings.shootVelocityClose)) {
                        if (parkAfter) {
                            robot.follower.setMaxPower(DrivePower.shootOnThFly);
                        }
                        if (sort) {
                            robot.doSmartShoot(true);
                            robot.indexer.setQueueGivenAttemptedRampOrder(robot.getMotif());
                        } else {
                            robot.indexer.shootAll();
                        }
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (pathTimer.getElapsedTimeSeconds() > 1.5 && !sort) {
                        robot.indexerUnjam();
                    }
                    if ((robot.indexer.isHasBallsEmpty() || robot.indexer.autoFastEnd()) || (sort && robot.indexer.isQueuedBallsEmpty())) {
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
            this.startPose = prevPlanner.getEndPoseEst();
            this.sort = sort;
            this.parkAfter = parkAfter;
            this.longLever = longLever;
            if (!prevPlanner.hasComms()) {
                startPose = getOptimalStartPose();
            } else {
                startPose = prevPlanner.getEndPoseEst();
            }

            leverTimeOut = longLever ? Timings.longLeverPressTime         : Timings.shortLeverPressTime;
            rampTimeOut =  longLever ? Timings.longSTunnelIntakeTimeOut   : Timings.shortSTunnelIntakeTimeOut;
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
            return ShootPoses.optimalSpike1Start;
        }

        //Path initialization
        PathChain toLever;
        PathChain toIntake;
        PathBuilder toShootBuilder;
        PathChain toShootPose;
        @Override
        public void buildPaths() {
            // Path creation
            toLever = robot.follower.linearPathChainBuilder(startPose, IntakeBallPoses.pushLever);

            toIntake = robot.follower.linearPathChainBuilder(IntakeBallPoses.pushLever, IntakeBallPoses.intakeFromSTunnel);

            toShootBuilder = robot.follower.pathBuilder();
            if (sort && !longLever) {
                toShootBuilder.addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeFromSTunnel, IntakeBallPoses.pushLever));
            }
            if (sort || longLever) {
                toShootBuilder.addPath(robot.follower.linearPathBuilder(IntakeBallPoses.pushLever, lastPose));
            } else {
                toShootBuilder.addPath(robot.follower.linearPathBuilder(IntakeBallPoses.intakeFromSTunnel, lastPose));
            }
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
                    if ((robot.inShootingZone() || !robot.follower.isBusy()) && robot.follower.getVelocity().getMagnitude() < Timings.shootVelocityClose) {
                        if (parkAfter) {
                            robot.follower.setMaxPower(DrivePower.shootOnThFly);
                        }
                        if (sort) {
                            robot.doSmartShoot(true);
                            robot.indexer.setQueueGivenAttemptedRampOrder(robot.getMotif());
                        } else {
                            robot.indexer.shootAll();
                        }
                        setPathState(5);
                    }
                    break;
                case 5://       if robot shoot all balls v                                                                                                              if timeout v
                    if (pathTimer.getElapsedTimeSeconds() > 1.5 && !sort) {
                        robot.indexerUnjam();
                    }
                    if ((robot.indexer.isHasBallsEmpty() || robot.indexer.autoFastEnd()) || (sort && robot.indexer.isQueuedBallsEmpty())) {
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

}