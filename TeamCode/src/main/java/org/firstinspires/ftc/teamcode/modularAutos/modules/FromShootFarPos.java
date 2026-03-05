package org.firstinspires.ftc.teamcode.modularAutos.modules;

import static org.firstinspires.ftc.teamcode.modularAutos.Common.IntakeBallPoses;
import static org.firstinspires.ftc.teamcode.modularAutos.Common.ShootPoses;
import static org.firstinspires.ftc.teamcode.modularAutos.Common.Timings;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.modularAutos.Common;
import org.firstinspires.ftc.teamcode.modularAutos.PathPlanner;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.MeanBallPoses;

public class FromShootFarPos {

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
        public ToIntakeSpike3(Robot robot, Pose startPose, boolean sort) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.sort = sort;
        }

        //Path initialization
        Path toIntakeSpike3;
        Path finishIntakeSpike3;
        Path toShootPose;
        @Override
        public void buildPaths() {
            // Path creation
            toIntakeSpike3 = robot.follower.linearPathBuilder(startPose, IntakeBallPoses.intakeSpike3Start);
            finishIntakeSpike3 = robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike3Start, IntakeBallPoses.intakeSpike3End);
            toShootPose = robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike3End, ShootPoses.farShoot);
            toShootPose.setBrakingStrength(0.3);
        }

        @Override
        public Pose getEndPoseEst() {
            return ShootPoses.farShoot;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(toIntakeSpike3);
                    setPathState(1);
                    break;
                case 1:
                    if (!robot.follower.isBusy()) {
                        robot.follower.followPath(finishIntakeSpike3);
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (!robot.follower.isBusy()) {
                        robot.follower.followPath(toShootPose);
                        setPathState(3);
                    }
                    break;
                case 3:
                    if (robot.indexer.isHasBallsFull() || pathTimer.getElapsedTimeSeconds() > Timings.spikeIntakeTimeOut) {
                        robot.indexer.setHasBalls(new BallColor[]{BallColor.Green, BallColor.Purple, BallColor.Purple});
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (robot.inShootingZone() || !robot.follower.isBusy()) {
                        robot.doSmartShoot(true);
                        if (sort) {
                            robot.indexer.setQueuedBalls(robot.getMotif());
                        } else {
                            robot.indexer.setQueuedBalls(new BallColor[]{BallColor.Any, BallColor.Any, BallColor.Any});
                        }
                        setPathState(5);
                    }
                    break;
                case 5:
//                    if ((robot.indexer.isHasBallsEmpty() || (sort && robot.indexer.isQueuedBallsEmpty())) || (sort && pathTimer.getElapsedTimeSeconds() > Timings.sortShootTimeOut || !sort && pathTimer.getElapsedTimeSeconds() > Timings.shootTimeOut)) {
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
            return "From shoot Far to intake spike 3, state: " + state;
        }
    }

    public static class ToIntakeWVision implements PathPlanner {
        /// Intakes from around the secret Tunnel with vision
        // Variables
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private Pose lastPose;
        public ToIntakeWVision(Robot robot, Pose startPose) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.lastPose = ShootPoses.farShoot;
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
        PathChain toIntakeBalls;
        PathChain toShootPose;
        @Override
        public void buildPaths() {
            // We create these on the fly during runtime
        }

        @Override
        public Pose getEndPoseEst() {
            //todo
            return ShootPoses.farShoot;
        }
        Pose intakeBallPose;
        @Override
        public boolean run() {
            switch (state) {
                case 0:
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
                    if (!robot.follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3 || robot.indexer.isHasBallsFull()) {
                        toShootPose = robot.follower.linearPathChainBuilder(intakeBallPose, lastPose);

                        robot.follower.followPath(toShootPose);
                    }
                    break;
                case 2:
                    if (!robot.follower.isBusy()) {
                        robot.indexer.shootAll();
                        setPathState(5);
                    }
                    break;
                case 3:
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
        //todo
        public String toString() {
            return "NAME, state: " + state;
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
        private boolean sort;
        public ToIntakeHuman(Robot robot, Pose startPose, boolean sort) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.sort = sort;
        }

        //Path initialization
        Path toIntakeHuman;
        Path toShootPose;
        @Override
        public void buildPaths() {
            // Path creation
            toIntakeHuman = robot.follower.linearPathBuilder(startPose, IntakeBallPoses.intakeHuman);
            toShootPose = robot.follower.linearPathBuilder(IntakeBallPoses.intakeHuman, ShootPoses.farShoot);
            toShootPose.setBrakingStrength(0.3);
        }

        @Override
        public Pose getEndPoseEst() {
            return ShootPoses.farShoot;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(toIntakeHuman);
                    setPathState(1);
                    break;
                case 1:
                    if (!robot.follower.isBusy()) {
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.indexer.isHasBallsFull() || pathTimer.getElapsedTimeSeconds() > Timings.humanIntakeTime) {
                        robot.follower.followPath(toShootPose);
                        setPathState(3);
                    }
                    break;
                case 3:
                    if (robot.indexer.isHasBallsFull() || pathTimer.getElapsedTimeSeconds() > Timings.spikeIntakeTimeOut) {
                        robot.indexer.setHasBalls(new BallColor[]{BallColor.Green, BallColor.Purple, BallColor.Purple});
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (!robot.follower.isBusy()) {
                        robot.doSmartShoot(true);
                        if (sort) {
                            robot.indexer.setQueuedBalls(robot.getMotif());
                        } else {
                            robot.indexer.setQueuedBalls(new BallColor[]{BallColor.Any, BallColor.Any, BallColor.Any});
                        }
                        setPathState(5);
                    }
                    break;
                case 5:
//                    if ((robot.indexer.isHasBallsEmpty() || (sort && robot.indexer.isQueuedBallsEmpty())) || (sort && pathTimer.getElapsedTimeSeconds() > Timings.sortShootTimeOut || !sort && pathTimer.getElapsedTimeSeconds() > Timings.shootTimeOut)) {
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
            return "From shoot mid to intake spike 3, state: " + state;
        }
    }
    public static class ToIntakeHumanThenWait implements PathPlanner {
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
        private boolean sort;
        public ToIntakeHumanThenWait(Robot robot, Pose startPose, boolean sort) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.sort = sort;
        }

        //Path initialization
        Path toIntakeHuman;
        PathChain toWaitPose;
        Path toShootPose;
        @Override
        public void buildPaths() {
            // Path creation
            toIntakeHuman = robot.follower.linearPathBuilder(startPose, IntakeBallPoses.intakeHuman);
            toWaitPose = robot.follower.pathBuilder()
                    .addPath(new BezierCurve(IntakeBallPoses.intakeHuman, IntakeBallPoses.intakeSTunnelAfterHumanControl, IntakeBallPoses.intakeSTunnelAfterHuman))
                    .setLinearHeadingInterpolation(IntakeBallPoses.intakeHuman, IntakeBallPoses.intakeSTunnelAfterHuman)
                    .build();
            toShootPose = robot.follower.linearPathBuilder(IntakeBallPoses.intakeSpike3End, ShootPoses.farShoot);
            toShootPose.setBrakingStrength(0.3);
        }

        @Override
        public Pose getEndPoseEst() {
            return ShootPoses.farShoot;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(toIntakeHuman);
                    if (robot.getOpmodeTimeSeconds() > Timings.farShootWaitUntil) {
                        setPathState(67676767/*tehe*/);
                    } else {
                        setPathState(1);
                    }
                    break;
                case 1:
                    if (robot.indexer.isHasBallsFull() || !robot.follower.isBusy()) {
                        robot.follower.followPath(toWaitPose);
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.indexer.isHasBallsFull() || robot.getOpmodeTimeSeconds() > Timings.farShootWaitUntil) {
                        setPathState(3);
                    }
                    break;
                case 3:
                    if (robot.inShootingZone() || !robot.follower.isBusy()) {
                        robot.doSmartShoot(true);
                        if (sort) {
                            robot.indexer.setQueuedBalls(robot.getMotif());
                        } else {
                            robot.indexer.setQueuedBalls(new BallColor[]{BallColor.Any, BallColor.Any, BallColor.Any});
                        }
                        setPathState(4);
                    }
                    break;
                case 4:
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
            return "From shoot mid to intake spike 3, state: " + state;
        }
    }

}