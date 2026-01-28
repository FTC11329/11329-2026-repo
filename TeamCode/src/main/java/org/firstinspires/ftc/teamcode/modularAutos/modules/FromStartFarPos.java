package org.firstinspires.ftc.teamcode.modularAutos.modules;

import static org.firstinspires.ftc.teamcode.modularAutos.Common.IntakeBallPoses;
import static org.firstinspires.ftc.teamcode.modularAutos.Common.ShootPoses;
import static org.firstinspires.ftc.teamcode.modularAutos.Common.Timings;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.modularAutos.PathPlanner;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.BallColor;

public class FromStartFarPos {

    public static class shootPreloads implements PathPlanner {
        /// shoots preloads

        Pose offset = new Pose();
        // Variables
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private boolean sort;

        public shootPreloads(Robot robot, Pose startPose, boolean sort) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.sort = sort;
        }

        //Path initialization
        PathChain toShootPose;

        @Override
        public void buildPaths() {
            // Path creation
            toShootPose = robot.follower.linearPathChainBuilder(startPose, ShootPoses.farShoot);
        }

        @Override
        public Pose getEndPoseEst() {
            return ShootPoses.farShoot;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.setMaxPower(0.7);
                    robot.follower.followPath(toShootPose);
                    setPathState(1);
                    break;
                case 1:
                    if (!robot.follower.isBusy()) {
                        robot.follower.setMaxPower(1);
                        robot.doSmartShoot(true);
                        if (sort) {
                            robot.indexer.setQueuedBalls(robot.getMotif());
                        } else {
                            robot.indexer.setQueuedBalls(new BallColor[]{BallColor.Any, BallColor.Any, BallColor.Any});
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
            return "From start Far to shoot preloads, state: " + state;
        }
    }
}