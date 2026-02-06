package org.firstinspires.ftc.teamcode.modularAutos.runnableWrappers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.modularAutos.Common;
import org.firstinspires.ftc.teamcode.modularAutos.modules.FromShootMidPos;
import org.firstinspires.ftc.teamcode.modularAutos.modules.FromStartClosePos;
import org.firstinspires.ftc.teamcode.modularAutos.PathPlanner;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.EndValuesStorer;
import org.firstinspires.ftc.teamcode.util.RobotSide;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class ExampleRunnableWrapper extends OpMode {
    Pose startPose;
    RobotSide robotSide;
    Robot robot;
    private List<PathPlanner> steps = new ArrayList<>();
    Timer zeroVelocityTimer = new Timer();
    private int currentStep = 0;

    @Override
    public void init() {
        // todo Set These Before Creating
        robotSide = null;
        robot = new Robot(telemetry, hardwareMap, robotSide, 0,0,
                new BallColor[]{
                        BallColor.Green,
                        BallColor.Purple,
                        BallColor.Purple
                });
        // todo Set These Before Creating
        startPose = null;

        steps.add(new FromStartClosePos.ShootAndGoToMidShootPos(robot, lastPose()));
        steps.add(new FromShootMidPos.ToIntakeSpike1(robot, lastPose(), true, false, false));

        robot.follower.setPose(startPose);
    }

    @Override
    public void init_loop() {
        telemetry.addData("start pose", startPose);
        telemetry.addData("shoot pose", Common.ShootPoses.midShoot);
        telemetry.addLine("=== Motif ===");
        BallColor[] motif = robot.getMotif(true);
        if (motif == null) {
            motif = new BallColor[]{BallColor.None, BallColor.None, BallColor.None};
        }
        for (BallColor color : motif) {
            telemetry.addLine(color.name());
        }


        telemetry.update();
    }

    @Override
    public void start() {
        robot.start();
        steps.get(currentStep).buildPaths();
        robot.spinIntake();
    }

    @Override
    public void loop() {
        // to stop the auto
        if (robot.getOpmodeTimeSeconds() > 30) {
            telemetry.addData("Done", true);
            telemetry.update();

            robot.stopAllSubsystems();
            if (robot.follower.getVelocity().getMagnitude() > 1.5) {
                zeroVelocityTimer.resetTimer();
            }
            if (zeroVelocityTimer.getElapsedTimeSeconds() > 1.5) {
                requestOpModeStop();
            }
            return;
        }

        robot.update();
        Drawing.drawShapesDebug(robot.follower);

        // Stops the robot if done
        if (currentStep >= steps.size()) {
            telemetry.addData("Done", true);
            telemetry.update();
            return;
        }

        PathPlanner step = steps.get(currentStep);
        boolean done = step.run();

//        telemetry.addData("time", robot.getOpmodeTimeSeconds());
//        telemetry.addData("name", step);

//        telemetry.update();

        if (done) {
            currentStep++;
            if (currentStep >= steps.size()) {
                return;
            }
            steps.get(currentStep).buildPaths();
        }
    }

    private Pose lastPose() {
        if (steps.isEmpty()) {
            return startPose;
        } else {
            return steps.get(steps.size() - 1).getEndPoseEst();
        }
    }

    @Override
    public void stop() {
        EndValuesStorer endValuesStorer = new EndValuesStorer();
        endValuesStorer.saveEndValues(robot.getCurrentPose().getX(), robot.getCurrentPose().getY(), robot.getCurrentPose().getHeading(), robot.turret.getTicks(), robot.indexer.getEncoderPercentage());
    }
}
