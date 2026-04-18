package org.firstinspires.ftc.teamcode.modularAutos.runnableWrappers;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.modularAutos.Common;
import org.firstinspires.ftc.teamcode.modularAutos.modules.Commands;
import org.firstinspires.ftc.teamcode.modularAutos.modules.FromShootMidPos;
import org.firstinspires.ftc.teamcode.modularAutos.modules.FromStartClosePos;
import org.firstinspires.ftc.teamcode.modularAutos.PathPlanner;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.EndValuesStorer;
import org.firstinspires.ftc.teamcode.util.FieldShapes;
import org.firstinspires.ftc.teamcode.util.RobotSide;
import org.firstinspires.ftc.teamcode.util.ShapeDetection;

import java.util.ArrayList;
import java.util.List;

@Disabled
@Autonomous(name = "Exaple", group = "Comp", preselectTeleOp = "Main Teleop Red")
public class ExampleRunnableWrapper extends OpMode {
    Pose startPose;
    RobotSide robotSide;
    Robot robot;
    TelemetryManager panelsTelemetry;

    private List<PathPlanner> steps = new ArrayList<>();
    Timer zeroVelocityTimer = new Timer(2000000);
    private int currentStep = 0;
    private boolean parkPathFollowed = false;

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

        steps.add(new FromStartClosePos.ShootAndGoToMidShootPos(robot, lastPlanner()));
        steps.add(new FromShootMidPos.ToIntakeSpike1(robot, lastPlanner(), true, false, false));

        wComms(steps);
        robot.follower.setPose(startPose);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void init_loop() {
        telemetry.addData("start pose", startPose);
        telemetry.addData("shoot pose", Common.ShootPoses.midShoot);
        telemetry.addData("Is Red", Common.wasLastRed);
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
        robot.setPipelineIndex(2);
    }

    boolean firstDeInit = false;
    @Override
    public void loop() {
        // to stop the auto
        if (robot.getOpmodeTimeSeconds() > 30) {
            telemetry.addData("Done", true);
            telemetry.addData("Time ", 2 - zeroVelocityTimer.getElapsedTimeSeconds());
            telemetry.update();

            robot.stopAllSubsystems();
            if (robot.follower.getVelocity().getMagnitude() > 0.5 || !firstDeInit) {
                firstDeInit = true;
                zeroVelocityTimer.resetTimer();
            }
            if (zeroVelocityTimer.getElapsedTimeSeconds() > 2) {
                requestOpModeStop();
            }
            return;
        }

        robot.update();

        if (!parkPathFollowed && robot.getOpmodeTimeSeconds() > 29.25 && !ShapeDetection.isRobotInside(FieldShapes.closeTriangle, robot.getCurrentPose().plusVector(robot.follower.getVelocity(), 0.75))) {
            if (robot.getCurrentPose().getX() > - 25) {
                robot.follower.followPath(robot.follower.linearPathBuilder(Common.ShootPoses.parkShoot, Common.IntakeBallPoses.intakeSpike2Start));
            } else {
                robot.follower.followPath(robot.follower.linearPathBuilder(Common.ShootPoses.farShoot, Common.IntakeBallPoses.intakeSpike3StartFar));
            }
            parkPathFollowed = true;
            return;
        } else if (parkPathFollowed) {
            return;
        }


        // Stops the robot if done
        if (currentStep >= steps.size()) {
            telemetry.addData("Done", true);
            telemetry.update();
            return;
        }

        robot.prepareShooter(steps.get(currentStep).useSOTF());

        PathPlanner step = steps.get(currentStep);
        boolean done = step.run();

        if (done) {
            currentStep++;
            if (currentStep >= steps.size()) {
                return;
            }
            steps.get(currentStep).buildPaths();
        }

//        Drawing.drawDebug(robot.follower);
//        telemetry.addData("time", robot.getOpmodeTimeSeconds());
//        telemetry.addData("name", step);
//        for (BallColor i : robot.indexer.getBallCells()) {
//            telemetry.addData("hasBalls", i);
//        }
//        panelsTelemetry.addData("all", (System.nanoTime() - lastTime) * 1e-6);
//        panelsTelemetry.update();
//        lastTime = System.nanoTime();

    }

    private PathPlanner lastPlanner() {
        if (steps.isEmpty()) {
            return new Commands.nullPlanner(startPose);
        } else {
            return steps.get(steps.size() - 1);
        }
    }

    private void wComms(List<PathPlanner> steps) {
        Pose lastOptimalPose = null;
        for (int i = steps.size() - 1; i >= 0; i--) {
            PathPlanner planner = steps.get(i);
            if (!planner.hasComms()) {
                lastOptimalPose = null;
                continue;
            }
            if (lastOptimalPose != null) {
                planner.setOptimalEndPose(lastOptimalPose);
            }
            lastOptimalPose = planner.getOptimalStartPose();
        }
    }

    @Override
    public void stop() {
        EndValuesStorer endValuesStorer = new EndValuesStorer();
        endValuesStorer.saveEndValues(robot.getCurrentPose().getX(), robot.getCurrentPose().getY(), robot.getCurrentPose().getHeading(), robot.turret.getTicks(), robot.indexer.getEncoderPercentage());
    }
}
