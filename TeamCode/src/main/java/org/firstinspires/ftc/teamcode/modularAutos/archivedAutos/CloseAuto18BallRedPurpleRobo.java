package org.firstinspires.ftc.teamcode.modularAutos.archivedAutos;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.modularAutos.Common;
import org.firstinspires.ftc.teamcode.modularAutos.Common.StartPoses;
import org.firstinspires.ftc.teamcode.modularAutos.PathPlanner;
import org.firstinspires.ftc.teamcode.modularAutos.modules.FromShootMidPos;
import org.firstinspires.ftc.teamcode.modularAutos.modules.FromStartClosePos;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.EndValuesStorer;
import org.firstinspires.ftc.teamcode.util.RobotSide;

import java.util.ArrayList;
import java.util.List;

@Disabled
@Autonomous(name = "Close 18 Red Purple Robo", group = "      Testing", preselectTeleOp = "Main Teleop Red")
public class CloseAuto18BallRedPurpleRobo extends OpMode {
    Pose startPose;
    RobotSide robotSide;
    Robot robot;
    TelemetryManager panelsTelemetry;

    private List<PathPlanner> steps = new ArrayList<>();
    Timer zeroVelocityTimer = new Timer(2000000);
    private int currentStep = 0;
    long lastTime;

    @Override
    public void init() {
        lastTime = System.nanoTime();
        robotSide = RobotSide.Red;
        robot = new Robot(telemetry, hardwareMap, robotSide, 0,0,
                new BallColor[]{
                        BallColor.Green,
                        BallColor.Purple,
                        BallColor.Purple
                });
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        startPose = StartPoses.closeInner;

        steps.add(new FromStartClosePos.ShootAndGoToMidShootPos(robot, lastPose()));
        steps.add(new FromShootMidPos.ToIntakeSpike2  (robot, lastPose(), false,  false, false));
        steps.add(new FromShootMidPos.ToIntakeFromRamp(robot, lastPose(), false,  false, false));
        steps.add(new FromShootMidPos.ToIntakeFromRamp(robot, lastPose(), false,  false, true));
        steps.add(new FromShootMidPos.ToIntakeFromRamp(robot, lastPose(), false,  false, true));
        steps.add(new FromShootMidPos.ToIntakeSpike1  (robot, lastPose(), false,  true, false));

        robot.follower.setPose(startPose);
    }

    @Override
    public void init_loop() {
        telemetry.addData("start pose", startPose);
        telemetry.addData("shoot pose", Common.ShootPoses.midShoot);
        telemetry.addData("red", Common.wasLastRed);
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
            telemetry.addData("zeroVelTimer", zeroVelocityTimer.getElapsedTimeSeconds());
            telemetry.update();

            robot.stopAllSubsystems();
            robot.follower.update();
            if (robot.follower.getVelocity().getMagnitude() > 0.5) {
                zeroVelocityTimer.resetTimer();
            }
            if (zeroVelocityTimer.getElapsedTimeSeconds() > 2) {
                requestOpModeStop();
            }
            return;
        }

        robot.update();
        robot.prepareShooter();

        // Stops the robot if done
        if (currentStep >= steps.size()) {
            telemetry.addData("Done", true);
            telemetry.update();
            return;
        }


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
        panelsTelemetry.addData("all", (System.nanoTime() - lastTime) * 1e-6);
        panelsTelemetry.update();
        lastTime = System.nanoTime();
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
