package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.AutoReplayTime;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.EndValuesStorer;
import org.firstinspires.ftc.teamcode.util.FancyButton;
import org.firstinspires.ftc.teamcode.util.RobotSide;

public class AutoReplayTeleop {
    Robot robot;


    Gamepad gamepad1;
    Gamepad gamepad2;

    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepadInfo1;
    Gamepad gamepadInfo2;
    AutoReplayTime autoReplay;
    RobotSide robotSide;

    FancyButton intake;
    FancyButton spitIntake;

    FancyButton autoShoot;
    FancyButton queueGreen;
    FancyButton queuePurple;

    FancyButton overrideShootPosition;
    FancyButton panicShoot;

    FancyButton debug;
    FancyButton takePhoto;
    FancyButton movePoseUp;
    FancyButton movePoseDown;
    FancyButton movePoseLeft;
    FancyButton movePoseRight;
    FancyButton resetPose;

    FancyButton deleteme;

    public AutoReplayTeleop(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, RobotSide robotSide) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepadInfo1 = gamepad1;
        this.gamepadInfo2 = gamepad2;
        this.robotSide = robotSide;
    }

    ElapsedTime time;
    double lastTime;
    public double hoodAngle = 20;
    public double rpm = 3000;
    public Pose startPose;

    public void init() {

        EndValuesStorer endValuesStorer = new EndValuesStorer();
        EndValuesStorer.EndValues endValues = endValuesStorer.loadEndValues();
        int startTurretTicks = endValues.turretTicks;
        startPose = new Pose(endValues.robotX, endValues.robotY, endValues.robotHeading);

        robot = new Robot(telemetry, hardwareMap, robotSide, startTurretTicks);
        autoReplay = new AutoReplayTime(robot.follower, telemetry, gamepadInfo1, gamepadInfo2);
        autoReplay.init();

        intake = new FancyButton(FancyButton.PressType.Toggle);
        spitIntake = new FancyButton(FancyButton.PressType.LongPress);

        queueGreen = new FancyButton(FancyButton.PressType.LongPress);
        queuePurple = new FancyButton(FancyButton.PressType.LongPress);
        autoShoot = new FancyButton(FancyButton.PressType.Toggle);

        overrideShootPosition = new FancyButton(FancyButton.PressType.LongPress);
        panicShoot = new FancyButton(FancyButton.PressType.Toggle);

        debug = new FancyButton(FancyButton.PressType.Toggle);

        resetPose = new FancyButton(FancyButton.PressType.LongPress);
        takePhoto = new FancyButton(FancyButton.PressType.LongPress);
        movePoseUp = new FancyButton(FancyButton.PressType.LongPress);
        movePoseDown = new FancyButton(FancyButton.PressType.LongPress);
        movePoseLeft = new FancyButton(FancyButton.PressType.LongPress);
        movePoseRight = new FancyButton(FancyButton.PressType.LongPress);

        deleteme = new FancyButton(FancyButton.PressType.Toggle);

        time = new ElapsedTime();
    }

    public void init_loop() {
        telemetry.addLine("Use gamepad 2 Dpad to change Start Position");

        resetPose.checkStatus(gamepad2.y);

        if (resetPose.startPress) {
            startPose = new Pose(0,0,0);
            robot.turret.encoderOffset = 0;
        }

        telemetry.addData("Start Pose", startPose);
        telemetry.addData("Encoder Offset", robot.turret.encoderOffset);
        robot.follower.setPose(startPose);
        robot.follower.update();
        telemetry.addData("Encoder Offset", robot.getCurrentPose());
        telemetry.update();
    }

    public void start() {
        robot.follower.setPose(startPose);
        robot.follower.update();
    }

    public void loop() {
        autoReplay.update();

        robot.drivetrain.teleopMovement(-gamepadInfo1.left_stick_y, gamepadInfo1.left_stick_x, gamepadInfo1.right_stick_x, gamepadInfo1.right_bumper);
        if (autoReplay.IsReplayOn()){
            gamepad1 = autoReplay.getGamepad1();
            gamepad2 = autoReplay.getGamepad2();
        }
        else{
            gamepad1 = gamepadInfo1;
            gamepad2 = gamepadInfo2;
        }

        intake.checkStatus(gamepad2.left_bumper); // Toggle on to intake
        spitIntake.checkStatus(gamepad1.left_bumper || gamepad2.b); // Hold to spit

        // queueGreen.checkStatus(gamepad2.y); // Press to queue green
        // queuePurple.checkStatus(gamepad2.x); // Press to queue purple
        autoShoot.checkStatus(gamepad2.a); // Toggle to turn on auto shoot

        overrideShootPosition.checkStatus(gamepad2.back); // hold to turn on ignore position
        panicShoot.checkStatus(gamepad2.ps); // toggle to turn on panic shoot mode

        resetPose.checkStatus(gamepad2.y);
        movePoseUp.checkStatus(gamepad2.dpad_up);
        movePoseDown.checkStatus(gamepad2.dpad_down);  //Buttons to control where the robot aims
        movePoseLeft.checkStatus(gamepad2.dpad_left);
        movePoseRight.checkStatus(gamepad2.dpad_right);

        takePhoto.checkStatus(gamepad2.y);
        debug.checkStatus(gamepad1.start); // hold to print telemetry

        deleteme.checkStatus(false);

        if (intake.startPress) {
            robot.intakeManual();
        }
        if (intake.endPress) {
            robot.stopIntake();
            robot.stopIndexer();
        }

        if (spitIntake.startPress) {
            robot.spitIntake();
        }
        if (spitIntake.endPress) {
            if (intake.isOn){
                robot.intakeManual();
            } else {
                robot.stopIntake();
            }
        }

        if (!panicShoot.isOn) {
            if (autoShoot.isOn) {
                robot.prepareShooter();
                robot.shootQueue(overrideShootPosition.isOn);
            } else if (autoShoot.endPress){
                robot.shooter.casualModeOn();
                if (intake.isOn) {
                    robot.stopIndexer();
                }
            }
        }

        // Panic shoot mode
        if (panicShoot.startPress) {
            // distance 71.6
            robot.shooter.setTargetRPM(2336);
            robot.shooter.setHoodDeg(35);
            robot.turret.setTargetDeg(0);
        }
        if (panicShoot.isOn) {
            if (overrideShootPosition.startPress) {
                robot.indexer.spinIndexer(true);
                robot.indexer.transfer(true);
            } else if (overrideShootPosition.endPress) {
                robot.stopIndexer();
            }
        }

        // Take Photo to set position
        if (takePhoto.startPress) {
            robot.autoSetCurrentPose();
        }


        // Changing our aim
        if (robotSide == RobotSide.Blue) {
            if (movePoseUp.startPress) {
                robot.offsetPose.addY(1);
                robot.offsetPose.addX(1);
            }
            if (movePoseDown.startPress) {
                robot.offsetPose.addY(-1);
                robot.offsetPose.addX(-1);
            }
            if (movePoseLeft.startPress) {
                robot.offsetPose.addY(1);
                robot.offsetPose.addX(-1);
            }
            if (movePoseRight.startPress) {
                robot.offsetPose.addY(-1);
                robot.offsetPose.addX(1);
            }
        } else {
            if (movePoseUp.startPress) {
                robot.offsetPose.addY(-1);
                robot.offsetPose.addX(1);
            }
            if (movePoseDown.startPress) {
                robot.offsetPose.addY(1);
                robot.offsetPose.addX(-1);
            }
            if (movePoseLeft.startPress) {
                robot.offsetPose.addY(1);
                robot.offsetPose.addX(1);
            }
            if (movePoseRight.startPress) {
                robot.offsetPose.addY(-1);
                robot.offsetPose.addX(-1);
            }
            if (resetPose.startPress) {
                robot.offsetPose = new Pose(0,0,0);
            }
        }



        // if (queuePurple.startPress) {
        // robot.qBall(BallColor.Purple);
        // }
        // if (queueGreen.startPress) {
        // robot.qBall(BallColor.Green);
        // }

        robot.update(debug.isOn);
//        telemetry.addData("distance", robot.getCurrentPose().distanceFrom(Constants.Vision.blueGoal));

//        double deltaTime = time.milliseconds() - lastTime;
//        telemetry.addData("Loop Time", deltaTime);
//        lastTime = time.milliseconds();

//        telemetry.addData("Encoder RPM", robot.shooter.getRPM());
//        telemetry.addData("Hood angle", robot.shooter.getHoodPosDeg());
    }
    public void stop() {
        robot.stopAllSubsystems();
    }
}
