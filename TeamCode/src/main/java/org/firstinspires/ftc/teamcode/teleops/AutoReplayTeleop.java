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
    FancyButton smartShoot;
    FancyButton fastShootButton;
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
        int startIndexerTicks = endValues.indexerTicks;
        startPose = new Pose(endValues.x, endValues.y, endValues.heading);

        robot = new Robot(telemetry, hardwareMap, robotSide, startTurretTicks, startIndexerTicks);
        autoReplay = new AutoReplayTime(robot.follower, telemetry, gamepadInfo1, gamepadInfo2);
        autoReplay.init();
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
        smartShoot = new FancyButton(FancyButton.PressType.Toggle);
        fastShootButton = new FancyButton(FancyButton.PressType.LongPress);

        deleteme = new FancyButton(FancyButton.PressType.Toggle);

        time = new ElapsedTime();
    }

    public void init_loop() {
        telemetry.addLine("Use gamepad 2 Dpad to change Start Position");

        resetPose.checkStatus(gamepadInfo2.y);

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

        if (autoReplay.IsReplayOn()){
            gamepad1 = autoReplay.getGamepad1();
            gamepad2 = autoReplay.getGamepad2();
        }
        else{
            robot.drivetrain.teleopMovement(-gamepadInfo1.left_stick_y, gamepadInfo1.left_stick_x, gamepadInfo1.right_stick_x, gamepadInfo1.right_bumper);
            gamepad1 = gamepadInfo1;
            gamepad2 = gamepadInfo2;
        }

        intake.checkStatus(gamepad1.left_bumper); // Toggle on to intake
        spitIntake.checkStatus(gamepad2.left_bumper || gamepad1.right_bumper); // Hold to spit

        // queueGreen.checkStatus(gamepad2.y); // Press to queue green
        // queuePurple.checkStatus(gamepad2.x); // Press to queue purple
        autoShoot.checkStatus(gamepad1.a); // Toggle to turn on auto shoot
        smartShoot.checkStatus(gamepad2.b); // Toggle to turn on smart shoot
        fastShootButton.checkStatus(gamepad2.right_bumper); // Toggle to turn on smart shoot

        overrideShootPosition.checkStatus(gamepad2.back); // hold to turn on ignore position
        panicShoot.checkStatus(gamepad2.ps); // toggle to turn on panic shoot mode

        resetPose.checkStatus(gamepad2.y);
        movePoseUp.checkStatus(gamepad2.dpad_up);
        movePoseDown.checkStatus(gamepad2.dpad_down);  //Buttons to control where the robot aims
        movePoseLeft.checkStatus(gamepad2.dpad_left);
        movePoseRight.checkStatus(gamepad2.dpad_right);

        takePhoto.checkStatus(gamepad1.y); // hold to take photo
        debug.checkStatus(gamepad1.start); // hold to print telemetry

        deleteme.checkStatus(false);


        if (intake.startPress) {
            robot.intakeManual();
            robot.isIntaking(true);
        }
        if (intake.endPress) {
            robot.stopIntake();
            robot.isIntaking(false);
        }

        if (spitIntake.startPress) {
            robot.spitIntake();
        }
        if (spitIntake.endPress) {
            robot.spitIntake(false);
            if (intake.isOn){
                robot.intakeManual();
            } else {
                robot.stopIntake();
            }
        }
        // Take Photo to set position
        if (takePhoto.isOn) {
            robot.autoSetCurrentPose();
        }

        if (queuePurple.startPress) {
            robot.qBall(BallColor.Purple);
        }
        if (queueGreen.startPress) {
            robot.qBall(BallColor.Green);
        }


        if (autoShoot.isOn) {
            robot.prepareShooter();
        } else if (autoShoot.endPress) {
            robot.casualShooterModeOn();
        }
        if (smartShoot.startPress) {
            robot.indexer.setSmartShootBool(true);
        } else if (smartShoot.endPress) {
            robot.indexer.setSmartShootBool(false);
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
        robot.update(debug.isOn, fastShootButton.startPress);

    }
    public void stop() {
        robot.stopAllSubsystems();
    }
}
