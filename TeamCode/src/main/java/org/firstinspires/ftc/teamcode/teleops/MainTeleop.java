package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.EndValuesStorer;
import org.firstinspires.ftc.teamcode.util.FancyButton;
import org.firstinspires.ftc.teamcode.util.RobotSide;

public class MainTeleop {
    //This is where we introduce the tele-operated controls
    Robot robot;
    // PressHolds
    FancyButton intake;
    FancyButton spitIntake;

    FancyButton autoShoot;
    FancyButton smartShoot;
    FancyButton fastShootButton;
    FancyButton queueGreen;
    FancyButton queuePurple;

    FancyButton overrideShootPosition;
    FancyButton overrideIntake;
    FancyButton panicShoot;

    FancyButton debug;
    FancyButton takePhoto;
    FancyButton movePoseUp;
    FancyButton movePoseDown;
    FancyButton movePoseLeft;
    FancyButton movePoseRight;
    FancyButton resetPose;
    FancyButton unjamSpindexer;
    FancyButton climb;

    FancyButton deleteme;

    Gamepad gamepad1;
    Gamepad gamepad2;
    Telemetry telemetry;
    RobotSide robotSide;
    HardwareMap hardwareMap;
    double RPMoffset;
    double distanceOffset;
    double turretOffset;

    public MainTeleop(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap, RobotSide robotSide) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.robotSide = robotSide;
    }

    ElapsedTime time;
    double lastTime;
    public double hoodAngleOffset = 0;
    public double rpmOffset = 0;
    public Pose startPose;

    public void init() {
        EndValuesStorer endValuesStorer = new EndValuesStorer();
        EndValuesStorer.EndValues endValues = endValuesStorer.loadEndValues();
        int startTurretTicks = endValues.turretTicks;
        double startIndexerTicks = endValues.indexerPos;
        startPose = new Pose(endValues.x, endValues.y, endValues.heading);

        robot = new Robot(telemetry, hardwareMap, robotSide, startTurretTicks, startIndexerTicks);

        intake = new FancyButton(FancyButton.PressType.Toggle);
        spitIntake = new FancyButton(FancyButton.PressType.LongPress);

        queueGreen = new FancyButton(FancyButton.PressType.LongPress);
        queuePurple = new FancyButton(FancyButton.PressType.LongPress);
        autoShoot = new FancyButton(FancyButton.PressType.Toggle);
        smartShoot = new FancyButton(FancyButton.PressType.Toggle);
        fastShootButton = new FancyButton(FancyButton.PressType.LongPress);

        overrideShootPosition = new FancyButton(FancyButton.PressType.LongPress);
        overrideIntake = new FancyButton(FancyButton.PressType.LongPress);
        panicShoot = new FancyButton(FancyButton.PressType.Toggle);

        debug = new FancyButton(FancyButton.PressType.Toggle);

        climb = new FancyButton(FancyButton.PressType.Toggle);
        unjamSpindexer = new FancyButton(FancyButton.PressType.LongPress);
        resetPose = new FancyButton(FancyButton.PressType.LongPress);
        takePhoto = new FancyButton(FancyButton.PressType.LongPress);
        movePoseUp = new FancyButton(FancyButton.PressType.LongPress);
        movePoseDown = new FancyButton(FancyButton.PressType.LongPress);
        movePoseLeft = new FancyButton(FancyButton.PressType.LongPress);
        movePoseRight = new FancyButton(FancyButton.PressType.LongPress);

        deleteme = new FancyButton(FancyButton.PressType.Toggle);

        robot.follower.setPose(startPose);
        time = new ElapsedTime();
    }

    public void init_loop() {
        resetPose.checkStatus(gamepad2.y);
        if (gamepad2.bWasPressed() || gamepad1.bWasPressed()) {
            startPose = new Pose();
            robot.follower.setPose(startPose);
            robot.turret.encoderOffset = 0;
            robot.indexer.encoderOffsetFromAuto = 0;
            robot.indexer.setIndexerPos(0);
        }

        telemetry.addData("Start Pose", startPose);
        telemetry.addData("Turret Encoder Offset", robot.turret.encoderOffset);
        telemetry.addData("Indexer Encoder Offset", robot.indexer.encoderOffsetFromAuto);
        telemetry.update();
        robot.follower.update();
    }

    public void start() {
        robot.follower.setPose(startPose);
        robot.follower.update();
        robot.start();
    }

    public void loop() {
        unjamSpindexer.checkStatus(gamepad1.right_trigger > .5 || gamepad2.right_trigger > .5);
        intake.checkStatus(gamepad2.left_bumper); // Toggle on to intake
        spitIntake.checkStatus(gamepad2.right_bumper || gamepad1.left_bumper); // Hold to spit

        queueGreen.checkStatus(gamepad2.y); // Press to queue green
        queuePurple.checkStatus(gamepad2.x); // Press to queue purple
        autoShoot.checkStatus(gamepad2.a); // Toggle to turn on auto shoot
        fastShootButton.checkStatus(gamepad2.b || gamepad1.b); // press to shoot 3
        smartShoot.checkStatus(gamepad2.back); // Toggle to turn on smart shoot

        overrideIntake.checkStatus(gamepad2.left_bumper); // hold to turn on ignore allowintaking
        panicShoot.checkStatus(gamepad2.ps); // toggle to turn on panic shoot

        movePoseUp.checkStatus(gamepad2.dpad_up);
        movePoseDown.checkStatus(gamepad2.dpad_down);  //Buttons to control where the robot aims
        movePoseLeft.checkStatus(gamepad2.dpad_left);
        movePoseRight.checkStatus(gamepad2.dpad_right);

        takePhoto.checkStatus(gamepad1.y); // press to take photo
        debug.checkStatus(gamepad1.start); // toggle to print telemetry
        resetPose.checkStatus(gamepad1.x);
        climb.checkStatus(gamepad1.back);

        robot.drivetrain.teleopMovement(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_bumper);

        if (intake.startPress) {
            robot.spinIntake(true);
        }
        if (intake.endPress) {
            robot.spinIntake(false);
        }

        if (spitIntake.startPress) {
            robot.spitIntake(true);
        }
        if (spitIntake.endPress) {
            robot.spitIntake(false);
        }
        if (intake.isOn){
            robot.doIntake();
        } else {
            robot.stopIntake();
        }
        robot.setIntakeOverride(overrideIntake.isOn);

        if (queuePurple.startPress) {
            robot.qBall(BallColor.Purple);
        }
        if (queueGreen.startPress) {
            robot.qBall(BallColor.Green);
        }

        robot.doSmartShoot(smartShoot.isOn);

        if (fastShootButton.startPress) {
            robot.shootAll();
        }

        if (autoShoot.isOn) {
            robot.prepareShooter();
        } else if (autoShoot.endPress) {
            robot.casualShooterModeOn();
        }

        if (unjamSpindexer.startPress) {
            robot.indexerUnjam();
        }
        // Take Photo to set position
        if (takePhoto.isPressed) {
            robot.setAveragePose();
        } else if (takePhoto.endPress) {
            robot.clearAveragePose();
        }

         // Changing our aim
        if (movePoseUp.startPress) {
            distanceOffset += 1.4;
        }
        if (movePoseDown.startPress) {
            distanceOffset -= 1.4;
        }
        if (movePoseLeft.startPress) {
            turretOffset += 1;
        }
        if (movePoseRight.startPress) {
            turretOffset -= 1;
        }
        robot.setOffset(distanceOffset, turretOffset);
        robot.setPanicShoot(panicShoot.isOn, fastShootButton.isOn);
        if (resetPose.startPress) {
            robot.reZeroAtCorner();
        }
        if (climb.isOn) {
            robot.climb();
        } else {
            robot.storeClimber();
        }


        robot.setShooterOffset(rpmOffset, hoodAngleOffset);
        robot.update(debug.isOn);

//        double deltaTime = time.milliseconds() - lastTime;
//        telemetry.addData("loopTimes", deltaTime);
//        lastTime = time.milliseconds();

    }

    public void stop() {
        robot.stopAllSubsystems();
    }
}
