package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.FancyButton;
import org.firstinspires.ftc.teamcode.util.RobotSide;

public class TestingShooterValues {
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
    FancyButton panicShoot;

    FancyButton debug;
    FancyButton takePhoto;
    FancyButton moveHoodUp;
    FancyButton moveHoodDown;
    FancyButton decreaseRPM;
    FancyButton increaseRPM;
    FancyButton resetPose;

    FancyButton deleteme;

    Gamepad gamepad1;
    Gamepad gamepad2;
    Telemetry telemetry;
    RobotSide robotSide;
    HardwareMap hardwareMap;

    public TestingShooterValues(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap, RobotSide robotSide) {
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
        startPose = new Pose(0, 0, 0);

        robot = new Robot(telemetry, hardwareMap, robotSide, 0, 0);

        intake = new FancyButton(FancyButton.PressType.Toggle);
        spitIntake = new FancyButton(FancyButton.PressType.LongPress);

        queueGreen = new FancyButton(FancyButton.PressType.LongPress);
        queuePurple = new FancyButton(FancyButton.PressType.LongPress);
        autoShoot = new FancyButton(FancyButton.PressType.Toggle);
        smartShoot = new FancyButton(FancyButton.PressType.Toggle);
        fastShootButton = new FancyButton(FancyButton.PressType.LongPress);

        overrideShootPosition = new FancyButton(FancyButton.PressType.LongPress);
        panicShoot = new FancyButton(FancyButton.PressType.Toggle);

        debug = new FancyButton(FancyButton.PressType.Toggle);

        resetPose = new FancyButton(FancyButton.PressType.LongPress);
        takePhoto = new FancyButton(FancyButton.PressType.LongPress);
        moveHoodUp = new FancyButton(FancyButton.PressType.LongPress);
        moveHoodDown = new FancyButton(FancyButton.PressType.LongPress);
        decreaseRPM = new FancyButton(FancyButton.PressType.LongPress);
        increaseRPM = new FancyButton(FancyButton.PressType.LongPress);

        deleteme = new FancyButton(FancyButton.PressType.Toggle);

        time = new ElapsedTime();
    }

    public void init_loop() {
        resetPose.checkStatus(gamepad2.y);

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
        intake.checkStatus(gamepad1.left_bumper); // Toggle on to intake
        spitIntake.checkStatus(gamepad2.left_bumper || gamepad1.left_trigger > 0.5); // Hold to spit

        // queueGreen.checkStatus(gamepad2.y); // Press to queue green
        // queuePurple.checkStatus(gamepad2.x); // Press to queue purple
        autoShoot.checkStatus(gamepad1.a); // Toggle to turn on auto shoot
        smartShoot.checkStatus(gamepad2.b); // Toggle to turn on smart shoot
        fastShootButton.checkStatus(gamepad1.b); // Toggle to turn on smart shoot

        overrideShootPosition.checkStatus(gamepad2.back); // hold to turn on ignore position
        panicShoot.checkStatus(gamepad2.ps); // toggle to turn on panic shoot mode

        moveHoodUp.checkStatus(gamepad1.dpad_up);
        moveHoodDown.checkStatus(gamepad1.dpad_down);  //Buttons to control where the robot aims
        decreaseRPM.checkStatus(gamepad1.dpad_left);
        increaseRPM.checkStatus(gamepad1.dpad_right);

        takePhoto.checkStatus(gamepad1.y); // hold to take photo
        debug.checkStatus(gamepad1.start); // hold to print telemetry

        deleteme.checkStatus(false);


        robot.drivetrain.teleopMovement(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_bumper);


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
            robot.prepareShooter(rpmOffset, hoodAngleOffset);
        } else if (autoShoot.endPress) {
            robot.shooter.stopShooter();
        }
        if (smartShoot.startPress) {
            robot.indexer.setSmartShootBool(true);
        } else if (smartShoot.endPress) {
            robot.indexer.setSmartShootBool(false);
        }

        if (moveHoodUp.startPress) {
            hoodAngleOffset += 1;
        }
        if (moveHoodDown.startPress) {
            hoodAngleOffset -= 1;
        }
        if (decreaseRPM.startPress) {
            rpmOffset -= 10;
        }
        if (increaseRPM.startPress) {
            rpmOffset += 10;
        }

        telemetry.addData("RPM offset", rpmOffset);
        telemetry.addData("Hood offset", hoodAngleOffset);

        robot.update(debug.isOn, fastShootButton.startPress);

    }

    public void stop() {
        robot.stopAllSubsystems();
    }
}
