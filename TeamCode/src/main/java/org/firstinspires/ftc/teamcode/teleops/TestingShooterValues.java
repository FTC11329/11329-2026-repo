package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.EndValuesStorer;
import org.firstinspires.ftc.teamcode.util.FancyButton;
import org.firstinspires.ftc.teamcode.util.RobotSide;


public class TestingShooterValues {
    //This is where we introduce the tele-operated controls
    Robot robot;
    // PressHolds
    FancyButton intake;
    FancyButton spitIntake;

    FancyButton autoShoot;
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
    public double hoodAngleOffset;
    public double rpmOffset;
    public Pose startPose;

    public void init() {
        EndValuesStorer endValuesStorer = new EndValuesStorer();
        EndValuesStorer.EndValues endValues = endValuesStorer.loadEndValues();
        int startTurretTicks = endValues.turretTicks;
        startPose = new Pose(endValues.robotX, endValues.robotY, endValues.robotHeading);

        robot = new Robot(telemetry, hardwareMap, robotSide, startTurretTicks, 0);

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
        moveHoodUp = new FancyButton(FancyButton.PressType.LongPress);
        moveHoodDown = new FancyButton(FancyButton.PressType.LongPress);
        decreaseRPM = new FancyButton(FancyButton.PressType.LongPress);
        increaseRPM = new FancyButton(FancyButton.PressType.LongPress);

        deleteme = new FancyButton(FancyButton.PressType.Toggle);

        time = new ElapsedTime();
    }

    public void init_loop() {
        telemetry.addLine("Use gamepad 2 Dpad to change Start Position");
        telemetry.addData("If this works, I get to go home", true);

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
        intake.checkStatus(gamepad2.left_bumper); // Toggle on to intake
        spitIntake.checkStatus(gamepad1.left_bumper || gamepad2.b); // Hold to spit

        // queueGreen.checkStatus(gamepad2.y); // Press to queue green
        // queuePurple.checkStatus(gamepad2.x); // Press to queue purple
        autoShoot.checkStatus(gamepad2.a); // Toggle to turn on auto shoot

        overrideShootPosition.checkStatus(gamepad2.back); // hold to turn on ignore position
        panicShoot.checkStatus(gamepad2.ps); // toggle to turn on panic shoot mode

        resetPose.checkStatus(gamepad2.y);
        moveHoodUp.checkStatus(gamepad2.dpad_up);
        moveHoodDown.checkStatus(gamepad2.dpad_down);  //Buttons to control where the robot aims
        decreaseRPM.checkStatus(gamepad2.dpad_left);
        increaseRPM.checkStatus(gamepad2.dpad_right);

        takePhoto.checkStatus(gamepad2.y);
        debug.checkStatus(gamepad1.start); // hold to print telemetry

        deleteme.checkStatus(false);


        robot.drivetrain.teleopMovement(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_bumper);


        if (intake.startPress) {
            robot.intakeManual();
        }
        if (intake.endPress) {
            robot.stopIntake();
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

        robot.isIntaking(intake.isOn);


        if (moveHoodUp.startPress) {
            hoodAngleOffset += 2;
        }
        if (moveHoodDown.startPress) {
            hoodAngleOffset -= 2;
        }
        if (decreaseRPM.startPress) {
            rpmOffset -= 20;
        }
        if (increaseRPM.startPress) {
            rpmOffset += 20;
        }



        robot.update();
        Pose goal;
        if (robotSide == RobotSide.Blue)  {
            goal = Constants.Vision.blueGoal;
        } else {
            goal = Constants.Vision.redGoal;
        }

        telemetry.addData("distance", robot.getCurrentPose().distanceFrom(goal));
        telemetry.addData("Actual RPM", robot.shooter.getRPM());
        telemetry.addData("Target RPM", robot.shooter.getTargetRpm());
        telemetry.addData("Hood angle", robot.shooter.getHoodPosDeg());
        telemetry.update();
    }

    public void stop() {
        robot.stopAllSubsystems();
    }
}
