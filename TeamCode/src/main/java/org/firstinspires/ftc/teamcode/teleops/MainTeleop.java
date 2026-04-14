package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.math.Vector;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.EndValuesStorer;
import org.firstinspires.ftc.teamcode.util.FancyButton;
import org.firstinspires.ftc.teamcode.util.RobotSide;
import org.firstinspires.ftc.teamcode.util.ShootOnTheFly.ShotType;

public class MainTeleop {
    //This is where we introduce the tele-operated controls
    Robot robot;
    // PressHolds
    FancyButton brake;
    FancyButton intake;
    FancyButton spitIntake;

    FancyButton turnSOTFOn;
    FancyButton turnSOTFOff;

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
    FancyButton reZeroIndexer;
    FancyButton cycleCycler;
    FancyButton useCycleCycler;

    FancyButton manualTurretStart;

    Gamepad gamepad1;
    Gamepad gamepad2;
    Telemetry telemetry;
    RobotSide robotSide;
    HardwareMap hardwareMap;
    double RPMoffset;
    boolean sotfIsOn = true;

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

        brake = new FancyButton(FancyButton.PressType.LongPress);
        intake = new FancyButton(FancyButton.PressType.Toggle);
        spitIntake = new FancyButton(FancyButton.PressType.LongPress);

        turnSOTFOn = new FancyButton(FancyButton.PressType.LongPress);
        turnSOTFOff = new FancyButton(FancyButton.PressType.LongPress);

        queueGreen = new FancyButton(FancyButton.PressType.LongPress);
        queuePurple = new FancyButton(FancyButton.PressType.LongPress);
        autoShoot = new FancyButton(FancyButton.PressType.Toggle);
        smartShoot = new FancyButton(FancyButton.PressType.Toggle);
        fastShootButton = new FancyButton(FancyButton.PressType.LongPress);

        overrideShootPosition = new FancyButton(FancyButton.PressType.LongPress);
        overrideIntake = new FancyButton(FancyButton.PressType.LongPress);
        panicShoot = new FancyButton(FancyButton.PressType.Toggle);
        reZeroIndexer = new FancyButton(FancyButton.PressType.Toggle);

        debug = new FancyButton(FancyButton.PressType.Toggle);

        climb = new FancyButton(FancyButton.PressType.Toggle);
        unjamSpindexer = new FancyButton(FancyButton.PressType.LongPress);
        resetPose = new FancyButton(FancyButton.PressType.LongPress);
        takePhoto = new FancyButton(FancyButton.PressType.LongPress);
        movePoseUp = new FancyButton(FancyButton.PressType.LongPress);
        movePoseDown = new FancyButton(FancyButton.PressType.LongPress);
        movePoseLeft = new FancyButton(FancyButton.PressType.LongPress);
        movePoseRight = new FancyButton(FancyButton.PressType.LongPress);
        cycleCycler = new FancyButton(FancyButton.PressType.Toggle);
        useCycleCycler = new FancyButton(FancyButton.PressType.Toggle);

        manualTurretStart = new FancyButton(FancyButton.PressType.Toggle);

        robot.follower.setPose(startPose);
        time = new ElapsedTime();
    }



    public void init_loop() {
        resetPose.checkStatus(gamepad2.b || gamepad1.b);
        manualTurretStart.checkStatus(gamepad2.a || gamepad1.a);
        if (resetPose.startPress) {
            startPose = new Pose();
            robot.follower.setPose(startPose);
            robot.turret.encoderOffset = 12830;
            robot.indexer.encoderOffsetFromAuto = 0;
            robot.indexer.setIndexerPos(0);
        }

        if (manualTurretStart.isOn) {
            robot.turret.setPower((gamepad2.right_stick_x + gamepad1.right_stick_x) * 0.3);
        }
        if (manualTurretStart.endPress) {
            robot.turret.encoderOffset = 12830;
            robot.turret.reZero();
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
        brake.checkStatus(gamepad1.right_bumper); // hold to turn on brake
        unjamSpindexer.checkStatus(gamepad1.right_trigger_pressed || gamepad2.right_trigger_pressed);
        intake.checkStatus(gamepad2.left_bumper); // Toggle on to intake
        spitIntake.checkStatus(gamepad2.right_bumper || gamepad1.left_bumper); // Hold to spit

//        turnSOTFOn.checkStatus(gamepad1.left_stick_button); // Toggle on SOTF
//        turnSOTFOff.checkStatus(gamepad1.right_stick_button); // Toggle off SOTF
        turnSOTFOn.checkStatus(false); // Toggle on SOTF
        turnSOTFOff.checkStatus(false); // Toggle off SOTF

        queueGreen.checkStatus(gamepad2.y); // Press to queue green
        queuePurple.checkStatus(gamepad2.x); // Press to queue purple
        autoShoot.checkStatus(gamepad2.a); // Toggle to turn on auto shoot
        fastShootButton.checkStatus(gamepad2.b || gamepad1.b); // press to shoot 3
        smartShoot.checkStatus(gamepad2.back); // Toggle to turn on smart shoot

        overrideIntake.checkStatus(gamepad2.left_trigger_pressed || gamepad1.left_trigger_pressed); // hold to turn on ignore allowintaking
        panicShoot.checkStatus(gamepad2.ps); // toggle to turn on panic shoot
        reZeroIndexer.checkStatus(gamepad1.ps);

        movePoseUp.checkStatus(gamepad2.dpad_up);
        movePoseDown.checkStatus(gamepad2.dpad_down);  //Buttons to control where the robot aims
        movePoseLeft.checkStatus(gamepad2.dpad_left);
        movePoseRight.checkStatus(gamepad2.dpad_right);

        takePhoto.checkStatus(gamepad1.y); // press to take photo
        debug.checkStatus(gamepad1.start); // toggle to print telemetry
        resetPose.checkStatus(gamepad1.x);
        climb.checkStatus(gamepad1.back);
        cycleCycler.checkStatus(robot.indexer.isHasBallsEmpty());
        useCycleCycler.checkStatus(gamepad1.dpad_left);

        if (!brake.isOn) {
            robot.drivetrain.teleopMovement(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, true);
        } else {
            robot.breakDrivetrain();
        }

        if (turnSOTFOn.startPress) {
            sotfIsOn = true;
        }
        if (turnSOTFOff.startPress) {
            sotfIsOn = false;
        }

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

        if (autoShoot.isOn && !climb.isOn) {
            robot.prepareShooter(ShotType.TABLE, !brake.isOn && sotfIsOn, cycleCycler.isOn && useCycleCycler.isOn);
        } else if (autoShoot.endPress || climb.startPress) {
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
        robot.shooterTrim(movePoseUp.startPress, movePoseDown.startPress, movePoseLeft.startPress, movePoseRight.startPress, resetPose.startPress);


        robot.setPanicShoot(panicShoot.isOn, fastShootButton.isOn);

        if (reZeroIndexer.startPress) {
            robot.reZeroIndexer();
        }

        if (climb.startPress) {
            robot.climb();
        } else if (climb.endPress) {
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
