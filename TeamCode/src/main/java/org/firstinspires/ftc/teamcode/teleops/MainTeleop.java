package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
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
    FancyButton panicShoot;

    FancyButton debug;
    FancyButton takePhoto;
    FancyButton movePoseUp;
    FancyButton movePoseDown;
    FancyButton movePoseLeft;
    FancyButton movePoseRight;
    FancyButton resetPose;

    FancyButton deleteme;

    Gamepad gamepad1;
    Gamepad gamepad2;
    Telemetry telemetry;
    RobotSide robotSide;
    HardwareMap hardwareMap;

    public MainTeleop(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap, RobotSide robotSide) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
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

//        robot = new Robot(telemetry, hardwareMap, robotSide, startTurretTicks, 0); todo
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
        movePoseUp = new FancyButton(FancyButton.PressType.LongPress);
        movePoseDown = new FancyButton(FancyButton.PressType.LongPress);
        movePoseLeft = new FancyButton(FancyButton.PressType.LongPress);
        movePoseRight = new FancyButton(FancyButton.PressType.LongPress);

        deleteme = new FancyButton(FancyButton.PressType.Toggle);

        time = new ElapsedTime();
    }

    public void init_loop() {
        resetPose.checkStatus(gamepad2.y);

        if (resetPose.startPress || true) {
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
        smartShoot.checkStatus(gamepad2.b); // Toggle to turn on smart shoot
        fastShootButton.checkStatus(gamepad2.right_bumper); // Toggle to turn on smart shoot

        overrideShootPosition.checkStatus(gamepad2.back); // hold to turn on ignore position
        panicShoot.checkStatus(gamepad2.ps); // toggle to turn on panic shoot mode

        resetPose.checkStatus(gamepad2.y);
        movePoseUp.checkStatus(gamepad2.dpad_up);
        movePoseDown.checkStatus(gamepad2.dpad_down);  //Buttons to control where the robot aims
        movePoseLeft.checkStatus(gamepad2.dpad_left);
        movePoseRight.checkStatus(gamepad2.dpad_right);

        takePhoto.checkStatus(gamepad2.y); // hold to take photo
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

//        for (BallColor i : robot.indexer.indexerState.getBallCells()) {
//            telemetry.addData("BALLER", i);
//        }

//        double deltaTime = time.milliseconds() - lastTime;
//        telemetry.addData("loopTimes", deltaTime);
//        lastTime = time.milliseconds();

//        telemetry.addData("distance", robot.getCurrentPose().distanceFrom(Constants.Vision.blueGoal));
//        telemetry.addData("Encoder RPM", robot.shooter.getRPM());
//        telemetry.addData("Hood angle", robot.shooter.getHoodPosDeg());

        /* //todo implement some type of panic shoot before comp
        if (!panicShoot.isOn) {
            if (autoShoot.isOn) {
                robot.prepareShooter();
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
*/
    }

    public void stop() {
        robot.stopAllSubsystems();
    }
}
