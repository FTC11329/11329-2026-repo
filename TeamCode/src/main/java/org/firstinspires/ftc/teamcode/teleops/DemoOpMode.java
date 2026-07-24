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
import org.firstinspires.ftc.teamcode.util.ShootOnTheFly.ShotType;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class DemoOpMode {
    private static final Logger log = LoggerFactory.getLogger(DemoOpMode.class);
    //This is where we introduce the teleoperated controls
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
    FancyButton queueRed;
    FancyButton queueOrange;
    FancyButton queueYellow;
    FancyButton queueGreen;
    FancyButton queueBlue;
    FancyButton queuePurple;
    FancyButton forceSpit;

    FancyButton overrideShootPosition;
    FancyButton overrideIntake;
    FancyButton panicShoot;

    FancyButton debug;
    FancyButton takePhoto;
    FancyButton movePoseUp;
    FancyButton movePoseDown;
    FancyButton movePoseLeft;
    FancyButton movePoseRight;
    FancyButton resetPoseClose;
    FancyButton resetPoseFar;
    FancyButton unjamSpindexer;
    FancyButton climb;
    FancyButton reZeroIndexer;
    FancyButton cycleCycler;
    FancyButton swapGoal;

    FancyButton manualTurretStart;
    FancyButton reCheckColors;

    Gamepad gamepad1;
    Gamepad gamepad2;
    Telemetry telemetry;
    RobotSide robotSide;
    HardwareMap hardwareMap;
    double RPMoffset;
    boolean sotfIsOn = true;
    boolean brakeAllowSotfIsOn = false;
    boolean brakeAllowSotfDebounce = false;

    public DemoOpMode(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap, RobotSide robotSide) {
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

        queueRed = new FancyButton(FancyButton.PressType.LongPress);;
        queueOrange = new FancyButton(FancyButton.PressType.LongPress);;
        queueYellow = new FancyButton(FancyButton.PressType.LongPress);;
        queueGreen = new FancyButton(FancyButton.PressType.LongPress);;
        queueBlue = new FancyButton(FancyButton.PressType.LongPress);;
        queuePurple = new FancyButton(FancyButton.PressType.LongPress);;
        forceSpit = new FancyButton(FancyButton.PressType.LongPress);;
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
        resetPoseClose = new FancyButton(FancyButton.PressType.LongPress);
        resetPoseFar = new FancyButton(FancyButton.PressType.LongPress);
        takePhoto = new FancyButton(FancyButton.PressType.LongPress);
        movePoseUp = new FancyButton(FancyButton.PressType.LongPress);
        movePoseDown = new FancyButton(FancyButton.PressType.LongPress);
        movePoseLeft = new FancyButton(FancyButton.PressType.LongPress);
        movePoseRight = new FancyButton(FancyButton.PressType.LongPress);
        cycleCycler = new FancyButton(FancyButton.PressType.Toggle);
        swapGoal = new FancyButton(FancyButton.PressType.Toggle);
        reCheckColors = new FancyButton(FancyButton.PressType.LongPress);

        manualTurretStart = new FancyButton(FancyButton.PressType.Toggle);

        robot.follower.setPose(startPose);
        time = new ElapsedTime();
    }



    public void init_loop() {
        resetPoseClose.checkStatus(gamepad2.b || gamepad1.b);
        manualTurretStart.checkStatus(gamepad2.a || gamepad1.a);
        if (resetPoseClose.startPress) {
            startPose = new Pose();
            robot.follower.setPose(startPose);
            robot.turret.encoderOffset = 12830;
            robot.turret.setTargetDeg(0);
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
        unjamSpindexer.checkStatus(gamepad1.right_trigger_pressed);
        intake.checkStatus(gamepad1.a); // Toggle on to intake
        spitIntake.checkStatus(gamepad1.left_bumper); // Hold to spit

//        turnSOTFOn.chec2kStatus(gamepad1.left_stick_button); // Toggle on SOTF
//        turnSOTFOff.checkStatus(gamepad1.right_stick_button); // Toggle off SOTF
        turnSOTFOn.checkStatus(false); // Toggle on SOTF
        turnSOTFOff.checkStatus(false); // Toggle off SOTF

        boolean red = false;
        boolean orange = false;
        boolean yellow = false;
        boolean green = false;
        boolean blue = false;
        boolean purple = false;

        if (gamepad1.touchpad) {
            if (gamepad1.touchpad_finger_1_y > 0) {
                if (gamepad1.touchpad_finger_1_x < -0.66){
                    red = true;
                } else if (gamepad1.touchpad_finger_1_x > 0.66) {
                    yellow = true;
                } else {
                    orange = true;
                }
            } else {
                if (gamepad1.touchpad_finger_1_x < -0.66){
                    green = true;
                } else if (gamepad1.touchpad_finger_1_x > 0.66) {
                    purple = true;
                } else {
                    blue = true;
                }
            }
        }
        queueRed.checkStatus   (red && smartShoot.isOn);
        queueOrange.checkStatus(orange && smartShoot.isOn);
        queueYellow.checkStatus(yellow && smartShoot.isOn);
        queueGreen.checkStatus (green && smartShoot.isOn);
        queueBlue.checkStatus  (blue && smartShoot.isOn);
        queuePurple.checkStatus(purple && smartShoot.isOn);

        autoShoot.checkStatus(gamepad1.a); // Toggle to turn on auto shoot
        fastShootButton.checkStatus(gamepad1.b);// press to shoot 3
        smartShoot.checkStatus(gamepad1.ps); // Toggle to turn on smart shoot

        overrideIntake.checkStatus(gamepad1.left_trigger_pressed); // hold to turn on ignore allowintaking
        panicShoot.checkStatus(false); // toggle to turn on panic shoot
        reZeroIndexer.checkStatus(false);
        forceSpit.checkStatus(gamepad1.left_bumper);

        movePoseUp.checkStatus(gamepad1.dpad_up);
        movePoseDown.checkStatus(gamepad1.dpad_down);  //Buttons to control where the robot aims
        movePoseLeft.checkStatus(gamepad1.dpad_left);
        movePoseRight.checkStatus(gamepad1.dpad_right);

        takePhoto.checkStatus(false); // press to take photo
        debug.checkStatus(false); // toggle to print telemetry
        resetPoseClose.checkStatus(gamepad1.x);
        resetPoseFar.checkStatus(gamepad1.y);
        climb.checkStatus(gamepad1.back);
//        cycleCycler.checkStatus(robot.indexer.isHasBallsEmpty());
        reCheckColors.checkStatus(gamepad2.left_stick_button);
        swapGoal.checkStatus(false);

        if (brake.endPress) {
            brakeAllowSotfIsOn = false;
        }
        if (brake.isOn && robot.follower.getVelocity().getMagnitude() < 0.1) {
            brakeAllowSotfIsOn = true;
        }

        if (!brake.isOn) {
            if ((Math.abs(gamepad1.left_stick_y) > 0.01) || (Math.abs(gamepad1.left_stick_x) > 0.01) || (Math.abs(gamepad1.right_stick_x) > 0.01)){
                robot.drivetrain.teleopMovement(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, true);
                robot.notBrakeDriveTrain();
            } else {
                robot.drivetrain.teleopMovement(-gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x, true);
                robot.notBrakeDriveTrain();
            }
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

        if (queueRed.startPress) {
            robot.qBall(BallColor.Red);
        }
        if (queueOrange.startPress) {
            robot.qBall(BallColor.Orange);
        }
        if (queueYellow.startPress) {
            robot.qBall(BallColor.Yellow);
        }
        if (queueGreen.startPress) {
            robot.qBall(BallColor.Green);
        }
        if (queueBlue.startPress) {
            robot.qBall(BallColor.Blue);
        }
        if (queuePurple.startPress) {
            robot.qBall(BallColor.Purple);
        }

        robot.doSmartShoot(smartShoot.isOn);

        if (fastShootButton.startPress) {
            robot.shootAll();
        } else if (reCheckColors.startPress) {
            robot.indexer.reReadHasBalls();
        }

        if (autoShoot.isOn && !climb.isOn) {
            robot.prepareShooter(ShotType.TABLE, (!brake.isOn || brakeAllowSotfIsOn) && sotfIsOn, swapGoal.isOn, false);
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
        robot.shooterTrim(movePoseUp.startPress, movePoseDown.startPress, movePoseLeft.startPress, movePoseRight.startPress, panicShoot.isOn);

        if (resetPoseClose.startPress) {
            robot.reZero48();
        } else if (resetPoseFar.startPress) {
            robot.reZero72();
        }

        robot.setPanicShoot(panicShoot.isOn, fastShootButton.isOn);

        if (reZeroIndexer.startPress) {
            robot.reZeroIndexer();
        }
        robot.setTeleopSpit(forceSpit.isOn);

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
