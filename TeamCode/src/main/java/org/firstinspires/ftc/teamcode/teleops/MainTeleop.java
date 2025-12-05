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
import org.firstinspires.ftc.teamcode.util.FancyButton;
import org.firstinspires.ftc.teamcode.util.RobotSide;

//  Shooting logic (in psudo code)
//  if (togg) {
//      spin up
//  }
//  if ((!failsafeToggle) && (togg and (inShootZone || overrideButton) ) ) {
//      if ((purple or green) in queue) {
//          shoot queue
//      } else {
//          shoot any
//      }
//  } else if (failsafeToggle) {
//      if (gamepad2.a) {
//          spin indexer
//          spin shooter(2000rpm)
//          setHood (10deg)
//      }
//      if (gamepad2.back) {
//          spin transfer
//      } else {
//          don't spin transfer
//      }
//   }

public class MainTeleop {
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
    FancyButton fastChangeInit;
    FancyButton movePoseUp;
    FancyButton movePoseDown;
    FancyButton movePoseLeft;
    FancyButton movePoseRight;
    FancyButton rotatePoseRightInit;
    FancyButton rotatePoseLeftInit;

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
        //todo add logic to get position at the end of auto
        startPose = new Pose(0,0,0);

        robot = new Robot(telemetry, hardwareMap, robotSide);

        intake = new FancyButton(FancyButton.PressType.Toggle);
        spitIntake = new FancyButton(FancyButton.PressType.LongPress);

        queueGreen = new FancyButton(FancyButton.PressType.LongPress);
        queuePurple = new FancyButton(FancyButton.PressType.LongPress);
        autoShoot = new FancyButton(FancyButton.PressType.Toggle);

        overrideShootPosition = new FancyButton(FancyButton.PressType.LongPress);
        panicShoot = new FancyButton(FancyButton.PressType.Toggle);

        takePhoto = new FancyButton(FancyButton.PressType.LongPress);
        debug = new FancyButton(FancyButton.PressType.Toggle);

        takePhoto = new FancyButton(FancyButton.PressType.LongPress);
        fastChangeInit = new FancyButton(FancyButton.PressType.LongPress);
        movePoseUp = new FancyButton(FancyButton.PressType.LongPress);
        movePoseDown = new FancyButton(FancyButton.PressType.LongPress);
        movePoseLeft = new FancyButton(FancyButton.PressType.LongPress);
        movePoseRight = new FancyButton(FancyButton.PressType.LongPress);
        rotatePoseLeftInit = new FancyButton(FancyButton.PressType.LongPress);
        rotatePoseRightInit = new FancyButton(FancyButton.PressType.LongPress);

        time = new ElapsedTime();
    }

    public void init_loop() {
        telemetry.addLine("Use gamepad 2 Dpad to change Start Position");

        fastChangeInit.checkStatus(gamepad2.right_bumper);
        movePoseUp.checkStatus(gamepad2.dpad_up);
        movePoseDown.checkStatus(gamepad2.dpad_down);
        movePoseLeft.checkStatus(gamepad2.dpad_left);
        movePoseRight.checkStatus(gamepad2.dpad_right);
        rotatePoseLeftInit.checkStatus(gamepad2.left_trigger > 0.2);
        rotatePoseRightInit.checkStatus(gamepad2.right_trigger > 0.2);

        double moveSpeed = 0.25; // inches per press
        double rotateSpeed = 5; // degrees per press
        if (fastChangeInit.startPress) {
            moveSpeed = 1;
            rotateSpeed = 45;
        } else if (fastChangeInit.endPress) {
            moveSpeed = 0.25;
            rotateSpeed = 5;
        }

        if (robotSide == RobotSide.Blue) {
            if (movePoseUp.startPress) {
                startPose.addY(-moveSpeed);
            }
            if (movePoseDown.startPress) {
                startPose.addY(moveSpeed);
            }
            if (movePoseLeft.startPress) {
                startPose.addX(moveSpeed);
            }
            if (movePoseRight.startPress) {
                startPose.addX(-moveSpeed);
            }
        } else {
            if (movePoseUp.startPress) {
                startPose.addY(moveSpeed);
            }
            if (movePoseDown.startPress) {
                startPose.addY(-moveSpeed);
            }
            if (movePoseLeft.startPress) {
                startPose.addX(-moveSpeed);
            }
            if (movePoseRight.startPress) {
                startPose.addX(moveSpeed);
            }
        }
        if (rotatePoseRightInit.startPress) {
            startPose.addHeading(-rotateSpeed);
        }
        if (rotatePoseLeftInit.startPress) {
            startPose.addHeading(rotateSpeed);
        }

        telemetry.addData("Start Pose", startPose);
    }

    public void start() {
        robot.follower.setStartingPose(startPose);
    }

    public void loop() {
        intake.checkStatus(gamepad2.left_bumper); // Toggle on to intake
        spitIntake.checkStatus(gamepad1.b || gamepad2.b); // Hold to spit

        // queueGreen.checkStatus(gamepad2.y); // Press to queue green
        // queuePurple.checkStatus(gamepad2.x); // Press to queue purple
        autoShoot.checkStatus(gamepad2.a); // Toggle to turn on auto shoot

        overrideShootPosition.checkStatus(gamepad2.back); // hold to turn on ignore position
        panicShoot.checkStatus(gamepad2.ps); // toggle to turn on panic shoot mode

        movePoseUp.checkStatus(gamepad2.dpad_up);  
        movePoseDown.checkStatus(gamepad2.dpad_down);  //Buttons to control where the robot aims
        movePoseLeft.checkStatus(gamepad2.dpad_left);
        movePoseRight.checkStatus(gamepad2.dpad_right);


        debug.checkStatus(gamepad1.start); // hold to print telemetry


        robot.drivetrain.teleopMovement(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_bumper);


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
                robot.stopIndexer();
            }
        }

        // Panic shoot mode
        if (panicShoot.startPress) {
            // distance 71.6
            robot.shooter.setTargetRPM(2336);
            robot.shooter.setHoodAngleDeg(35);
            robot.turret.setTargetDegree(0);
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
                robot.offsetPose.addY(-1);
            }
            if (movePoseDown.startPress) {
                robot.offsetPose.addY(1);
            }
            if (movePoseLeft.startPress) {
                robot.offsetPose.addX(1);
            }
            if (movePoseRight.startPress) {
                robot.offsetPose.addX(-1);
            }
        } else {
            if (movePoseUp.startPress) {
                robot.offsetPose.addY(1);
            }
            if (movePoseDown.startPress) {
                robot.offsetPose.addY(-1);
            }
            if (movePoseLeft.startPress) {
                robot.offsetPose.addX(-1);
            }
            if (movePoseRight.startPress) {
                robot.offsetPose.addX(1);
            }
        }

        
            
        // if (queuePurple.startPress) {
            // robot.qBall(BallColor.Purple);
        // }
        // if (queueGreen.startPress) {
            // robot.qBall(BallColor.Green);
        // }

        robot.update(debug.isOn);
        telemetry.addData("distance", robot.getCurrentPose().distanceFrom(Constants.Vision.blueGoal));

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
