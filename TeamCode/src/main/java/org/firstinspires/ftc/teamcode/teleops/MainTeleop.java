package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

@TeleOp(name = "Main Teleop", group = "                                                      group")
public class MainTeleop extends OpMode {
    //This is where we introduce the tele-operated controls
    Robot robot;
    // PressHolds
    FancyButton intake;
    FancyButton spitIntake;
    FancyButton autoShoot;
    FancyButton queueGreen;
    FancyButton queuePurple;
    FancyButton overrideShootPosition;
    FancyButton debug;
    FancyButton fastChangeInit;
    FancyButton movePoseUpInit;
    FancyButton movePoseDownInit;
    FancyButton movePoseLeftInit;
    FancyButton movePoseRightInit;
    FancyButton rotatePoseRightInit;
    FancyButton rotatePoseLeftInit;

    public double hoodAngle = 20;
    public double rpm = 3000;
    public Pose startPose;
    @Override
    public void init() {
        //todo add logic to get position at the end of auto
        startPose = new Pose(0,0,0);

        robot = new Robot(telemetry, hardwareMap, RobotSide.Blue);

        intake = new FancyButton(FancyButton.PressType.Toggle);
        spitIntake = new FancyButton(FancyButton.PressType.LongPress);
        queueGreen = new FancyButton(FancyButton.PressType.LongPress);
        queuePurple = new FancyButton(FancyButton.PressType.LongPress);
        autoShoot = new FancyButton(FancyButton.PressType.Toggle);
        overrideShootPosition = new FancyButton(FancyButton.PressType.LongPress);
        debug = new FancyButton(FancyButton.PressType.Toggle);
        fastChangeInit = new FancyButton(FancyButton.PressType.LongPress);
        movePoseUpInit = new FancyButton(FancyButton.PressType.LongPress);
        movePoseDownInit = new FancyButton(FancyButton.PressType.LongPress);
        movePoseLeftInit = new FancyButton(FancyButton.PressType.LongPress);
        movePoseRightInit = new FancyButton(FancyButton.PressType.LongPress);
        rotatePoseLeftInit = new FancyButton(FancyButton.PressType.LongPress);
        rotatePoseRightInit = new FancyButton(FancyButton.PressType.LongPress);
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Use gamepad 2 Dpad to change Start Position");

        fastChangeInit.checkStatus(gamepad2.right_bumper);
        movePoseUpInit.checkStatus(gamepad2.dpad_up);
        movePoseDownInit.checkStatus(gamepad2.dpad_down);
        movePoseLeftInit.checkStatus(gamepad2.dpad_left);
        movePoseRightInit.checkStatus(gamepad2.dpad_right);
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

        if (RobotSide == RobotSide.Blue) {
            if (movePoseUpInit.startPress) {
                startPose.addY(-moveSpeed);
            }
            if (movePoseDownInit.startPress) {
                startPose.addY(moveSpeed);
            }
            if (movePoseLeftInit.startPress) {
                startPose.addX(moveSpeed);
            }
            if (movePoseRightInit.startPress) {
                startPose.addX(-moveSpeed);
            }
        } else {
            if (movePoseUpInit.startPress) {
                startPose.addY(moveSpeed);
            }
            if (movePoseDownInit.startPress) {
                startPose.addY(-moveSpeed);
            }
            if (movePoseLeftInit.startPress) {
                startPose.addX(-moveSpeed);
            }
            if (movePoseRightInit.startPress) {
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

    @Override public void start() {
        robot.follower.setStartingPose(startPose);
    }

    @Override
    public void loop() {
        intake.checkStatus(gamepad1.left_bumper); // Toggle on to intake
        spitIntake.checkStatus(gamepad1.b); // Hold to spit

        queueGreen.checkStatus(gamepad1.y); // Press to queue green
        queuePurple.checkStatus(gamepad1.x); // Press to queue purple
        autoShoot.checkStatus(gamepad1.a); // Toggle to turn on auto shoot
        overrideShootPosition.checkStatus(gamepad1.back); // hold to turn on ignore position
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
            if (intake.isOn){
                // Makes the intake toggle correct
                intake.checkStatus(false);
                intake.checkStatus(true);
                intake.checkStatus(false);
            }
        }
        if (spitIntake.endPress) {
            robot.stopIntake();
        }

        if (autoShoot.isOn) {
            robot.prepareShooter();
            robot.shootQueue(overrideShootPosition.isOn);
        } else if (autoShoot.endPress){
            robot.shooter.casualModeOn();
            robot.stopIndexer();
        }
        if (queuePurple.startPress) {
            robot.qBall(BallColor.Purple);
        }
        if (queueGreen.startPress) {
            robot.qBall(BallColor.Green);
        }

        robot.update(debug.isOn);

//        robot.update();
//        shoot.checkStatus(gamepad1.x);
//        if (shoot.isOn && intake.isOn){
//            intake.checkStatus(false);
//            intake.checkStatus(true);
//            intake.checkStatus(false);
//        }else {
//            intake.checkStatus(gamepad1.a);
//        }
//        robot.drivetrain.teleopMovement(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.left_bumper);
//
//        //TURRET : 2 x
//        robot.turret.setPower(gamepad2.left_stick_x);
//        telemetry.addData("turret velocity encoder", robot.turret.encoder.getVelocity());
//        telemetry.addData("turret position encoder", robot.turret.encoder.getCurrentPosition());
//
//        //INTAKE : a
//        robot.intake.setIntakePower(gamepad1.a ? 1 : 0);
//
//        //INDEXER : b
//        robot.indexer.setIndexerPower(gamepad1.b ? 1 : 0);
//
//        //SHOOTER : y
//        if (gamepad1.y){
//            robot.passiveShoot(6000, false);
//        }
//
//        //Shooter 2 : 2 a, and y
//        robot.shooter.setPower(gamepad2.a ? 1 : 0);
//
//        telemetry.addData("Spindexer Power", -gamepad1.left_stick_y);
//
//        //TRANSFER
//        robot.indexer.setIndexerToShooterPower(gamepad1.x ? 1 : -gamepad2.right_stick_x);
//
//        telemetry.addData("indexer to shooter power", -gamepad2.right_stick_x);
//
//
//        telemetry.addData("intake Power", gamepad1.right_trigger - gamepad1.left_trigger);
//
//        telemetry.addData("shooter power", -gamepad2.left_stick_y);
//
//        angle += -gamepad2.right_stick_y * 0.5;
//        robot.shooter.setHoodDeg(angle);
//        telemetry.addData("shooter angle", angle);
//
//
//        telemetry.addData("current", robot.shooter.flywheel.getCurrent(CurrentUnit.AMPS));
//
        telemetry.addData("Encoder RPM", robot.shooter.getRPM());
        telemetry.addData("Hood angle", robot.shooter.getHoodPosDeg());
//
//        telemetry.addData("turret power", gamepad2.right_trigger - gamepad2.left_trigger);
    }
    @Override
    public void stop() {
        robot.stopAllSubsystems();
    }
}
