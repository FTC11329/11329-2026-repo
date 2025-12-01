package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.FancyButton;
import org.firstinspires.ftc.teamcode.util.RobotSide;
//  Shooting logic (in psudo code)
//
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


    @Override
    public void init() {
        robot = new Robot(telemetry, hardwareMap, RobotSide.Blue);

        intake = new FancyButton(FancyButton.PressType.Toggle);
        spitIntake = new FancyButton(FancyButton.PressType.LongPress);
        queueGreen = new FancyButton(FancyButton.PressType.LongPress);
        queuePurple = new FancyButton(FancyButton.PressType.LongPress);
        autoShoot = new FancyButton(FancyButton.PressType.Toggle);
    }

    @Override
    public void loop() {
        intake.checkStatus(gamepad1.left_bumper); // Toggle on to intake
        spitIntake.checkStatus(gamepad1.b); // Hold to spit
        queueGreen.checkStatus(gamepad1.y); // Press to queue green
        queuePurple.checkStatus(gamepad1.x); // Press to queue purple
        autoShoot.checkStatus(gamepad1.a); // Toggle to turn on auto shoot

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
            robot.shootQueue();
        }
        if (queuePurple.startPress) {
            robot.qBall(BallColor.Purple);
        }
        if (queueGreen.startPress) {
            robot.qBall(BallColor.Green);
        }

        robot.update();


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
//        telemetry.addData("Encoder RPM", robot.shooter.getRPM());
//
//        telemetry.addData("turret power", gamepad2.right_trigger - gamepad2.left_trigger);
    }
    @Override
    public void stop() {
        robot.stopAllSubsystems();
    }
}
