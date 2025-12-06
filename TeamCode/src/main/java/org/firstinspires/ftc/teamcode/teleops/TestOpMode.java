package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.control.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.FancyButton;
import org.firstinspires.ftc.teamcode.util.RobotSide;

@TeleOp(name = "Test OpMode", group = "group")
public class TestOpMode extends OpMode {
    //This is where we introduce the tele-operated controls
    Robot robot;
    Shooter shooter;

    ElapsedTime time;
    double deltaTime;
    double lastTime;
    double shooterDeg = 5;
    double lastShooterRPM = 0;
    double shooterRPM = 0;
    double shooterP = Constants.Turret.P;
    double shooterI = Constants.Turret.I;
    double shooterD = Constants.Turret.D;
    double shooterF = 1;

    Pose robotPose;
    FancyButton toggle;
    FancyButton toggle2;
    FancyButton press1;
    FancyButton press2;
    FancyButton press3;
    FancyButton press4;
    FancyButton press5;
    FancyButton press6;
    FancyButton press7;
    FancyButton press8;
    FancyButton press9;
    FancyButton press10;
    FancyButton press11;

    @Override
    public void init() {
        //do stuff init
        robot = new Robot(telemetry, hardwareMap, RobotSide.Blue, 0);
        toggle = new FancyButton(FancyButton.PressType.Toggle);
        toggle2 = new FancyButton(FancyButton.PressType.Toggle);
        press1 = new FancyButton(FancyButton.PressType.LongPress);
        press2 = new FancyButton(FancyButton.PressType.LongPress);
        press3 = new FancyButton(FancyButton.PressType.LongPress);
        press4 = new FancyButton(FancyButton.PressType.LongPress);
        press5 = new FancyButton(FancyButton.PressType.LongPress);
        press6 = new FancyButton(FancyButton.PressType.LongPress);
        press7 = new FancyButton(FancyButton.PressType.LongPress);
        press8 = new FancyButton(FancyButton.PressType.LongPress);
        press9 = new FancyButton(FancyButton.PressType.LongPress);
        press10 = new FancyButton(FancyButton.PressType.LongPress);
        press11 = new FancyButton(FancyButton.PressType.LongPress);
    }

    @Override
    public void start(){
        time = new ElapsedTime();
        time.reset();
        lastTime = time.milliseconds();
    }

    @Override
    public void loop() {
        robot.drivetrain.teleopMovement(-gamepad1.left_stick_y, gamepad1.left_stick_x,  gamepad1.right_stick_x, gamepad1.right_bumper);
        toggle.checkStatus(gamepad1.a);
        toggle2.checkStatus(gamepad1.start);
        press1.checkStatus(gamepad2.dpad_right);
        press2.checkStatus(gamepad2.dpad_left);
        press3.checkStatus(gamepad2.left_stick_button);
        press4.checkStatus(gamepad2.right_bumper);
        press5.checkStatus(gamepad2.left_bumper);
        press6.checkStatus(gamepad2.right_trigger > 0.5);
        press7.checkStatus(gamepad2.left_trigger > 0.5);
        press8.checkStatus(gamepad1.right_bumper);
        press9.checkStatus(gamepad1.left_bumper);
        press10.checkStatus(gamepad1.right_trigger > 0.5);
        press11.checkStatus(gamepad1.left_trigger > 0.5);

        if (toggle2.startPress) {
            robot.indexer.transfer(true);
        } else if (toggle2.endPress) {
            robot.indexer.transfer(false);
        }
        if (toggle.startPress) {
            robot.indexer.setIndexerPower(1);
            robot.intake.setIntakePower(1);
        } else if (toggle.endPress) {
            robot.indexer.setIndexerPower(0);
            robot.intake.setIntakePower(0);
        }
        if (gamepad2.dpad_up) {
            shooterDeg += 0.3;
        } else if (gamepad2.dpad_down) {
            shooterDeg -= 0.3;
        }
        telemetry.addData("shooter Deg", shooterDeg);
        robot.shooter.setHoodDeg(shooterDeg);

        if (gamepad2.y) {
            shooterRPM += 4;
        } else if (gamepad2.a) {
            shooterRPM -= 4;
        }
        if (press1.startPress) {
            shooterRPM += 500;
        } else if (press2.startPress) {
            shooterRPM -= 500;
        }

        if (lastShooterRPM != shooterRPM) {
            robot.setShooterTargetRPM(shooterRPM);
        }
        telemetry.addData("tar shooterRPM", shooterRPM);
        telemetry.addData("Distance", robot.getCurrentPose().distanceFrom(Constants.Vision.blueGoal));


        robotPose = robot.getCurrentPose();
        lastShooterRPM = shooterRPM;
        robot.prepareShooter();
        robot.update(false);

//        deltaTime = time.milliseconds() - lastTime;
//        telemetry.addData("Loop Time", deltaTime);
//        lastTime = time.milliseconds();
        /*
        if (gamepad1.back) {
            shooterP = Constants.Turret.P;
            shooterI = Constants.Turret.I;
            shooterD = Constants.Turret.D;
            shooterF = 1;
        }

        if (press4.startPress) {
            shooterP *= 1.1;
        }
        if (press5.startPress) {
            shooterP /= 1.1;
        }
        if (press6.startPress) {
            shooterI *= 1.1;
        }
        if (press7.startPress) {
            shooterI /= 1.1;
        }
        if (press8.startPress) {
            shooterD *= 1.1;
        }
        if (press9.startPress) {
            shooterD /= 1.1;
        }
        if (press10.startPress) {
            shooterF += 0.01;
        }
        if (press11.startPress) {
            shooterF -= 0.01;
        }

        telemetry.addData("P", shooterP);
        telemetry.addData("I", shooterI);
        telemetry.addData("D", shooterD);
        telemetry.addData("F", shooterF);
        telemetry.addData("PIDF", robot.turret.turretPID.getCoefficients());

        if (press4.endPress || press5.endPress || press6.endPress || press7.endPress || press8.endPress || press9.endPress || press10.endPress || press11.endPress) {
            robot.turret.turretPID.setCoefficients(new PIDFCoefficients(
                    shooterP,
                    shooterI,
                    shooterD,
                    shooterF
            ));
        }

         */
    }
}
