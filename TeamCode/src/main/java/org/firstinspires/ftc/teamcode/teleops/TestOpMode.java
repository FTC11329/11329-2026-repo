package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTag;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
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

    Pose robotPose;
    FancyButton toggle;
    FancyButton toggle2;
    FancyButton press1;
    FancyButton press2;
    FancyButton press3;
    FancyButton press4;
    FancyButton press5;

    @Override
    public void init() {
        //do stuff init
        robot = new Robot(telemetry, hardwareMap, RobotSide.Blue);
        toggle = new FancyButton(FancyButton.PressType.Toggle);
        toggle2 = new FancyButton(FancyButton.PressType.Toggle);
        press1 = new FancyButton(FancyButton.PressType.LongPress);
        press2 = new FancyButton(FancyButton.PressType.LongPress);
        press3 = new FancyButton(FancyButton.PressType.LongPress);
        press4 = new FancyButton(FancyButton.PressType.LongPress);
        press5 = new FancyButton(FancyButton.PressType.LongPress);
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
        toggle2.checkStatus(gamepad1.left_bumper);
        press1.checkStatus(gamepad2.dpad_right);
        press2.checkStatus(gamepad2.dpad_left);
        press3.checkStatus(gamepad2.left_stick_button);
        press4.checkStatus(gamepad2.right_bumper);
        press5.checkStatus(gamepad2.left_bumper);

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
            shooterDeg += 0.5;
        } else if (gamepad2.dpad_down) {
            shooterDeg -= 0.5;
        }
        telemetry.addData("shooter Deg", shooterDeg);
        robot.shooter.setHoodDeg(shooterDeg);


//        if (gamepad2.y) {
//            shooterRPM += 2;
//        } else if (gamepad2.a) {
//            shooterRPM -= 2;
//        }
        if (press1.startPress) {
            robot.shooter.setPower(0.9);
        } else if (press2.startPress) {
            robot.shooter.setPower(0);
        }


        robotPose = robot.getCurrentPose();
        lastShooterRPM = shooterRPM;
        robot.update(true);
//
//        if (robotPose != null) {
//            telemetry.addData("Pose", robotPose);
//            telemetry.addData("Distance To Goal", robot.vision.distanceXToGoal(robotPose));
//            telemetry.addData("Velocity", robot.vision.getVelocity());
//            telemetry.addData("RPM", robot.vision.getRPMNeeded());
//        } else {
//            telemetry.addData("Robot Pose", "null");
//            telemetry.addData("Distance To Goal", "null");
//            telemetry.addData("Velocity", "null");
//            telemetry.addData("RPM", "null");
//        }
//
//        deltaTime = time.milliseconds() - lastTime;
//        telemetry.addData("Loop Time", deltaTime);
//        lastTime = time.milliseconds();
    }
}
