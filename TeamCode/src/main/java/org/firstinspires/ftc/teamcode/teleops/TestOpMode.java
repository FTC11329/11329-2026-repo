package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTag;
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

    Pose robotPose;
    FancyButton toggle;
    FancyButton toggle2;

    @Override
    public void init() {
        //do stuff init
        robot = new Robot(telemetry, hardwareMap, RobotSide.Blue);
        toggle = new FancyButton(FancyButton.PressType.Toggle);
        toggle2 = new FancyButton(FancyButton.PressType.Toggle);
    }

    @Override
    public void start(){
        time = new ElapsedTime();
        time.reset();
        lastTime = time.milliseconds();
    }

    @Override
    public void loop() {
//        robot.update();
        robot.drivetrain.teleopMovement(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, gamepad1.right_bumper);
        toggle.checkStatus(gamepad1.a);
        toggle2.checkStatus(gamepad1.left_bumper);
//        robotPose = robot.getCurrentPose();
        if (toggle2.startPress) {
            robot.indexer.transfer(true);
        } else if (toggle2.endPress) {
            robot.indexer.transfer(false);
        }
        if (toggle.startPress) {
            robot.shooter.setPower(1);
            robot.indexer.setIndexerPower(1);
            robot.intake.setIntakePower(1);
        } else if (toggle.endPress) {
            robot.shooter.setPower(0);
            robot.indexer.setIndexerPower(0);
            robot.intake.setIntakePower(0);
        }

        if (gamepad1.b) {
            robot.shooter.setHoodDeg(60);

        } else if (gamepad1.x) {
            robot.shooter.setHoodDeg(40);

        } else if (gamepad1.y) {
            robot.shooter.setHoodDeg(20);
        } else {
            robot.shooter.setHoodDeg(0);
        }
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
