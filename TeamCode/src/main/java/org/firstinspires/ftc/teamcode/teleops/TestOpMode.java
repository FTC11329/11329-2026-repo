package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.RobotSide;

@TeleOp(name = "Test OpMode", group = "   group")
public class TestOpMode extends OpMode {
    //This is where we introduce the tele-operated controls
    Vision vision;
    Drivetrain drivetrain;

    ElapsedTime time;
    double deltaTime;
    double lastTime;

    Pose robotPose;

    @Override
    public void init() {
        //do stuff init
        vision = new Vision(hardwareMap, RobotSide.Blue);
        drivetrain = new Drivetrain(hardwareMap);
    }

    @Override
    public void start(){
        time = new ElapsedTime();
        time.reset();
        lastTime = time.milliseconds();
    }

    @Override
    public void loop() {
        drivetrain.teleopMovement(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_y, gamepad1.right_bumper);

        robotPose = vision.getRobotPose();
        if (robotPose != null) {
            telemetry.addData("Robot Pose", robotPose);
            telemetry.addData("Distance To Goal", vision.distanceXToGoal(robotPose));
            telemetry.addData("Velocity", vision.getVelocity());
            telemetry.addData("RPM", vision.getRPMNeeded());
        } else {
            telemetry.addData("Robot Pose", "null");
            telemetry.addData("Distance To Goal", "null");
            telemetry.addData("Velocity", "null");
            telemetry.addData("RPM", "null");
        }

        deltaTime = time.milliseconds() - lastTime;
        telemetry.addData("Loop Time", deltaTime);
        lastTime = time.milliseconds();
    }
}
