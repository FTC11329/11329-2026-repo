package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CentralArmController;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainMain;

@TeleOp(name = "Main Teleop", group = "group")
public class MainTeleop extends OpMode {
    //This is where we introduce the tele-operated controls
    CentralArmController arm;
    DrivetrainMain drivetrain;
    @Override
    public void init() {
        //do stuff init
        arm = new CentralArmController(hardwareMap);
        drivetrain = new DrivetrainMain(hardwareMap);
    }

    @Override
    public void loop() {
        //do stuff always start
        drivetrain.teleopMovement(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_bumper);
        arm.teleopArmMovement(gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad1.right_stick_y);
        telemetry.addLine("driveTrain");
        telemetry.addData("arm", true);
        telemetry.update();
    }

}
