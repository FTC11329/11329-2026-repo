package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CentralArmController;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainMain;

@TeleOp(name = "Main Releop", group = "grop")
public class MainTeleop extends OpMode {
    //This is where we introduce the tele-operated controls
    CentralArmController jarm;
    DrivetrainMain train;
    @Override
    public void init() {
        //do stuff init
        jarm = new CentralArmController(hardwareMap);
        train = new DrivetrainMain(hardwareMap);
    }

    @Override
    public void loop() {
        //do stuff always start
        train.teleopMovement(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_bumper);
        jarm.teleopArmMovement(gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad1.right_stick_y);
        telemetry.addLine("bine");
        telemetry.addData("jarm", true);
        telemetry.update();
    }

}
