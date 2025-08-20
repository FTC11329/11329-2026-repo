package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CentralArmController;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainMain;

@TeleOp(name = "Main Releop", group = "grop")
public class MainTeleop extends OpMode {
    //This is where we introduce the tele-operated controls
    CentralArmController arm;
    DrivetrainMain train;
    @Override
    public void init() {
        //do stuff init
        arm = new CentralArmController(hardwareMap);
        train = new DrivetrainMain(hardwareMap);
    }

    @Override
    public void loop() {
        //do stuff always start
        telemetry.addLine("bine");
        telemetry.addData("jarm", true);
        telemetry.update();
    }

}
