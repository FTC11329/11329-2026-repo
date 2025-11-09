package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp(name = "Main Teleop", group = "group")
public class MainTeleop extends OpMode {
    //This is where we introduce the tele-operated controls
    Intake arm;
    Drivetrain drivetrain;

    @Override
    public void init() {
        //do stuff init
        arm = new Intake(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);

    }

    @Override
    public void loop() {
        //do stuff always start

        telemetry.addLine("driveTrain");
        telemetry.addData("arm", true);
        telemetry.update();
    }

}
