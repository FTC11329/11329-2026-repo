package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;

@TeleOp(name = "TEstuingiuwenio", group = "zgroup")
public class TemporaryClassThatWeAreTotallyGoingToDelete extends OpMode {
    Indexer indexer;
    @Override
    public void init() {
       indexer = new Indexer(hardwareMap);
    }

    @Override
    public void loop() {
        double power1 = -gamepad1.right_stick_y;
        double power3 = -gamepad1.left_stick_y;
        telemetry.addData("pow1", power1);
        telemetry.addData("pow3", power3);
        indexer.setMotor(power1);
        indexer.setIndexerPower(power3);
    }
}
