package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TEST", group = "       group")
public class TemporaryClassThatWeAreTotallyGoingToDelete extends OpMode {
    CRServo servo1;
    CRServo servo2;
    DcMotorSimple motor1;
    DcMotorSimple motor2;
    double power = 1;

    @Override
    public void init() {
//        servo1 = hardwareMap.get(CRServo.class, "spindexer1");
//        servo2 = hardwareMap.get(CRServo.class, "spindexer2");
//        servo1.setDirection(CRServo.Direction.REVERSE);
//        servo2.setDirection(CRServo.Direction.REVERSE);

        motor1 = hardwareMap.get(DcMotorSimple.class, "flywheel");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        motor2 = hardwareMap.get(DcMotorSimple.class, "transfer");
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        motor1.setPower(0.8);
        motor2.setPower(0.8);
    }

    @Override
    public void loop() {
        telemetry.addData("Pos", power);
        servo1.setPower(power);
        servo2.setPower(power);
    }
}
