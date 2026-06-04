package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestOpMode", group = "          ") // more spaces means higher up on the teleop list
public class SimpleOpMode extends OpMode {

    DcMotorEx transferMotor;
    Servo indexer;
    DcMotorEx shooter;

    double startPos = 0;
    double endPos = 1;


    @Override
    public void init() {
        // name of intake motor is "intake" and it's direction is FORWARD
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        indexer = hardwareMap.get(Servo.class, "spindexer1");
        indexer.setPosition(startPos);

        shooter = hardwareMap.get(DcMotorEx.class, "flywheel1");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

        transferMotor.setPower(-gamepad1.left_stick_y);

        if (gamepad1.a) {
            indexer.setPosition(endPos);
        } else if (gamepad1.b) {
            indexer.setPosition(startPos);
        }

        double input = gamepad1.right_trigger - gamepad1.left_trigger;
        shooter.setPower(input);
    }
}
