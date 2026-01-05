package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.RobotSide;

@TeleOp(name = "TEST", group = "       group")
public class TemporaryClassThatWeAreTotallyGoingToDelete extends OpMode {
    CRServo servo1;
    CRServo servo2;
    DcMotorSimple motor1;
    DcMotorSimple motor2;
    Robot robot;
    double power = 1;

    @Override
    public void init() {
        robot = new Robot(telemetry, hardwareMap, RobotSide.Blue, 0, 0);

        motor1 = hardwareMap.get(DcMotorSimple.class, "flywheel");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        motor2 = hardwareMap.get(DcMotorSimple.class, "transfer");
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        motor1.setPower(1);
        motor2.setPower(1);
    }

    @Override
    public void loop() {
        motor1.setPower(1);
        motor2.setPower(1);

        robot.indexer.indexerState.averageTime();

    }
}
