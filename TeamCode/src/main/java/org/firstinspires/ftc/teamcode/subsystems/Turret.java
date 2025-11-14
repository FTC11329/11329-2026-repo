package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Turret {
    // declaring motor variables
    Servo turretServo1;
    Servo turretServo2;

    double hoodPos = 0;

    private DcMotor encoder;

    double position;

    public Turret(HardwareMap hardwareMap){

        turretServo1 = hardwareMap.get(Servo.class, "turret1");
        turretServo1.setDirection(Servo.Direction.FORWARD);
        turretServo1.setPosition(0);

        turretServo2 = hardwareMap.get(Servo.class, "turret2");
        turretServo2.setDirection(Servo.Direction.REVERSE);
        turretServo2.setPosition(0);

        encoder = hardwareMap.get(DcMotor.class, "encoder");

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void turnTo(double degrees){

    }
}
