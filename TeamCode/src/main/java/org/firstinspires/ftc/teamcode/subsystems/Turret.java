package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Turret {
    // declaring motor variables
    CRServo turretServo1;
    CRServo turretServo2;

    double hoodPos = 0;

    private DcMotor encoder;

    double position;

    public Turret(HardwareMap hardwareMap){

        turretServo1 = hardwareMap.get(CRServo.class, "turret1");
        turretServo1.setDirection(CRServo.Direction.FORWARD);

        turretServo2 = hardwareMap.get(CRServo.class, "turret2");
        turretServo2.setDirection(CRServo.Direction.FORWARD);

//        encoder = hardwareMap.get(DcMotor.class, "encoder");
//
//        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void turnTo(double degrees) {

    }

    public void setPower(double set) {
        turretServo1.setPower(set);
        turretServo2.setPower(set);
    }

}
