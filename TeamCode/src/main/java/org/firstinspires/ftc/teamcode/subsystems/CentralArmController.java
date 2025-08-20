package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CentralArmController {
    // declaring motor variables

    DcMotorEx arm;
    Servo claw;
    Servo wrist;

    public CentralArmController(HardwareMap hardwaremap){
        arm = hardwaremap.get(DcMotorEx.class, "arm");
        claw = hardwaremap.get(Servo.class, "claw");
        wrist = hardwaremap.get(Servo.class, "wrist");
    }

    public void setarmPower(double set){
        arm.setPower(set);
    }
    public void setclawPower(double set){
        claw.setPosition(set);
    }
    public void setwristPower(double set){
        wrist.setPosition(set);
    }

    public void teleopArmMovement(double forward, double backward, double open, double close, double up, double down){
        arm.setPower(forward - backward);
        claw.setPosition(open - close);
        wrist.setPosition(up - down);
    }
}
