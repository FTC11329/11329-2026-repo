package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CentralArmController {
    // declaring motor variables

    DcMotorEx arm;
    Servo claw;
    Servo wrist1;
    Servo wrist;

    public CentralArmController(HardwareMap hardwaremap){
        arm = hardwaremap.get(DcMotorEx.class, "arm");
        claw = hardwaremap.get(Servo.class, "claw");
        wrist1 = hardwaremap.get(Servo.class, "wrist");
        wrist = hardwaremap.get(Servo.class, "wrist");
    }

    public void setarmPower(double set){
        arm.setPower(set);
    }
    public void setclawPower(double set){
        claw.setPosition(set);
    }
    public void setwristPower(double set){
        wrist1.setPosition(set);
        wrist.setPosition(-set);
    }

    public void teleopArmMovement(double forwardBackward, double openClose, double upDown){
        arm.setPower(forwardBackward);
        claw.setPosition(openClose);
        wrist.setPosition(upDown);
    }
}
