package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CentralArmController {
    // declaring motor variables

    DcMotorEx arm;
    Servo claw;
    Servo wrist1;
    Servo wrist2;

    public CentralArmController(HardwareMap hardwaremap){
        arm = hardwaremap.get(DcMotorEx.class, "arm");
        claw = hardwaremap.get(Servo.class, "claw");
        wrist1 = hardwaremap.get(Servo.class, "wrist1");
        wrist2 = hardwaremap.get(Servo.class, "wrist2");
    }

    public void setArmPower(double set){
        arm.setPower(set);
    }
    public void setClaw(boolean set){
        if (set) {
            setClawPos(1);
        } else {
            setClawPos(0);
        }
    }

    public void setClawPos(double set){
        claw.setPosition(set);
    }

    public void setWristPos(double set){
        wrist1.setPosition(set);
        wrist2.setPosition(-set);
    }

    public void teleopArmMovement(double armPower, double clawPos, double wristPos){
        setArmPower(armPower);
        setClawPos(clawPos);
        setWristPos(wristPos);
    }

    public void teleopArmMovement(double armPower, boolean clawBool, double wristPos){
        setArmPower(armPower);
        setClaw(clawBool);
        setWristPos(wristPos);
    }
}
