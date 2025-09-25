package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Stilts {
    // declaring motor variables

    DcMotorEx stiltMotor;
    Servo stiltServo;

    public Stilts(HardwareMap hardwaremap){
        stiltMotor = hardwaremap.get(DcMotorEx.class, "wheel1");
        stiltServo = hardwaremap.get(Servo.class, "claw");

    }

    public void setStiltMotorPower(double set){
        stiltMotor.setPower(set);
    }
    public void setStiltServo(boolean set){
        if (set) {
            setStiltPos(1);
        } else {
            setStiltPos(0);
        }
    }

    public void setStiltPos(double set){
        stiltServo.setPosition(set);
    }

    public void teleopArmMovement(double stiltPower, double stiltPos){
        setStiltMotorPower(stiltPower);
        setStiltPos(stiltPos);

    }
}
