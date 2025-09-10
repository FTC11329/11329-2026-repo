package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class indexerControllerGeneral {
    // declaring motor variables

    DcMotorEx spindexer;
    Servo claw;
    Servo wrist1;
    Servo wrist2;

    public void indexControllerGeneral(HardwareMap hardwaremap){
        spindexer = hardwaremap.get(DcMotorEx.class, "wheel1");
        claw = hardwaremap.get(Servo.class, "claw");
        wrist1 = hardwaremap.get(Servo.class, "wrist1");
        wrist2 = hardwaremap.get(Servo.class, "wrist2");
    }

    public void setSpindexerPower(double set){
        spindexer.setPower(set);
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

    public void teleopIndexMovement(double spindexPower, double clawPos, double wristPos){
        setSpindexerPower(spindexPower);
        setClawPos(clawPos);
        setWristPos(wristPos);
    }
}