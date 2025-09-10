package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class shooterControllerGeneral {
    // declaring motor variables

    DcMotorEx wheel1;
    DcMotorEx wheelTwo;
    Servo claw;
    Servo wrist1;
    Servo wrist2;

    public void shootethControllerGeneral(HardwareMap hardwaremap){
        wheel1 = hardwaremap.get(DcMotorEx.class, "wheel1");
        wheelTwo = hardwaremap.get(DcMotorEx.class, "wheel1");
        claw = hardwaremap.get(Servo.class, "claw");
        wrist1 = hardwaremap.get(Servo.class, "wrist1");
        wrist2 = hardwaremap.get(Servo.class, "wrist2");
    }

    public void setWheel1Power (double set){
        wheel1.setPower(set);
    }
    public void setWheelTwoPower (double set){
        wheelTwo.setPower(set);
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

    public void teleopArmMovement(double wheel1Power, double wheelTwoPower, double clawPos, double wristPos){
        setWheel1Power(wheel1Power);
        setWheelTwoPower(wheelTwoPower);
        setClawPos(clawPos);
        setWristPos(wristPos);
    }

}
