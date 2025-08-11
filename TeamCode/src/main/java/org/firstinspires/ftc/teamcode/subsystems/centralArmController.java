package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class centralArmController {
    // declaring motor variables

    DcMotorEx arm;
    Servo claw;

    public centralArmController(HardwareMap hardwaremap){
        arm = hardwaremap.get(DcMotorEx.class, "arm");
        claw = hardwaremap.get(Servo.class, "claw");
    }

    public void setPower(double set){
        arm.setPower(set);
        claw.setPosition(set);
    }
    public void teleopArmMovement(double forward, double backward, double open, double close){
        arm.setPower(forward - backward);
        claw.setPosition(open - close);
    }
}
