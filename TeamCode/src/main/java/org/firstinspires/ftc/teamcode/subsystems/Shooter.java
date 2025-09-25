package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    // declaring motor variables
    DcMotorEx wheel;

    public Shooter(HardwareMap hardwaremap){
        wheel = hardwaremap.get(DcMotorEx.class, "wheel");
    }

    public void setWheel1Power (double set){
        wheel.setPower(set);
    }
}
