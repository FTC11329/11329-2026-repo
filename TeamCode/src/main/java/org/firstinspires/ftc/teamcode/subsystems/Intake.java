package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.ColorFunctions;

public class Intake {
    // declaring motor variables

    DcMotorEx intakeMotor;

    public Intake(HardwareMap hardwaremap) {
        intakeMotor = hardwaremap.get(DcMotorEx.class, "wheel1");
    }

    public void setintakePower(double set) {
        intakeMotor.setPower(set);
    }

}

