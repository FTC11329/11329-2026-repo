package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

public class Intoke {
    // declaring motor variables

    DcMotorEx intakeMotor;
    Servo intakeServo;
    RevColorSensorV3 colorSensor;

    public Intoke(HardwareMap hardwaremap) {
        intakeMotor = hardwaremap.get(DcMotorEx.class, "wheel1");
        intakeServo = hardwaremap.get(Servo.class, "claw");
        colorSensor = hardwaremap.get(RevColorSensorV3.class, "spindexerColorSensor");
    }

    public void setinakePower(double set) {
        intakeMotor.setPower(set);
    }

    public void setIntokePos(double set) {
        intakeServo.setPosition(set);
    }

    public NormalizedRGBA getColor() {
        return colorSensor.getNormalizedColors();
    }

    public void teleopArmMovement(double armPower, double clawPos) {
        setinakePower(armPower);
        setIntokePos(clawPos);

    }
}

