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
    Servo intakeServo;
    RevColorSensorV3 colorSensor;

    public Intake(HardwareMap hardwaremap) {
        intakeMotor = hardwaremap.get(DcMotorEx.class, "wheel1");
        intakeServo = hardwaremap.get(Servo.class, "claw");
        colorSensor = hardwaremap.get(RevColorSensorV3.class, "spindexerColorSensor");
    }

    public void setintakePower(double set) {
        intakeMotor.setPower(set);
    }

    public void setIntakePos(double set) {
        intakeServo.setPosition(set);
    }

    public NormalizedRGBA getRGBA() {
        return colorSensor.getNormalizedColors();
    }

    public BallColor getColor() {
        return ColorFunctions.toColor(getRGBA());
    }

    public void teleopArmMovement(double armPower, double clawPos) {
        setintakePower(armPower);
        setIntakePos(clawPos);

    }
}

