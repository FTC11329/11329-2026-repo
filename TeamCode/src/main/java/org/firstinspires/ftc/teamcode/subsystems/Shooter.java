package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTag;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class Shooter {
    // declaring motor variables
    DcMotorEx flywheel;

    double maxRPM = 6000;

    Servo hoodServo1;
    Servo hoodServo2;

    double hoodPos = 0;

    public Shooter(HardwareMap hardwareMap){
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setCurrentAlert(4, CurrentUnit.AMPS);

        hoodServo1 = hardwareMap.get(Servo.class, "hood1");
        hoodServo1.setDirection(Servo.Direction.FORWARD);
        hoodServo1.setPosition(0);

        hoodServo2 = hardwareMap.get(Servo.class, "hood2");
        hoodServo2.setDirection(Servo.Direction.REVERSE);
        hoodServo2.setPosition(0);
    }

    public void setPower (double power){
        flywheel.setPower(power);
    }

    public void setHood(double angle){
        if (hoodPos != angle) {
            hoodPos  = Math.max(Math.min(angle, 1), 0);
            hoodServo1.setPosition(angle);
            hoodServo2.setPosition(angle);
        }
    }

    public void setHoodDeg(double hoodDeg) {
        setHood((hoodDeg - 5) / 80);
    }

    public double getHoodPos() {
        return (hoodPos * 80) + 5;
    }

    public boolean spinUp(double targetRPM){
        setPower(targetRPM/maxRPM);
        return (flywheel.getVelocity(AngleUnit.RADIANS) > (targetRPM));
    }

//    public void rotateHood(double speed){
//        setHood(hoodPos + speed);
//    }
}
