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
import org.firstinspires.ftc.teamcode.pedroPathing.control.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.control.PIDFController;

public class Shooter {
    // declaring motor variables
    DcMotorEx flywheel;

    double maxRPM = 6000;

    Servo hoodServo1;
    Servo hoodServo2;

    double targetVelocity;

    double hoodPos = 0;
    public PIDFController shooterPID;

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

        shooterPID = new PIDFController(new PIDFCoefficients(0.35, 0.2, 0.8, 0.5)); //change this!
    }

    public void setPower (double power){
        flywheel.setPower(power);
    }
    public double getRPM (){ return flywheel.getVelocity() * 60 / flywheel.getMotorType().getTicksPerRev(); }
    public double rpmToVelocity(double RPM){
        return RPM * flywheel.getMotorType().getTicksPerRev() / 60;
    }

    public void setHood(double angle){

        if (hoodPos != angle) {
            hoodPos = Math.max(Math.min(angle, 0.5), 0);
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

    public void spinUp(double targetRPM){
        shooterPID.reset();
        shooterPID.setTargetPosition(rpmToVelocity(targetRPM));
        updateShooter();
    }

    public boolean updateShooter(double targetRPM){
        shooterPID.setTargetPosition(rpmToVelocity(targetRPM));
        shooterPID.updatePosition(flywheel.getVelocity());
        setPower(clamp(shooterPID.run(), 0, 1));

        return Math.abs(shooterPID.getError()) <= 20;
    }

    public void updateShooter(){
        shooterPID.updatePosition(getRPM());
        setPower(shooterPID.run() / maxRPM);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

}
