package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.control.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.control.PIDFController;

public class Shooter {
    // declaring motor variables
    public DcMotorEx flywheel;

    double maxRPM = 6000;

    Servo hoodServo1;
    Servo hoodServo2;

    double targetVelocity;

    int TICKS_PER_REVOLUTION = 28;

    double hoodPos = 0;
    public PIDFController shooterPID;

    public Shooter(HardwareMap hardwareMap){
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setCurrentAlert(4, CurrentUnit.AMPS);

        hoodServo1 = hardwareMap.get(Servo.class, "hood1");
        hoodServo1.setDirection(Servo.Direction.FORWARD);
        hoodServo1.setPosition(0);

        hoodServo2 = hardwareMap.get(Servo.class, "hood2");
        hoodServo2.setDirection(Servo.Direction.REVERSE);
        hoodServo2.setPosition(0);

        shooterPID = new PIDFController(new PIDFCoefficients(0.0006, 0.0002, 0.00005, 0.1)); // todo change this!
        shooterPID.reset();
    }

    public void setPower(double power){
        flywheel.setPower(power);
    }
    public double getRPM(){
        return flywheel.getVelocity() * 60 / TICKS_PER_REVOLUTION;
    }
    public double rpmToVelocity(double RPM){
        return RPM * TICKS_PER_REVOLUTION / 60;
    }
    public double getVelocity(){
        return flywheel.getVelocity();
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

    public void resetShooter(){
        shooterPID.reset();
    }

    public boolean closeEnough() {
        return Math.abs(shooterPID.getError()) <= rpmToVelocity(60);
    }

    // targetRPM is in ticks/sec
    public void setTargetRPM(double targetRPM) {
        double targetVel = rpmToVelocity(targetRPM);
        shooterPID.reset();
        shooterPID.setTargetPosition(targetVel);
    }

    public void update(int set) {
        shooterPID.updatePosition(flywheel.getVelocity());  // ticks/sec
        setPower(shooterPID.run());
    }

}
