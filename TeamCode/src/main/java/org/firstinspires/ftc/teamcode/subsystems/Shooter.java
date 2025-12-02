package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.control.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.control.PIDFController;

public class Shooter {
    // declaring motor variables
    public DcMotorEx flywheel;

    double maxRPM = 6000;

    Servo hoodServo1;
    Servo hoodServo2;

    double targetVelocity;


    double hoodPos = 0;
    public PIDFController shooterPID;
    
    boolean shooterSpin;

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

        shooterPID = new PIDFController(new PIDFCoefficients(
                Constants.Shooter.P,
                Constants.Shooter.I,
                Constants.Shooter.D,
                Constants.Shooter.F));
        shooterPID.reset();
    }

    public void setPower(double power){
        flywheel.setPower(power);
    }
    public double getRPM(){
        return flywheel.getVelocity() * 60 / Constants.Shooter.ticksPerRevolution;
    }
    public double rpmToVelocity(double RPM){
        return RPM * Constants.Shooter.ticksPerRevolution / 60;
    }
    public double getVelocity(){
        return flywheel.getVelocity();
    }

    // Set hood from 0-1
    public void setHood(double set){
        if (hoodPos != set) {
            hoodPos = Math.max(Math.min(set, 0.5), 0);
            hoodServo1.setPosition(set);
            hoodServo2.setPosition(set);
        }
    }
    
    public void setHoodDeg(double hoodDeg) {
        setHood((hoodDeg - 5) / 80);
    }

    // get the degrees of the hood
    public double getHoodPosDeg() {
        return (hoodPos * 80) + 5;
    }

    public void resetShooter(){
        shooterPID.reset();
    }

    public boolean closeEnoughToTarget() {
        return Math.abs(shooterPID.getError()) <= rpmToVelocity(Constants.Shooter.closeEnoughRPM);
    }

    // targetRPM is in ticks/sec
    public void setTargetRPM(double targetRPM) {
        double targetVel = rpmToVelocity(targetRPM);
        shooterPID.setTargetPosition(targetVel);
        shooterSpin = true;
    }

    public void stopShooter(){
        shooterSpin = false;
        setPower(0);
    }

    public double getShooterPower() {
        return shooterPID.run();
    }

    public void setPID() {
        shooterPID.setCoefficients(new PIDFCoefficients(
                Constants.Shooter.P,
                Constants.Shooter.I,
                Constants.Shooter.D,
                Constants.Shooter.F));
    }

    public PIDFCoefficients getPID() {
        return shooterPID.getCoefficients();
    }

    public void update() {
        if (shooterSpin){
            shooterPID.updatePosition(flywheel.getVelocity());  // ticks/sec
            setPower(shooterPID.run());
        }
    }

}
