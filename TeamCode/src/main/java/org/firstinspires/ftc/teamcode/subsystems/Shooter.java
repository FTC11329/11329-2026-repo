package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.PI;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.control.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.control.MovingPIDFController;

public class Shooter {
    // declaring motor variables
    public DcMotorEx flywheel1;
    public DcMotorEx flywheel2;
    boolean usePID = false;
    Servo hoodServo1;
//    Servo hoodServo2; todo un comment if new shooter bad, else delete
    double hoodPos = 0.6767676767676767676767676767676767;
    public MovingPIDFController shooterPID;
    boolean isGettingUpToSpeed = true;
    boolean onceShot = false;
    double previousError;
    double derivative;
    double lastPower;
    double flywheelVelocity = 0;

    public Shooter(HardwareMap hardwareMap){
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");

        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel1.setCurrentAlert(4, CurrentUnit.AMPS);

        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel2.setCurrentAlert(4, CurrentUnit.AMPS);

        hoodServo1 = hardwareMap.get(Servo.class, "hood1");
        hoodServo1.setDirection(Servo.Direction.REVERSE);

//        hoodServo2 = hardwareMap.get(Servo.class, "hood2");
//        hoodServo2.setDirection(Servo.Direction.REVERSE);

        setHoodDeg(Constants.Shooter.minHoodAngle);

        shooterPID = new MovingPIDFController(Constants.Shooter.shooterVelocityPID, Constants.Shooter.kV);
        shooterPID.updateFeedForwardInput(1);
    }

    public void setPower(double power){
        lastPower = power;
        flywheel1.setPower(power);
        flywheel2.setPower(power);
    }
    public boolean hasShot() {
        if (closeEnoughToTarget() || isGettingUpToSpeed){
            onceShot = false;
            return false;
        }
        derivative = shooterPID.getErrorDerivative();
        previousError = shooterPID.getLastsError();

        if (derivative <= 0 && previousError > 160 && !onceShot) {
            onceShot = true;
            return true;
        }
        return false;
    }
    public void casualModeOn(){
        setHoodDeg(Constants.Shooter.minHoodAngle);
        usePID = false;
    }
    public double getRPM(){
        return (getVelocity()) * 60 / Constants.Shooter.ticksPerRevolution;
    }
    public double rpmToVelocity(double RPM){
        return RPM * Constants.Shooter.ticksPerRevolution / 60;
    }
    public double getVelocity(){
        return flywheelVelocity;
    }

    // Set hood from 0-1
    public double lastSet;
    public void setHood(double set){
        if (Math.abs(hoodPos - set) >= .004) {
            hoodPos = Math.max(Math.min(set,
                    (Constants.Shooter.maxHoodAngle - 13.92) / 44.83),
                    (Constants.Shooter.minHoodAngle - 13.92) / 44.83);
            hoodServo1.setPosition(hoodPos);
        }
    }
    
    public void setHoodDeg(double hoodDeg) {
        setHood((hoodDeg - 13.92) / 44.83);
    }
    public void setHoodRad(double hoodRad) {
        setHoodDeg(Math.toDegrees(hoodRad));
    }

    // get the degrees of the hood
    public double getHoodPosDeg() {
        return hoodPos;
    }

    public void resetShooter(){
        shooterPID.reset();
    }

    public boolean closeEnoughToTarget() {
        return Math.abs(shooterPID.getError()) - Constants.Shooter.closeEnoughRPM < 0;
    }

    public boolean getUsePID() {return usePID;}
    // targetRPM is in ticks/sec
    public void setTargetRPM(double targetRPM) {
        usePID = true;
        shooterPID.setTargetPosition(targetRPM);
    }
    // this is for when you want to continuously change the RPM of the flywheel1
    public void adjustTargetRPM(double targetRPM) {
        usePID = true;
        shooterPID.moveTargetPosition(targetRPM);
    }
    public double getTargetRpm() {
        return shooterPID.getTargetPosition();
    }

    public double velocityToRPM_Regression(double exitVelocity) {
        double a = 0;
        double b = 14.3659377869;
        double c = 400;
        return a * exitVelocity * exitVelocity + b * exitVelocity + c;
    }

    public double velocityToRPM(double exitVelocity) {
        // exitVelocity in in/s
        double wheelDiameter = 1.75;

        double surfaceVelocity = PI * wheelDiameter;

        double rotationsPerSec = exitVelocity / surfaceVelocity;

        return rotationsPerSec * 60 + 140;
    }

    public void resetController() {
        shooterPID.resetController();
    }

    public void stop() {
        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel1.setPower(0);
        setHood(0);
    }

    public double getShooterPower() {
        return shooterPID.run();
    }

    public void setPID() {
        shooterPID.setCoefficients(Constants.Shooter.shooterVelocityPID);
    }

    public PIDFCoefficients getPID() {
        return shooterPID.getCoefficients();
    }

    public void update() {
        update(1, false, false);
    }
    public void update(double voltageCompensation, boolean panicShoot, boolean panicShootButton) {
        flywheelVelocity = flywheel1.getVelocity();
        if (!panicShoot && usePID) {
            shooterPID.updatePosition(getRPM());
            if (shooterPID.getTargetPosition() > 10) {
                if (isGettingUpToSpeed) {
                    if (shooterPID.getError() < Constants.Shooter.closeEnoughRPM) {
                        isGettingUpToSpeed = false;
                    }
                }
                setPower(shooterPID.run() * voltageCompensation);
            } else {
                setPower(0);
            }
        } else if (!panicShoot) {
            if (!isGettingUpToSpeed) {
                isGettingUpToSpeed = true;
            }
//            setPower(0.5);
        } else/* if (panicShoot)*/ {
            if (panicShootButton) {
                setPower(1);
            } else {
                setPower(shooterPID.customFeedForwardOutput());
            }
        }
    }

}
