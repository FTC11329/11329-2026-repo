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
    public DcMotorEx flywheel;
    boolean usePID = false;
    Servo hoodServo1;
    Servo hoodServo2;
    double hoodPos = 0;
    public MovingPIDFController shooterPID;
    boolean shooterSpin;
    boolean isGettingUpToSpeed = true;
    boolean onceShot = false;
    double previousError;
    double derivative;
    double lastPower;



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

        shooterPID = new MovingPIDFController(Constants.Shooter.shooterVelocityPID, Constants.Shooter.kF);
        shooterPID.updateFeedForwardInput(1);
    }

    public void setPower(double power){
        lastPower = power;
        flywheel.setPower(power);
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
        usePID = false;
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
            hoodPos = Math.max(Math.min(set, (Constants.Shooter.maxHoodAngle - 5) / 80), Constants.Shooter.minHoodAngle / 80);
            hoodServo1.setPosition(hoodPos);
            hoodServo2.setPosition(hoodPos);
        }
    }
    
    public void setHoodDeg(double hoodDeg) {
        setHood((hoodDeg - 5) / 80);
    }
    public void setHoodRad(double hoodRad) {
        setHoodDeg(Math.toDegrees(hoodRad));
    }

    // get the degrees of the hood
    public double getHoodPosDeg() {
        return (hoodPos * 80) + 5;
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
        shooterSpin = true;
    }
    // this is for when you want to continuously change the RPM of the flywheel
    public void adjustTargetRPM(double targetRPM) {
        usePID = true;
        shooterPID.moveTargetPosition(targetRPM);
        shooterSpin = true;
    }
    public double getTargetRpm() {
        return shooterPID.getTargetPosition();
    }

    public double velocityToRPM_Regression(double exitVelocity) {
        double a = 0.0013149578634563265;
        double b = 5.559109887421296;
        double c = 741.0912371298367;
        return a * exitVelocity * exitVelocity + b * exitVelocity + c;
    }

    public double velocityToRPM(double exitVelocity) {
        // exitVelocity in in/s
        double wheelDiameter = 2.0;

        double surfaceVelocity = PI * wheelDiameter;

        double rotationsPerSec = exitVelocity / surfaceVelocity;

        return rotationsPerSec * 60 + 140;
    }

    public double getBallVelocity(){
        double rps = 2 * PI * getRPM() / 60.0;
        return rps * (Constants.ShooterParamaters.MotorToWheel+Constants.ShooterParamaters.R_WHEEL_M) / 2.0;
    }

    public void resetController() {
        shooterPID.resetController();
    }

    public void stop() {
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setPower(0);
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
        if (shooterSpin && usePID) {
            shooterPID.updatePosition(getRPM());
            if (shooterPID.getTargetPosition() > 10) {
                if (isGettingUpToSpeed) {
                    if (shooterPID.getError() < Constants.Shooter.closeEnoughRPM) {
                        isGettingUpToSpeed = false;
                    }
                }
                setPower(shooterPID.run());
            } else {
                setPower(0);
            }
        } else if (shooterSpin) {
            if (!isGettingUpToSpeed) {
                isGettingUpToSpeed = true;
            }
            setPower(0.5);
        }
    }
}
