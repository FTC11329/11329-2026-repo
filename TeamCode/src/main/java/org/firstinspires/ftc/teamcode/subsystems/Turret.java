package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.control.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.control.PIDFController;

public class Turret {
    // declaring motor variables
    CRServo turretServo1;
    CRServo turretServo2;

    double hoodPos = 0;

    public final DcMotorEx encoder;
    // Constants — CHANGE FOR YOUR ROBOT
    private static final int TICKS_PER_REV = 4096;   // or your encoder type
    private static final double GEAR_RATIO = 6.25;    // gear reduction to output

     public PIDFController turretPID;

    public Turret(HardwareMap hardwareMap){

        turretServo1 = hardwareMap.get(CRServo.class, "turret1");
        turretServo1.setDirection(CRServo.Direction.FORWARD);

        turretServo2 = hardwareMap.get(CRServo.class, "turret2");
        turretServo2.setDirection(CRServo.Direction.FORWARD);

        encoder = hardwareMap.get(DcMotorEx.class, "encoder");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretPID = new PIDFController(new PIDFCoefficients(0.01, 0.0001, 0.0002, 0.0));
    }

    public void setPower(double set) {
        turretServo1.setPower(set);
        turretServo2.setPower(set);
    }

    // Converts raw encoder ticks to turret angle
    private double ticksToDegrees(int ticks) {
        double motorRevs = ticks / (double) TICKS_PER_REV;
        double turretRevs = motorRevs / GEAR_RATIO;
        return turretRevs * 360.0;
    }


    // Converts degrees to power
    private double degreesToPower(double deg) {
        double turretRevs = deg / 360.0;
        return turretRevs * GEAR_RATIO;
    }

    // Sets the target turret angle
    public void resetTurret() {
        turretPID.reset();
    }

    public void updateTurret(double degrees) {

        turretPID.setTargetPosition(degrees);
        // read angle
        double currentDeg = ticksToDegrees(encoder.getCurrentPosition());

        // get PIDF output
        turretPID.updatePosition(currentDeg);
        double pidOut = turretPID.run();

        pidOut = clamp(pidOut, -1.0, 1.0);

        setPower(pidOut);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    public double getAngle() {
        return ticksToDegrees(encoder.getCurrentPosition());
    }

    public double getTarget() {
        return turretPID.getTargetPosition();
    }
}
