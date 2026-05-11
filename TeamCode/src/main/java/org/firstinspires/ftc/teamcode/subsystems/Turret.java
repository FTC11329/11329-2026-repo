package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.control.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.control.PIDFController;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.RobotSide;

public class Turret {
    // declaring motor variables
    CRServo turretServo1;
    CRServo turretServo2;
    public int encoderOffset = 12830;

    Pose goalPose;

    public final DcMotorEx encoder;
    private static final int TICKS_PER_REV = 4096;
    private static final double GEAR_RATIO = 6.25;

    public PIDFController turretPID;
    public PIDFController secondaryTurretPIDF;

    public Turret(HardwareMap hardwareMap, int startTurretTicks, RobotSide robotSide){

        turretServo1 = hardwareMap.get(CRServo.class, "turret1");
        turretServo1.setDirection(CRServo.Direction.FORWARD);

        turretServo2 = hardwareMap.get(CRServo.class, "turret2");
        turretServo2.setDirection(CRServo.Direction.FORWARD);

        encoder = hardwareMap.get(DcMotorEx.class, "transfer");
        reZero();

        encoderOffset += startTurretTicks;

        turretPID = new PIDFController(Constants.Turret.turretPID);
        turretPID.updateFeedForwardInput(Constants.Turret.CW_F);
        turretPID.setTargetPosition(ticksToDegrees(encoderOffset));
        if (robotSide == RobotSide.Blue) {
            goalPose = new Pose(72, 72);
        } else {
            goalPose = new Pose(72, -72);
        }
    }

    public void reZero() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTargetDeg(double deg) {
        double robotDeg = 180 - deg;
        robotDeg = closestTargetAngle(robotDeg);
        while (robotDeg > 360) {
            robotDeg -= 360;
        }
        while (robotDeg < -45) {
            robotDeg += 360;
        }

        turretPID.setTargetPosition(robotDeg + Constants.Turret.turretOffset);
    }
    private double closestTargetAngle(double targetDeg) {
        double currentDeg = curAngle;
        // normalize target into [0,360)
        targetDeg %= 360.0;
        if (targetDeg < 0) targetDeg += 360.0;

        // find shortest delta in [-180,180)
        double delta = targetDeg - (currentDeg % 360.0);
        delta = (delta + 540.0) % 360.0 - 180.0;

        // shift target so it lives near the current angle
        return currentDeg + delta;
    }
    public void setTargetRad(double rad) {
        setTargetDeg(Math.toDegrees(rad));
    }
    public void update(double angVel) {
        update(angVel, 0, 1);
    }
    public void update(double angVel, double angAccel) {
        update(angVel, angAccel, 1);
    }

    double curAngle = 0;
    double previousAngle = 0;
    double previousTime = System.nanoTime() * 1e-9;
    double curTime = System.nanoTime() * 1e-9;
    public void update(double angVel, double angAccel, double voltageCompensation) {
        update(angVel,angAccel,voltageCompensation, true);
    }
    public void update(double angVel, double angAccel, double voltageCompensation, boolean usePid) {
        previousAngle = curAngle;
        previousTime = curTime;
        curTime = System.nanoTime() * 1e-9;
        curAngle = getAngle();

        if (Math.abs(curAngle - turretPID.getTargetPosition()) < 0.75) {
            turretPID.updateFeedForwardInput(0);
        } else if (curAngle > turretPID.getTargetPosition()) {
            if (curAngle < 90) {
                turretPID.updateFeedForwardInput(Constants.Turret.CableCW_F);
            } else {
                turretPID.updateFeedForwardInput(Constants.Turret.CCW_F);
            }
        } else {
            turretPID.updateFeedForwardInput(Constants.Turret.CW_F);
        }

        turretPID.updatePosition(curAngle);  // degrees
        if (usePid) {
            turretPID.setCoefficients(Constants.Turret.turretPID);
            double kV = Constants.Turret.kV;
            if (curAngle > 245) {
                if (angVel > 0) {
                    kV += Constants.Turret.Cable_kV;
                }
                if (turretPID.getError() < 0) {
                    double kP = Constants.Turret.P + Constants.Turret.Cable_kP;
                    turretPID.setP(kP);
                }
            }
            double velocityFF = angVel * kV;
            double accelerationFF = angAccel * Constants.Turret.kA;
            setPower((( turretPID.run() + velocityFF * voltageCompensation) + accelerationFF));
        } else {
            setPower(0);
        }
    }

    // deg / sec
    public double getVelocity() {
        return (curAngle - previousAngle) / (curTime - previousTime);
    }

    public void setPower(double set) {
        turretServo1.setPower(set);
        turretServo2.setPower(set);
    }

    // Converts raw encoder ticks to turret angle
    public double ticksToDegrees(double ticks) {
        double motorRevs = ticks / (double) TICKS_PER_REV;
        double turretRevs = motorRevs / GEAR_RATIO;
        return turretRevs * 360.0;
    }

    // Degrees
    public double getAngle() {
        return ticksToDegrees(getTicks());
    }
    public int getTicks() {
        return encoder.getCurrentPosition() + encoderOffset;
    }
    public boolean closeEnoughToTarget(Pose robotPose) {
        return robotPose.distanceFrom(goalPose) * Math.sin(Math.toRadians(Math.abs(turretPID.getError()/4))) <= Constants.Turret.closeEnough;
}

    public void stop() {
        turretServo1.setPower(0);
        turretServo2.setPower(0);
    }
}