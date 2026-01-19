package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    double hoodPos = 0;
    public int encoderOffset;


    Pose goalPose;

    public final DcMotorEx encoder;
    private static final int TICKS_PER_REV = 4096;
    private static final double GEAR_RATIO = 6.25;

     public PIDFController turretPID;

    public Turret(HardwareMap hardwareMap, int startTurretTicks, RobotSide robotSide){

        turretServo1 = hardwareMap.get(CRServo.class, "turret1");
        turretServo1.setDirection(CRServo.Direction.FORWARD);

        turretServo2 = hardwareMap.get(CRServo.class, "turret2");
        turretServo2.setDirection(CRServo.Direction.FORWARD);

        encoder = hardwareMap.get(DcMotorEx.class, "transfer"); //name = "encoder" for absolute, currently broken in electrical
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        encoderOffset = startTurretTicks;

        turretPID = new PIDFController(Constants.Turret.turretPID);
        turretPID.updateFeedForwardInput(Constants.Turret.CW_F);
        if (robotSide == RobotSide.Blue) {
            goalPose = new Pose(72, 72);
        } else {
            goalPose = new Pose(72, -72);
        }
    }

    public void setTargetDeg(double deg) {
        double robotDeg = 180 - deg;
        while (robotDeg > 360) {
            robotDeg -= 360;
        }
        while (robotDeg < 0) {
            robotDeg += 360;
        }

        turretPID.setTargetPosition(robotDeg + Constants.Turret.turretOffset);
    }

    public void update(double angVel, double angAccel) {
        double curAngle = getAngle();

        if (Math.abs(curAngle - turretPID.getTargetPosition()) < 0.23) {
            turretPID.updateFeedForwardInput(0);
        } else if (curAngle > turretPID.getTargetPosition()) {
            turretPID.updateFeedForwardInput(Constants.Turret.CCW_F);
        } else {
            turretPID.updateFeedForwardInput(Constants.Turret.CW_F);
        }

        turretPID.updatePosition(curAngle);  // degrees
        double velocityFF = angVel * Constants.Turret.kV;
        double accelerationFF = angAccel * Constants.Turret.kA;
//        if (turretPID.getError() > 80) {
//            setPower(Math.signum(turretPID.getError()));
//        } else {
        setPower(turretPID.run() + velocityFF + accelerationFF);
//        }
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
