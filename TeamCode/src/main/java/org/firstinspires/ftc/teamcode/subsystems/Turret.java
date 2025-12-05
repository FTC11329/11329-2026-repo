package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.ftcontrol.panels.plugins.html.primitives.P;
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

    Pose goalPose;

    public final DcMotorEx encoder;
    // Constants — CHANGE FOR YOUR ROBOT
    private static final int TICKS_PER_REV = 4096;   // or your encoder type
    private static final double GEAR_RATIO = 6.25;    // gear reduction to output

     public PIDFController turretPID;

    public Turret(HardwareMap hardwareMap, RobotSide robotSide){

        turretServo1 = hardwareMap.get(CRServo.class, "turret1");
        turretServo1.setDirection(CRServo.Direction.FORWARD);

        turretServo2 = hardwareMap.get(CRServo.class, "turret2");
        turretServo2.setDirection(CRServo.Direction.FORWARD);

        encoder = hardwareMap.get(DcMotorEx.class, "encoder");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretPID = new PIDFController(Constants.Turret.turretPID);
        turretPID.updateFeedForwardInput(Constants.Turret.rightF);
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

    public void update() {
        double curAngle = getAngle();

        if (Math.round(curAngle * 10) == Math.round(turretPID.getTargetPosition() * 10)) {
            turretPID.updateFeedForwardInput(0);
        } else if (curAngle > turretPID.getTargetPosition()) {
            turretPID.updateFeedForwardInput(Constants.Turret.leftF);
        } else {
            turretPID.updateFeedForwardInput(Constants.Turret.rightF);
        }

        turretPID.updatePosition(curAngle);  // degrees
        setPower(turretPID.run());
    }


    public void setPower(double set) {
        turretServo1.setPower(set);
        turretServo2.setPower(set);
    }

    // Converts raw encoder ticks to turret angle
    public double ticksToDegrees(int ticks) {
        double motorRevs = ticks / (double) TICKS_PER_REV;
        double turretRevs = motorRevs / GEAR_RATIO;
        return turretRevs * 360.0;
    }

    public double getAngle() {
        return ticksToDegrees(encoder.getCurrentPosition());
    }
    public double getTicks() {
        return encoder.getCurrentPosition();
    }
    public boolean closeEnoughToTarget(Pose robotPose) {
        return robotPose.distanceFrom(goalPose) * Math.sin(Math.toRadians(Math.abs(turretPID.getError()))) <= Constants.Turret.closeEnough;
    }
}
