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
    public int encoderOffset;

    Pose goalPose;
    double curAngle;

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
        turretPID.setTargetPosition(180 + ticksToDegrees(encoderOffset));
        if (robotSide == RobotSide.Blue) {
            goalPose = new Pose(72, 72);
        } else {
            goalPose = new Pose(72, -72);
        }
    }

    /*
    Turret is bounded from [-180, 225] where 0 degrees is facing straight forward
    Turret is initialized at 180
    Turret angle runs counter clockwise ()
    Encoder ticks increase as angle decreases (can't flip this or the intake will reverse)
    negative power runs counter clockwise, positive runs clockwise
     */

    public void setTargetDeg(double deg) {

        deg = closestTargetAngle(deg);
        while (deg > 225) {
            deg -= 360;
        }
        while (deg < -180) {
            deg += 360;
        }

        turretPID.setTargetPosition(deg + Constants.Turret.turretOffset);
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
        update(angVel, 0);
    }

    public void update(double angVel, double angAccel) {
        curAngle = getAngle();
        double targetAngle = turretPID.getTargetPosition();

        double error = getAngularError(curAngle, targetAngle);

        if (Math.abs(error) < 0.23) {
            turretPID.updateFeedForwardInput(0);
        } else if (curAngle > turretPID.getTargetPosition()) {
            turretPID.updateFeedForwardInput(Constants.Turret.CCW_F);
        } else {
            turretPID.updateFeedForwardInput(Constants.Turret.CW_F);
        }

        turretPID.updatePosition(curAngle);  // degrees
        double velocityFF = angVel * Constants.Turret.kV;
        double accelerationFF = angAccel * Constants.Turret.kA;
        setPower(turretPID.run() - velocityFF - accelerationFF);
    }

    public double getAngularError(double currentAngle, double targetAngle) {
        return targetAngle - currentAngle;
    }


    public void setPower(double set) {
        turretServo1.setPower(- set);
        turretServo2.setPower(- set);
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
        // this gives the ticks in robot relative numbers so it is easier to use
        return 12800 - (encoder.getCurrentPosition() + encoderOffset);
    }
    public boolean closeEnoughToTarget(Pose robotPose) {
        return robotPose.distanceFrom(goalPose) * Math.sin(Math.toRadians(Math.abs(turretPID.getError()/4))) <= Constants.Turret.closeEnough;
    }

    public void stop() {
        turretServo1.setPower(0);
        turretServo2.setPower(0);
    }
}
