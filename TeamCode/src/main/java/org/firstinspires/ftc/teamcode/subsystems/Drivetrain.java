
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.math.Vector;
import org.firstinspires.ftc.teamcode.util.RobotSide;

public class Drivetrain {
    // declaring variables
    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;
    DcMotorEx rightBack;

    public Drivetrain(HardwareMap hardwareMap) {
        // telling each drive motor that it is in fact, a motor
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        // setting the motor direction to go correctly
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    // sets power of each motor
    public void setPower(double set) {
        leftFront.setPower(set);
        leftBack.setPower(set);
        rightFront.setPower(set);
        rightBack.setPower(set);

    }

    public void teleopMovement(double forwardBackPower, double strafePower, double turning, boolean TURBO) {
        double speed = 0.6;
        // TURBO MODE
        if (TURBO) {
            speed = 1;
        }
        //MATH
        leftFront.setPower((forwardBackPower + strafePower + turning) * speed);
        leftBack.setPower((forwardBackPower - strafePower + turning) * speed);
        rightFront.setPower((forwardBackPower - strafePower - turning) * speed);
        rightBack.setPower((forwardBackPower + strafePower - turning) * speed);
    }

    double aTowardMax = 15.0;     // max accel toward goal
    double aAwayMax   = 10.0;     // max accel away (braking harder)
    double lastTime;
    double velocityToGoal;
    double vRadialCmd;
    public void profiledMovement(double forwardBackPower, double strafePower, double turning, boolean TURBO, Pose curPose, RobotSide robotSide, Vector velocity) {
        double speed = 0.6;
        // TURBO MODE
        if (TURBO) {
            speed = 1;
        }
        long now = System.currentTimeMillis();
        if (lastTime == 0) {lastTime = now; return;}
        double dt = (now - lastTime) * 1e-3;
        lastTime = now;
        Pose goal;
        if (robotSide == RobotSide.Blue)  {
            goal = Constants.Vision.blueGoal;
        } else {
            goal = Constants.Vision.redGoal;
        }

        double deltaX = goal.getX() - curPose.getX();
        double deltaY = goal.getY() - curPose.getY();

        double distanceToGoal = Math.hypot(deltaX, deltaY);


        double ux = deltaX / distanceToGoal;
        double uy = deltaY / distanceToGoal;


        double vX = velocity.getXComponent();
        double vY = velocity.getYComponent();

        velocityToGoal = vX * ux + vY * uy;

        double cos = Math.cos(curPose.getHeading());
        double sin = Math.sin(curPose.getHeading());

        double vFieldX = forwardBackPower * cos - strafePower * sin;
        double vFieldY = forwardBackPower * sin + strafePower * cos;

        vRadialCmd = (vFieldX * ux + vFieldY * uy) * 1e-2;

        double dv = vRadialCmd - velocityToGoal;

        double accelLimit = (dv > 0) ? aTowardMax : aAwayMax;
        double dvMax = accelLimit * dt;

        dv = clamp(dv, -dvMax, dvMax);

        double vRadialProfiled = velocityToGoal + dv;

        double vTanX = vFieldX - vRadialCmd * ux;
        double vTanY = vFieldY - vRadialCmd * uy;

        double vFieldXProfiled = vRadialProfiled * ux + vTanX;
        double vFieldYProfiled = vRadialProfiled * uy + vTanY;

        double cosH = Math.cos(-curPose.getHeading());
        double sinH = Math.sin(-curPose.getHeading());

        double forwardProfiled = vFieldXProfiled * cosH - vFieldYProfiled * sinH;
        double strafeProfiled  = vFieldXProfiled * sinH + vFieldYProfiled * cosH;

        leftFront.setPower((forwardProfiled + strafeProfiled + turning) * speed);
        leftBack.setPower((forwardProfiled - strafeProfiled + turning) * speed);
        rightFront.setPower((forwardProfiled - strafeProfiled - turning) * speed);
        rightBack.setPower((forwardProfiled + strafeProfiled - turning) * speed);

    }
    public void stop() {
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}