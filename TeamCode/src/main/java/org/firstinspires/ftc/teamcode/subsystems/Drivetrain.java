
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
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void setLeftBackPower(double power) {
        leftBack.setPower(power);
    }

    public void setLeftFrontPower(double power) {
        leftFront.setPower(power);
    }

    public void setRightBackPower(double power) {
        rightBack.setPower(power);
    }

    public void setRightFrontPower(double power) {
        rightFront.setPower(power);
    }

    // sets power of each motor
    public void setPower(double set) {
        leftFront.setPower(set);
        leftBack.setPower(set);
        rightFront.setPower(set);
        rightBack.setPower(set);
    }

    public void teleopMovement(double forwardBackPower, double strafePower, double turning, boolean TURBO) {
        double speed = Constants.Drivetrain.notTurboPower;
        // TURBO MODE
        if (TURBO) {
            speed = Constants.Drivetrain.turboPower;
        }

        double leftFrontPower  = (forwardBackPower + strafePower + turning);
        double leftBackPower   = (forwardBackPower - strafePower + turning);
        double rightFrontPower = (forwardBackPower - strafePower - turning);
        double rightBackPower  = (forwardBackPower + strafePower - turning);

        double maxPower = Math.max(1,
                Math.max(
                    Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower)),
                    Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower))
                )
        );

        leftFrontPower /= maxPower;
        leftBackPower /= maxPower;
        rightFrontPower /= maxPower;
        rightBackPower /= maxPower;

        //MATH
        leftFront.setPower(leftFrontPower * speed);
        leftBack.setPower(leftBackPower * speed);
        rightFront.setPower(rightFrontPower * speed);
        rightBack.setPower(rightBackPower * speed);
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