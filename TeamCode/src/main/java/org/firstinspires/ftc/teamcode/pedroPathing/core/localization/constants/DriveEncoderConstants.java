package org.firstinspires.ftc.teamcode.pedroPathing.core.localization.constants;

import org.firstinspires.ftc.teamcode.pedroPathing.core.localization.Encoder;

public class DriveEncoderConstants {

    public double forwardTicksToInches = 1;
    public double strafeTicksToInches = 1;
    public double turnTicksToInches = 1;

    public double robot_Width = 1;
    public double robot_Length = 1;

    public double leftFrontEncoderDirection = Encoder.REVERSE;
    public double rightFrontEncoderDirection = Encoder.FORWARD;
    public double leftRearEncoderDirection = Encoder.REVERSE;
    public double rightRearEncoderDirection = Encoder.FORWARD;

    public String leftFrontMotorName = "leftFront";
    public String leftRearMotorName = "leftRear";
    public String rightFrontMotorName = "rightFront";
    public String rightRearMotorName = "rightRear";

    public DriveEncoderConstants forwardTicksToInches(double forwardTicksToInches) {
        this.forwardTicksToInches = forwardTicksToInches;
        return this;
    }

    public DriveEncoderConstants strafeTicksToInches(double strafeTicksToInches) {
        this.strafeTicksToInches = strafeTicksToInches;
        return this;
    }

    public DriveEncoderConstants turnTicksToInches(double turnTicksToInches) {
        this.turnTicksToInches = turnTicksToInches;
        return this;
    }

    public DriveEncoderConstants robotWidth(double robot_Width) {
        this.robot_Width = robot_Width;
        return this;
    }

    public DriveEncoderConstants robotLength(double robot_Length) {
        this.robot_Length = robot_Length;
        return this;
    }

    public DriveEncoderConstants leftFrontEncoderDirection(double leftFrontEncoderDirection) {
        this.leftFrontEncoderDirection = leftFrontEncoderDirection;
        return this;
    }

    public DriveEncoderConstants rightFrontEncoderDirection(double rightFrontEncoderDirection) {
        this.rightFrontEncoderDirection = rightFrontEncoderDirection;
        return this;
    }

    public DriveEncoderConstants leftRearEncoderDirection(double leftRearEncoderDirection) {
        this.leftRearEncoderDirection = leftRearEncoderDirection;
        return this;
    }

    public DriveEncoderConstants rightRearEncoderDirection(double rightRearEncoderDirection) {
        this.rightRearEncoderDirection = rightRearEncoderDirection;
        return this;
    }

    public DriveEncoderConstants leftFrontMotorName(String leftFrontMotorName) {
        this.leftFrontMotorName = leftFrontMotorName;
        return this;
    }

    public DriveEncoderConstants leftRearMotorName(String leftRearMotorName) {
        this.leftRearMotorName = leftRearMotorName;
        return this;
    }

    public DriveEncoderConstants rightFrontMotorName(String rightFrontMotorName) {
        this.rightFrontMotorName = rightFrontMotorName;
        return this;
    }

    public DriveEncoderConstants rightRearMotorName(String rightRearMotorName) {
        this.rightRearMotorName = rightRearMotorName;
        return this;
    }

    public void defaults() {
        forwardTicksToInches = 1;
        strafeTicksToInches = 1;
        turnTicksToInches = 1;

        robot_Width = 1;
        robot_Length = 1;

        leftFrontEncoderDirection = Encoder.REVERSE;
        rightFrontEncoderDirection = Encoder.FORWARD;
        leftRearEncoderDirection = Encoder.REVERSE;
        rightRearEncoderDirection = Encoder.FORWARD;

        leftFrontMotorName = "leftFront";
        leftRearMotorName = "leftRear";
        rightFrontMotorName = "rightFront";
        rightRearMotorName = "rightRear";
    }
}