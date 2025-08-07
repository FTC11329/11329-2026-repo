
package org.firstinspires.ftc.teamcode.subsystems;



import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utility.DriveSpeedEnum;
import org.firstinspires.ftc.teamcode.utility.SimplePIDControl;

import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class drivetrain_b {
    public final DcMotorEx leftFront;
    public final DcMotorEx leftBack;
    public final DcMotorEx rightBack;
    public final DcMotorEx rightFront;
    private final List<DcMotorEx> motors;

    public SimplePIDControl pidControl;

    public boolean isAtPTOPosition = false;
    public double lastLeftFrontPower = 0;
    public double lastLeftBackPower = 0;
    public double lastRightBackPower = 0;
    public double lastRightFrontPower = 0;


    public Drivetrain(HardwareMap hardwareMap) {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
//        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
//        leftBack = hardwareMap.get(DcMotorEx.class, "backLeft");
//        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
//        rightBack = hardwareMap.get(DcMotorEx.class, "backRight");

        motors = Arrays.asList(leftFront, leftBack, rightBack, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        pidControl = new SimplePIDControl(Constants.PTO.p, Constants.PTO.i, Constants.PTO.d);
    }

    public void drive(double forward, double strafe, double turn, DriveSpeedEnum driveSpeed) {
        double speed = 0;
        if (driveSpeed == DriveSpeedEnum.Fast) {
            speed = Constants.Drivetrain.fastSpeed;
        } else if (driveSpeed == DriveSpeedEnum.Slow) {
            speed = Constants.Drivetrain.slowSpeed;
        } else if (driveSpeed == DriveSpeedEnum.Auto) {
            speed = 1;
        } else if (driveSpeed == DriveSpeedEnum.PTOSpeed) {
            speed = Constants.PTO.speed;
        }

        setWeightedDrivePower(new Pose2d(forward * speed, strafe * speed, turn * speed));
    }

    public void PTOLoop(double feedForward) {
        rightFront.setPower(pidControl.update(rightFront.getCurrentPosition(), feedForward));
        rightBack.setPower(pidControl.update(rightFront.getCurrentPosition(), feedForward));
        leftFront.setPower(pidControl.update(leftFront.getCurrentPosition(), feedForward));
        leftBack.setPower(pidControl.update(leftFront.getCurrentPosition(), feedForward));
    }

    public void moveBackWheels() {
        rightBack.setPower(0.7);
        leftBack.setPower(0.7);
    }

    public void setRunToPos() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void setPTOPos(int ptoPos) {
        pidControl.setTargetValue(ptoPos);
        pidControl.setTargetValue(ptoPos);
    }
    public void setPTOPower(double power) {
        rightFront.setPower(power);
        rightBack.setPower(power);
        leftFront.setPower(power);
        leftBack.setPower(power);
    }
    public int getPTOTPos() {
        return (int) pidControl.getTargetValue();
    }
    public int getPTOPos() {
        return (leftFront.getCurrentPosition() + rightFront.getCurrentPosition() / 2);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.position.x) + Math.abs(drivePower.position.y)
                + Math.abs(drivePower.heading.toDouble()) > 1) {
            // re-normalize the powers according to the weights
            double denom = Math.abs(drivePower.position.x)
                    + Math.abs(drivePower.position.y)
                    + Math.abs(drivePower.heading.toDouble());

            vel = new Pose2d(
                    drivePower.position.x / denom,
                    drivePower.position.y / denom,
                    drivePower.heading.toDouble() / denom
            );
        }

        setDrivePowers(new PoseVelocity2d(vel.position, vel.heading.toDouble()));
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));
        double rightBackScalar = 1;
        double leftBackScalar = 1;
        double rightFrontScalar = 1;
        double leftFrontScalar = 1;
        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        //Optimizing loop times
        double leftFrontPower = (wheelVels.leftFront.get(0) / maxPowerMag) * leftFrontScalar;
        double leftBackPower = (wheelVels.leftBack.get(0) / maxPowerMag) * leftBackScalar;
        double rightBackPower = (wheelVels.rightBack.get(0) / maxPowerMag) * rightBackScalar;
        double rightFrontPower = (wheelVels.rightFront.get(0) / maxPowerMag) * rightFrontScalar;

        if (lastLeftFrontPower != leftFrontPower) {
            lastLeftFrontPower = leftFrontPower;
            leftFront.setPower(leftFrontPower);
        }
        if (lastLeftBackPower != leftBackPower) {
            lastLeftBackPower = leftBackPower;
            leftBack.setPower(leftBackPower);
        }
        if (lastRightBackPower != rightBackPower) {
            lastRightBackPower = rightBackPower;
            rightBack.setPower(rightBackPower);
        }
        if (lastRightFrontPower != rightFrontPower) {
            lastRightFrontPower = rightFrontPower;
            rightFront.setPower(rightFrontPower);
        }
    }

    public double[] getDrivePowers() {
        return new double[]{
                leftFront.getPower(),
                leftBack.getPower(),
                rightFront.getPower(),
                rightBack.getPower()
        };
    }

    public double[] getDriveCurrent() {
        return new double[]{
                leftFront.getCurrent(CurrentUnit.AMPS),
                leftBack.getCurrent(CurrentUnit.AMPS),
                rightFront.getCurrent(CurrentUnit.AMPS),
                rightBack.getCurrent(CurrentUnit.AMPS)
        };
    }

    public boolean isStalled(double tripCurrent) {
        double[] currentList = getDriveCurrent();
        double current = Math.max(Math.max(currentList[0], currentList[1]), Math.max(currentList[2], currentList[3]));
        return current > tripCurrent;
    }

    public void stopDrive() {
        drive(0, 0, 0, DriveSpeedEnum.Slow);
    }
}