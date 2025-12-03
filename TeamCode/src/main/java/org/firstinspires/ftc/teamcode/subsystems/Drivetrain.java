
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    /** dricetraine
     * drivetrain
     * drivetrine
     * drivetrine
     * drivrtrine
     * drivrtrine
     * drivrtrine
     * drivrtrine
     * rdvirtrine
     */
    //
    public void teleopMovement(double forwardBackPower, double strafePower, double turning, boolean TURBO) {
        double speed = 0.6;
        // TURBO MODE
        if (TURBO) {
            speed = 1;
        }
        //MATH
//        leftFront.setPower((forwardBackPower + strafePower + turning) * speed);
//        leftBack.setPower((forwardBackPower  - strafePower + turning) * speed);
//        rightFront.setPower((forwardBackPower - strafePower - turning) * speed);
//        rightBack.setPower((forwardBackPower + strafePower - turning) * speed);

        setSafePower(leftFront, ((forwardBackPower + strafePower + turning) * speed));
        setSafePower(leftBack, ((forwardBackPower  - strafePower + turning) * speed));
        setSafePower(rightFront, ((forwardBackPower - strafePower - turning) * speed));
        setSafePower(rightBack, ((forwardBackPower + strafePower - turning) * speed));
    }
    // todo remove once ve have power switch v2
    void setSafePower(DcMotorEx motor, double targetPower){
        final double SLEW_RATE = 0.4;
        double currentPower = motor.getPower();
        double desiredChange = targetPower - currentPower;
        double limitedChange = Math.max(-SLEW_RATE, Math.min(desiredChange, SLEW_RATE));
        motor.setPower(currentPower += limitedChange);
    }}