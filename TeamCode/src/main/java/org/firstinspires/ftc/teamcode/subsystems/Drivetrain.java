
package org.firstinspires.ftc.teamcode.subsystems;

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
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);

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
        double speed = 0.3;
        // TURBO MODE
        if (TURBO) {
            speed = 1;
        }
        //MATH
        leftFront.setPower((forwardBackPower + strafePower + turning) * speed);
        leftBack.setPower((forwardBackPower  - strafePower + turning) * speed);
        rightFront.setPower((forwardBackPower - strafePower - turning) * speed);
        rightBack.setPower((forwardBackPower + strafePower - turning) * speed);
    }
}