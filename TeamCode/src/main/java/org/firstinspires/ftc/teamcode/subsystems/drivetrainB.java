
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class drivetrainB {
    // declaring variables
    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;
    DcMotorEx rightBack;

    public drivetrainB(HardwareMap hardwareMap) {
        // telling each drive motor that it is in fact, a motor
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
    }
    // sets power of each motor
    public void setPower(double set) {
        leftFront.setPower(set);
        leftBack.setPower(set);
        rightFront.setPower(set);
        rightBack.setPower(set);

    }
    /** drivetrine
     * drivetrine
     * drivrtrine
     * drivrtrine
     * drivrtrine
     * drivrtrine
     * rdvirtrine
     */
}