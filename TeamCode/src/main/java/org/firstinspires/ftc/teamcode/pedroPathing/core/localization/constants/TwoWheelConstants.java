package org.firstinspires.ftc.teamcode.pedroPathing.core.localization.constants;


import org.firstinspires.ftc.teamcode.pedroPathing.core.localization.CustomIMU;
import org.firstinspires.ftc.teamcode.pedroPathing.core.localization.Encoder;
import org.firstinspires.ftc.teamcode.pedroPathing.core.localization.RevHubIMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

/**
 * This is the TwoWheelConstants class. It holds many constants and parameters for the Two Wheel Localizer.
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 12/24/2024
 */


public class TwoWheelConstants {

    /** The number of inches per tick of the encoder for forward movement
     * Default Value: .001989436789 */
    public double forwardTicksToInches = .001989436789;

    /** The number of inches per tick of the encoder for lateral movement (strafing)
     * Default Value: .001989436789 */
    public double strafeTicksToInches = .001989436789;

    /** The y offset of the forward encoder (Deadwheel) from the center of the robot
     * Default Value: 1 */
    public double forwardPodY = 1;

    /** The x offset of the strafe encoder (Deadwheel) from the center of the robot
     * Default Value: -2.5 */
    public double strafePodX = -2.5;

    /** The Hardware Map Name of the IMU (built-in IMU will be Port 0, "imu")
     * Default Value: "imu" */
    public String IMU_HardwareMapName = "imu";

    /** The Hardware Map Name of the Forward Encoder (name of the motor port it is plugged into)
     * Default Value: "leftFront" */
    public String forwardEncoder_HardwareMapName = "leftFront";

    /** The Hardware Map Name of the Strafe Encoder (name of the motor port it is plugged into)
     * Default Value: "rightRear" */
    public String strafeEncoder_HardwareMapName = "rightRear";

    /** The Orientation of the IMU on the robot
     * Default Value: new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT) */
    public RevHubOrientationOnRobot IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

    /** The direction of the forward encoder
     * Default Value: Encoder.REVERSE */
    public double forwardEncoderDirection = Encoder.REVERSE;

    /** The direction of the strafe encoder
     * Default Value: Encoder.FORWARD */
    public double strafeEncoderDirection = Encoder.FORWARD;

    /**
     * This is the IMU that will be used for localization.
     */
    public CustomIMU imu = new RevHubIMU();

    /**
     * This creates a new TwoWheelConstants with default values.
     */
    public TwoWheelConstants() {
        defaults();
    }

    public TwoWheelConstants forwardTicksToInches(double forwardTicksToInches) {
        this.forwardTicksToInches = forwardTicksToInches;
        return this;
    }

    public TwoWheelConstants strafeTicksToInches(double strafeTicksToInches) {
        this.strafeTicksToInches = strafeTicksToInches;
        return this;
    }

    public TwoWheelConstants forwardPodY(double forwardPodY) {
        this.forwardPodY = forwardPodY;
        return this;
    }

    public TwoWheelConstants strafePodX(double strafePodX) {
        this.strafePodX = strafePodX;
        return this;
    }

    public TwoWheelConstants IMU_HardwareMapName(String IMU_HardwareMapName) {
        this.IMU_HardwareMapName = IMU_HardwareMapName;
        return this;
    }

    public TwoWheelConstants forwardEncoder_HardwareMapName(String forwardEncoder_HardwareMapName) {
        this.forwardEncoder_HardwareMapName = forwardEncoder_HardwareMapName;
        return this;
    }

    public TwoWheelConstants strafeEncoder_HardwareMapName(String strafeEncoder_HardwareMapName) {
        this.strafeEncoder_HardwareMapName = strafeEncoder_HardwareMapName;
        return this;
    }

    public TwoWheelConstants IMU_Orientation(RevHubOrientationOnRobot IMU_Orientation) {
        this.IMU_Orientation = IMU_Orientation;
        return this;
    }

    public TwoWheelConstants forwardEncoderDirection(double forwardEncoderDirection) {
        this.forwardEncoderDirection = forwardEncoderDirection;
        return this;
    }

    public TwoWheelConstants strafeEncoderDirection(double strafeEncoderDirection) {
        this.strafeEncoderDirection = strafeEncoderDirection;
        return this;
    }

    public TwoWheelConstants customIMU(CustomIMU customIMU) {
        this.imu = customIMU;
        return this;
    }

    /**
     * This sets the default values for the this.
     */
    public void defaults() {
        forwardTicksToInches = .001989436789;
        strafeTicksToInches = .001989436789;
        forwardPodY = 1;
        strafePodX = -2.5;
        IMU_HardwareMapName = "imu";
        forwardEncoder_HardwareMapName = "leftFront";
        strafeEncoder_HardwareMapName = "rightRear";
        IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        forwardEncoderDirection = Encoder.REVERSE;
        strafeEncoderDirection = Encoder.FORWARD;
        imu = new RevHubIMU();
    }
}
