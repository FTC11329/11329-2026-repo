package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.teamcode.pedroPathing.control.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.FollowerConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.core.FollowerBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.core.drivetrains.MecanumConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.core.localization.constants.PinpointConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathConstraints;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.97)
            .forwardZeroPowerAcceleration(-46.54367)
            .lateralZeroPowerAcceleration(-87.7)
            .zeroPowerAccelerationMultiplier(0.4)
            .lateralZeroPowerAcceleration(-20)
            .useSecondaryDrivePIDF(false)
            .useSecondaryHeadingPIDF(true)
            .headingPIDFCoefficients(new PIDFCoefficients(0.85, 0, 0.006, 0.05))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(3.0, 0.02, 0.15, 0.03))
            .headingPIDFSwitch(0.1)
            .useSecondaryTranslationalPIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.107, 0, 0.012, 0.048))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.21, 0, 0.018, 0.027))
            .integralSecondaryTranslational(new PIDFCoefficients(0,0,0,0.12))
            .useSecondaryTranslationalPIDF(true)
            .translationalPIDFSwitch(1)
            ;
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5.866)
            .strafePodX(-2.3622)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftBack")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorEx.Direction.FORWARD)
            .xVelocity(72.39374)
            .yVelocity(57.32301)
            ;
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build()
                ;
    }
}