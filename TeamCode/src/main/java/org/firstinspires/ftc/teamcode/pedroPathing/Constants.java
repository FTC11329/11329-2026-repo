package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.teamcode.pedroPathing.control.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.control.PredictiveBrakingCoefficients;
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
            .mass(13.5)
            .forwardZeroPowerAcceleration(-46.54367)
            .lateralZeroPowerAcceleration(-87.7)
            .lateralZeroPowerAcceleration(-20)
            .useSecondaryDrivePIDF(false)
            .useSecondaryHeadingPIDF(false)
            .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0, 0.15, 0))
            .useSecondaryTranslationalPIDF(false)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.107, 0, 0.012, 0.048))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.21, 0, 0.018, 0.027))
            .integralSecondaryTranslational(new PIDFCoefficients(0,0,0,0.12))
            .useSecondaryTranslationalPIDF(true)
            .translationalPIDFSwitch(1)
            .centripetalScaling(0.0005)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.115, 0.1492112333, 0.001217005667)) // (kP, kLinear, kQuadratic)
            .setUsePredictiveBraking(true)
            ;
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .strafePodX(-3.431)
            .forwardPodY(-6.191)
//            .strafePodX(-2.3622) prev
//            .forwardPodY(-5.866) prev
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .yawScalar(1.00022675)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.925,
            100,
            1,
            1);

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