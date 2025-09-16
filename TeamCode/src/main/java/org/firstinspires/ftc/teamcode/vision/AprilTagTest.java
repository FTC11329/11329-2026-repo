package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.RobotSide;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class AprilTagTest {
    public AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private RobotSide robotSide;
    private int goalID;
    public AprilTagTest(HardwareMap hardwareMap, RobotSide robotSide) {
        this.robotSide = robotSide;
        if (robotSide == RobotSide.Red) {
            goalID = 24;
        } else {
            goalID = 20;
        }
        // Setup the processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setCameraPose(new Position(DistanceUnit.INCH, 0, 0, 0, 0), new YawPitchRollAngles(AngleUnit.DEGREES,0,0, 0, 0))
                .setLensIntrinsics(903.79, 903.79, 699.758, 372.872) //todo IMPORTANT
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();
        // Setup the vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(aprilTagProcessor)
                .build();
    }
    // Gets whatever tags the camera sees
    public ArrayList<AprilTagDetection> getAprilTagList() {
        return aprilTagProcessor.getDetections();
    }

    //returns null if id isnt found
    public AprilTagDetection getTagWithID(int id) {
        for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }


    public double getRPMNeeded() {
        AprilTagDetection goalDetection = getTagWithID(goalID);
        if (goalDetection == null) {
            return -1;
        }
        // --- Constants you should configure/tune ---
        // Geometry
        final double LAUNCHER_HEIGHT_IN = 16.0;   // inches: where the ball leaves the shooter
        final double TARGET_HEIGHT_IN   = 29.0;   // inches: height of the target off the ground
        final double TAG_TO_TARGET_IN   = 19.0;   // inches: distance from AprilTag to middle of the hole

        // Physics constants
        final double G = 9.81;                   // m/s^2: gravity

        // Ball / air parameters (empirical, //todo tune me!)
        final double K_DRAG = 0.06;              // drag correction per meter (higher = more slowdown)
        final double K_SPIN = 0.12;              // lift correction factor from backspin
        final double SPIN_RPM = 1500.0;          // approximate ball backspin (rpm)

        // Shooter wheel parameters
        final double R_WHEEL_IN = 2.0;           // inches: shooter wheel radius
        final double R_WHEEL_M = R_WHEEL_IN * 0.0254; // convert to meters (don't tune)
        final double EFF = 0.95;                 // efficiency factor (0.9–1.0, accounts for slip)

        // Tuning factor
        final double K_TUNE = 1.0;               // final multiplier you can change without touching model

        // ---//todo Inputs for each shot ---
        // phi = chosen launch angle in degrees
        // D_tag_in = distance to AprilTag in inches
        double phi = 50.0;       // example angle
        double D_tag_in = goalDetection.ftcPose.y; // example distance to AprilTag in inches

        // --- Compute horizontal distance to target in meters ---
        // If target is directly behind the AprilTag (colinear case):
        double R_in = D_tag_in + TAG_TO_TARGET_IN;
        double R_M  = R_in * 0.0254;

        // Vertical difference (target height - launcher height), meters
        double DY_M = (TARGET_HEIGHT_IN - LAUNCHER_HEIGHT_IN) * 0.0254;

        double rpm = K_TUNE * (60.0 / (2.0 * Math.PI * R_WHEEL_M * EFF)) *
                (Math.sqrt((G * R_M * R_M) /
                        (2.0 * Math.pow(Math.cos(Math.toRadians(phi)), 2) *
                                (R_M * Math.tan(Math.toRadians(phi)) - DY_M))) *
                        (1.0 + K_DRAG * R_M) /
                        (1.0 + K_SPIN * (SPIN_RPM / 1500.0)));

        return rpm;

    }

}
