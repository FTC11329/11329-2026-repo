package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

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

public class AprilTag {
    public AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private RobotSide robotSide;
    private int goalID;
    public AprilTag(HardwareMap hardwareMap, RobotSide robotSide) {
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

    // --------------- SHOOTER RPM CALCULATOR by Chat GPT --------------
    //
    // Calculates the wheel target RPM required to hit the goal
    // given the robot’s distance from an AprilTag and the shooter’s launch angle.
    //
    // Assumptions:
    // - Launcher exit height: 16 in
    // - Target height: 29 in (so vertical delta = 13 in)
    // - Target is 19 in behind the AprilTag (colinear shot)
    // - Wiffle ball with backspin and drag, approximated empirically
    //
    // Inputs you provide each shot:
    //    double D_tag_in   = distance from robot to AprilTag (in inches)
    //    double phi_deg    = shooter launch angle above horizontal (in degrees)
    //
    // Output:
    //    double targetRPM  = wheel RPM required for that shot
    //
    // ------------------------------------------------------------------

    public double getRPMNeeded(double Distance_to_tag_inch) {
        AprilTagDetection goalDetection = getTagWithID(goalID);
        if (goalDetection == null) {
            return -1;
        }

        // --- Positioning ---
        final double D_tag_in = goalDetection.ftcPose.y;

        // --- Fixed geometry ---
        final double LAUNCHER_HEIGHT_IN = 16.0;   // inches: how high the ball leaves the shooter
        final double TARGET_HEIGHT_IN   = 29.0;   // inches: height of the target off the ground
        final double TAG_TO_TARGET_IN   = 19.0;   // inches: distance from AprilTag to middle of the targeted point

        // --- Physical constants ---
        final double G = 9.81;                   // gravity (m/s^2)
        final double IN_TO_M = 0.0254;           // inches to meters conversion

        // --- Empirical tuning constants ---
        final double K_DRAG = 0.06;              // drag correction per meter (higher = more required speed)
        final double K_SPIN = 0.12;              // lift correction factor from backspin
        final double SPIN_RPM = 1500.0;          // expected backspin of the ball
        final double EFF = 0.95;                 // wheel-to-ball efficiency (0.9–1.0 typical)
        final double K_TUNE = 1.0;               // final tuning multiplier (easy field tuning)

        // --- Shooter hardware parameters ---
        final double R_WHEEL_IN = 2.0;           // wheel radius (inches) //todo
        final double phi_deg = 60;               // Angle of the shot (degrees) // todo
        final double R_WHEEL_M = R_WHEEL_IN * IN_TO_M; // wheel radius in meters

        // --- Derived distances ---
        // Horizontal distance from shooter to target (colinear case)
        double R_in = D_tag_in + TAG_TO_TARGET_IN;
        double R_M  = R_in * IN_TO_M;  // convert to meters

        // Vertical difference between target and launcher (m)
        double DY_M = (TARGET_HEIGHT_IN - LAUNCHER_HEIGHT_IN) * IN_TO_M;  // 13 in = 0.3302 m

        // --- Angle conversions ---
        double phi_rad = Math.toRadians(phi_deg);

        // --- Step 1: Ideal (no drag) launch velocity for ballistic trajectory ---
        // v = sqrt( g * R^2 / [ 2 * cos^2(phi) * (R*tan(phi) - DY) ] )
        double numerator = G * R_M * R_M;
        double denominator = 2.0 * Math.pow(Math.cos(phi_rad), 2) * (R_M * Math.tan(phi_rad) - DY_M);
        double vIdeal = Math.sqrt(numerator / denominator);

        // --- Step 2: Apply empirical corrections for drag & Magnus lift ---
        double vCorrected = vIdeal * (1.0 + K_DRAG * R_M) / (1.0 + K_SPIN * (SPIN_RPM / 1500.0));

        // --- Step 3: Convert linear velocity to wheel RPM ---
        // ω = v / (r * efficiency)
        double targetRPM = K_TUNE * (vCorrected * 60.0) / (2.0 * Math.PI * R_WHEEL_M * EFF);

        return targetRPM;
    }


}
