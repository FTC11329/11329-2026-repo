package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.math.Matrix;
import org.firstinspires.ftc.teamcode.util.RobotSide;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

public class Vision {
    RobotSide robotSide;
    Limelight3A limelight;

    Pose redTag = Constants.Vision.redTag;
    Pose blueTag = Constants.Vision.blueTag;

    Pose cameraPos = Constants.Vision.cameraPos;

    double pitch = Constants.Vision.pitch;

    public Vision(HardwareMap hardwareMap, RobotSide robotSide){
        this.robotSide = robotSide;
        limelight = hardwareMap.get(Limelight3A .class, "limelight");
        limelight.start();
    }

    public Pose getRobotPose() {
        //Creating a 3d array to store the distances of each block for comparison
        LLResult result = limelight.getLatestResult();
        Pose pose = null;
        if (result != null) {
            if (result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    Pose3D fiducial = fr.getTargetPoseCameraSpace();
                    Pose3D camToTag = rotatePose3D(fiducial, 0, pitch, 0);
                    Pose botToTag = new Pose(camToTag.getPosition().x, camToTag.getPosition().y).plus(cameraPos);
                    Pose botToRealTag = botToTag.rotate(fiducial.getOrientation().getYaw(AngleUnit.RADIANS), false);
                    Pose botToField = botToRealTag;

                    if (fr.getFiducialId() == 20) {
                        botToField = botToField.plus(redTag);
                    } else if (fr.getFiducialId() == 24) {
                        botToField = botToField.plus(blueTag);
                    }
                    pose = botToField;

                }
            }
        }
        return pose;
    }

    public static Pose3D rotatePose3D(Pose3D pose, double yaw, double pitch, double roll) {

        // Build rotation matrix from yaw, pitch, roll
        double cy = Math.cos(yaw);
        double sy = Math.sin(yaw);
        double cp = Math.cos(pitch);
        double sp = Math.sin(pitch);
        double cr = Math.cos(roll);
        double sr = Math.sin(roll);

        // Rotation matrix R = Rz(yaw) * Ry(pitch) * Rx(roll)
        double[][] R = {
                { cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr },
                { sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr },
                {     -sp,               cp * sr,               cp * cr }
        };

        // Rotate the translation vector
        Position t = pose.getPosition();
        double x = R[0][0] * t.x + R[0][1] * t.y + R[0][2] * t.z;
        double y = R[1][0] * t.x + R[1][1] * t.y + R[1][2] * t.z;
        double z = R[2][0] * t.x + R[2][1] * t.y + R[2][2] * t.z;
        Position rotatedPosition = new Position(DistanceUnit.METER, x, y, z, 0);

        // Rotate the orientation as well (Pose3D uses yaw, pitch, roll)
        double newYaw   = pose.getOrientation().getYaw()   + yaw;
        double newPitch = pose.getOrientation().getPitch() + pitch;
        double newRoll  = pose.getOrientation().getRoll()  + roll;

        return new Pose3D(rotatedPosition, new YawPitchRollAngles(AngleUnit.RADIANS, newYaw, newPitch, newRoll, 0));
    }

    public double distanceXToGoal(Pose robotPose) {
        Pose goalPose = new Pose(72, -72);
        Pose distancePose = goalPose.minus(robotPose);
        return distancePose.returnPolar()[0];
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

    public double getRPMNeeded() {
        // todo: insert code to get robot pose
        Pose robotPose = getRobotPose(); //here

        // --- Positioning ---
        final double D_tag_in = distanceXToGoal(robotPose);

        // --- Fixed geometry ---
        final double LAUNCHER_HEIGHT_IN = 12.063;   // inches: how high the ball leaves the shooter
        final double TARGET_HEIGHT_IN   = 29.0;   // inches: height of the target off the ground
        final double TAG_TO_TARGET_IN   = 19.0;   // inches: distance from AprilTag to middle of the targeted point

        // --- Physical constants ---
        final double G = 9.81;                   // gravity (m/s^2)
        final double IN_TO_M = 0.0254;           // inches to meters conversion

        // --- Empirical tuning constants ---
        final double K_DRAG = 0.06;              // drag correction per meter (higher = more required speed)
        final double K_SPIN = 0.28; //0.12     // lift correction factor from backspin
        final double SPIN_RPM = 0;               // expected backspin of the ball
        final double EFF = 0.95;                 // wheel-to-ball efficiency (0.9–1.0 typical)
        final double K_TUNE = 1.0;               // final tuning multiplier (easy field tuning)

        // --- Shooter hardware parameters ---
        final double R_WHEEL_IN = 1.0;           // wheel radius (inches) //todo
        final double phi_deg = 50;               // 5-50 Angle of the shot (degrees) // todo
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

        return targetRPM; // Conversion for gear ratio
    }

}
