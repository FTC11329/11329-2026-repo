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
import org.firstinspires.ftc.teamcode.util.BallColor;
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
    double tagAngle = Constants.Vision.tagAngle;


    public Vision(HardwareMap hardwareMap, RobotSide robotSide){
        this.robotSide = robotSide;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
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
                    Pose3D robotPoseWeirdM = fr.getRobotPoseFieldSpace();
                    pose = new Pose(-robotPoseWeirdM.getPosition().x * 39.37, -robotPoseWeirdM.getPosition().y * 39.37, Math.toRadians(-robotPoseWeirdM.getOrientation().getYaw()));
                }
            }
        }
        return pose;
    }

    public BallColor[] getMotif() {
        //Creating a 3d array to store the distances of each block for comparison
        BallColor[] motif = null;
        LLResult result = limelight.getLatestResult();
        Pose pose = new Pose();
        if (result != null) {
            if (result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    if (fr.getFiducialId() == 21){
                        motif = new BallColor[]{BallColor.Green, BallColor.Green, BallColor.Purple};
                    }
                    if (fr.getFiducialId() == 22){
                        motif = new BallColor[]{BallColor.Green, BallColor.Purple, BallColor.Green};
                    }
                    if (fr.getFiducialId() == 23){
                        motif = new BallColor[]{BallColor.Purple, BallColor.Green, BallColor.Green};
                    }
                }
            }
        }
        return motif;
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
    //    double vCorrected  = velocity of ball needed for shot
    //
    // ------------------------------------------------------------------

    public double getVelocity() {
        // get robot pose
        Pose robotPose = getRobotPose();

        // --- Positioning ---
        final double D_tag_in = distanceXToGoal(robotPose);

        // --- Derived distances ---
        // Horizontal distance from shooter to target (colinear case)
        double R_in = D_tag_in + Constants.ShooterParamaters.TAG_TO_TARGET_IN;
        double R_M  = R_in * Constants.ShooterParamaters.IN_TO_M;  // convert to meters

        // Vertical difference between target and launcher (m)
        double DY_M = (Constants.ShooterParamaters.TARGET_HEIGHT_IN - Constants.ShooterParamaters.LAUNCHER_HEIGHT_IN) * Constants.ShooterParamaters.IN_TO_M;  // 13 in = 0.3302 m

        // --- Angle conversions ---
        double phi_rad = Math.toRadians(Constants.ShooterParamaters.phi_deg);

        // --- Step 1: Ideal (no drag) launch velocity for ballistic trajectory ---
        // v = sqrt( g * R^2 / [ 2 * cos^2(phi) * (R*tan(phi) - DY) ] )
        double numerator = Constants.ShooterParamaters.G * R_M * R_M;
        double denominator = 2.0 * Math.pow(Math.cos(phi_rad), 2) * (R_M * Math.tan(phi_rad) - DY_M);
        double vIdeal = Math.sqrt(numerator / denominator);

        // --- Step 2: Apply empirical corrections for drag & Magnus lift ---
        double vCorrected = vIdeal * (1.0 + Constants.ShooterParamaters.K_DRAG * R_M) / (1.0 + Constants.ShooterParamaters.K_SPIN * (Constants.ShooterParamaters.SPIN_RPM / 1500.0));
        return vCorrected;
    }

    public double getRPMNeeded(){
        double vCorrected = getVelocity();

        // --- Step 3: Convert linear velocity to wheel RPM ---
        // ω = v / (r * efficiency)
        double targetRPM = Constants.ShooterParamaters.K_TUNE * (vCorrected * 60.0) / (2.0 * Math.PI * Constants.ShooterParamaters.R_WHEEL_M * Constants.ShooterParamaters.EFF);

        return targetRPM; // Conversion for gear ratio
    }


    public double[] getVelocityTime(double time, Pose robotPose, Pose velocity) {
        Pose goalPose = new Pose(72, -72);
        Pose distancePose = goalPose.minus(robotPose);
        double dst = distancePose.returnPolar()[0];
        double heading = distancePose.returnPolar()[1];
        double x0 = (dst + Constants.ShooterParamaters.TAG_TO_TARGET_IN);
        double y0 = (Constants.ShooterParamaters.TARGET_HEIGHT_IN - Constants.ShooterParamaters.LAUNCHER_HEIGHT_IN);
        double vx = x0 / time;
        double vy = y0 / time + 0.5 * Constants.ShooterParamaters.G * time;

        double vrx = Math.sin(heading) * vx - velocity.getX();
        double vry = - Math.cos(heading) * vx - velocity.getY();

        vx = Math.sqrt(vrx * vrx + vry * vry);
        heading = Math.atan2(-vrx, vry);

        double v = Math.sqrt(vx * vx + vy * vy);
        double angle = Math.atan2(vy, vx);

        return new double[] {heading, angle, v};
    }

    public double[] getVelocityFinalAngle(double angleDeg, Pose robotPose, Pose velocity) {
        Pose goalPose = new Pose(72, -72);
        Pose distancePose = goalPose.minus(robotPose);
        double dst = distancePose.returnPolar()[0];
        double heading = distancePose.returnPolar()[1];
        double x0 = (dst + Constants.ShooterParamaters.TAG_TO_TARGET_IN);
        double y0 = (Constants.ShooterParamaters.TARGET_HEIGHT_IN - Constants.ShooterParamaters.LAUNCHER_HEIGHT_IN);
        double c = Math.tan(Math.toRadians(angleDeg));
        double vx = Math.sqrt(0.5 * Constants.ShooterParamaters.G * x0 * x0 / (y0 - c * x0));
        double vy = vx * c + Constants.ShooterParamaters.G * x0 / vx;

        double vrx = Math.sin(heading) * vx - velocity.getX();
        double vry = - Math.cos(heading) * vx - velocity.getY();

        vx = Math.sqrt(vrx * vrx + vry * vry);
        heading = Math.atan2(-vx, vy);

        double v = Math.sqrt(vx * vx + vy * vy);
        double angle = Math.atan2(vy, vx);

        return new double[] {heading, angle, v};
    }

    public double[] getVelocityMaxHeight(double maxHeight, Pose robotPose, Pose velocity) {
        double uy = Math.sqrt(2 * maxHeight * Constants.ShooterParamaters.G);
        double y0 = (Constants.ShooterParamaters.TARGET_HEIGHT_IN - Constants.ShooterParamaters.LAUNCHER_HEIGHT_IN);
        double discriminant = uy * uy - 2 * Constants.ShooterParamaters.G * y0;
        if (discriminant < 0) {
            // no real solution: the desired max height is impossible
            return null;
        }
        double t = (uy + Math.sqrt(discriminant)) / Constants.ShooterParamaters.G; // choose physically valid root
        return getVelocityTime(t, robotPose, velocity);
    }

    public double[] getVelocityRPM(double rpm, Pose robotPose, Pose velocity) {
        double v = Math.pow(RPMToVelocity(rpm),2);
        Pose goalPose = new Pose(72, -72);
        Pose distancePose = goalPose.minus(robotPose);
        double dst = distancePose.returnPolar()[0];
        double heading = distancePose.returnPolar()[1];
        double x0 = (dst + Constants.ShooterParamaters.TAG_TO_TARGET_IN);
        double y0 = (Constants.ShooterParamaters.TARGET_HEIGHT_IN - Constants.ShooterParamaters.LAUNCHER_HEIGHT_IN);
        double a = 0.25 * Constants.ShooterParamaters.G * Constants.ShooterParamaters.G;
        double b = y0 * Constants.ShooterParamaters.G - v;
        double c = x0 * x0 + y0 * y0;
        double t2 = (- b + Math.sqrt(b * b - 4 * a * c)) / (2 * a);
        return getVelocityTime(Math.sqrt(t2), robotPose, velocity);
    }

    public double velocityToRPM(double velocity){
        double ratio = Constants.ShooterParamaters.H_WHEEL_IN / Constants.ShooterParamaters.R_WHEEL_IN;
        double vf = 2 * velocity / (ratio + 1);
        double wf = vf / Constants.ShooterParamaters.R_WHEEL_IN;
        return Constants.ShooterParamaters.K_TUNE * wf / Constants.ShooterParamaters.MotorToWheel;
    }

    public double RPMToVelocity(double rpm){
        double ratio = Constants.ShooterParamaters.H_WHEEL_IN / Constants.ShooterParamaters.R_WHEEL_IN;

        double wf = rpm * Constants.ShooterParamaters.MotorToWheel / Constants.ShooterParamaters.K_TUNE;
        double vf = wf * Constants.ShooterParamaters.R_WHEEL_IN;
        return 0.5 * vf * (ratio + 1);
    }

    public double[] getRPM (Pose robotPose, double initalCondition, InitialCondition initialConditionType, Pose velocity){
        double[] info = null;
        switch (initialConditionType){
            case Time:
                info = getVelocityTime(initalCondition, robotPose, velocity);
            case MaxHeight:
                info = getVelocityMaxHeight(initalCondition, robotPose, velocity);
            case FinalAngle:
                info = getVelocityFinalAngle(initalCondition, robotPose, velocity);
            case RPM:
                info = getVelocityRPM(initalCondition, robotPose, velocity);
        }
        if (info != null){
            info[2] = velocityToRPM(info[2]);
        }
        return info;
    }

    public enum InitialCondition{
        RPM, MaxHeight, Time, FinalAngle
    }
}
