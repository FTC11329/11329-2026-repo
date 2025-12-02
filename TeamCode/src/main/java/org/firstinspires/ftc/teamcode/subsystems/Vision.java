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
import org.firstinspires.ftc.teamcode.util.shooterInterpolation.ShooterState;
import org.firstinspires.ftc.teamcode.util.shooterInterpolation.ShooterTestValues;


import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

public class Vision {
    ShooterTestValues shooterTestValues;


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

        shooterTestValues = new ShooterTestValues();
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

    ///Gets the shooter Params
    /// Returns an array: [Turret Angle in Degrees,
    /// Hood Angle from X axis in degrees,
    /// and RPM of Motor]
    public double[] getShooterParams(Pose robotPose, Pose robotVelocity) {
        // This is the goal pose of the goal based on what color the robot is playing
        Pose goalPose = robotSide == RobotSide.Blue ? blueTag : redTag;
        // This is the vector pointing at the goal from the robot
        Pose distancePose = goalPose.minus(robotPose);
        // This extracts the distance and angle
        double dst = distancePose.returnPolar()[0];
        double heading = distancePose.returnPolar()[1];
        // This extracts what the optimal shooting paramaters are from interpolating tested values
        ShooterState shooterState = shooterTestValues.get(dst);
        // This converts the Hood Angle and RPM to actual velocity vectors
        double vx = Math.cos(Math.toRadians(shooterState.hoodAngle)) * RPMToVelocity(shooterState.rpm);
        double vy = Math.sin(Math.toRadians(shooterState.hoodAngle)) * RPMToVelocity(shooterState.rpm);

        // This accounts for robot velocity
        double vrx = Math.sin(heading) * vx - robotVelocity.getX();
        double vry = - Math.cos(heading) * vx - robotVelocity.getY();

        vx = Math.sqrt(vrx * vrx + vry * vry);
        heading = Math.atan2(-vrx, vry);

        double v = Math.sqrt(vx * vx + vy * vy);
        double angle = Math.atan2(vy, vx);

        return new double[] {Math.toDegrees(heading - robotPose.getHeading()), Math.toDegrees(angle), velocityToRPM(v)};
    }

    public double distanceXToGoal(Pose robotPose) {
        Pose goalPose = new Pose(72, -72);
        Pose distancePose = goalPose.minus(robotPose);
        return distancePose.returnPolar()[0];
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
        heading = Math.atan2(-vrx, vry);

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
        double t2 = (- b + Math.sqrt(Math.abs(b * b - 4 * a * c))) / (2 * a);
        return getVelocityTime(Math.sqrt(Math.abs(t2)), robotPose, velocity);
    }

    public double velocityToRPM(double velocity){
        double ratio = Constants.ShooterParamaters.H_WHEEL_IN / Constants.ShooterParamaters.R_WHEEL_IN;
        double vf = 2 * velocity / (ratio + 1);
        double wf = vf / Constants.ShooterParamaters.R_WHEEL_IN;
        return wf / Constants.ShooterParamaters.MotorToWheel;
    }

    public double RPMToVelocity(double rpm){
        double ratio = Constants.ShooterParamaters.H_WHEEL_IN / Constants.ShooterParamaters.R_WHEEL_IN;

        double wf = rpm * Constants.ShooterParamaters.MotorToWheel;
        double vf = wf * Constants.ShooterParamaters.R_WHEEL_IN;
        return 0.5 * vf * (ratio + 1);
    }

    public double[] getRPM (Pose robotPose, double initalCondition, InitialCondition initialConditionType, Pose velocity){
        double[] info = null;
        switch (initialConditionType){
            case Time:
                info = getVelocityTime(initalCondition, robotPose, velocity);
                break;
            case MaxHeight:
                info = getVelocityMaxHeight(initalCondition, robotPose, velocity);
                break;
            case FinalAngle:
                info = getVelocityFinalAngle(initalCondition, robotPose, velocity);
                break;
            case RPM:
                info = getVelocityRPM(initalCondition, robotPose, velocity);
                break;
        }
        if (info != null){
            info[2] = velocityToRPM(info[2]);
        }
        return info;
    }

    public enum InitialCondition{
        RPM, MaxHeight, Time, FinalAngle //todo add hood angle
    }
}