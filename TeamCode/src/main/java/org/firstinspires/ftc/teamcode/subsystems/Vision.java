package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.RobotSide;
import org.firstinspires.ftc.teamcode.util.shooterInterpolation.ShooterState;
import org.firstinspires.ftc.teamcode.util.shooterInterpolation.ShooterTestValues;


import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.List;

public class Vision {
    RobotSide robotSide;
    Limelight3A limelight;

    public int seen = 0;

    public Vision(HardwareMap hardwareMap, RobotSide robotSide){
        this.robotSide = robotSide;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    public void start() {
        pipelineSwitch(0);
    }

    public void pipelineSwitch(int index) {
        limelight.pipelineSwitch(index);
    }
    public int getPipeline() {
        return limelight.getStatus().getPipelineIndex();
    }


    private LLResult cachedResult;

    public void update() {
        cachedResult = limelight.getLatestResult();
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
                    pose = new Pose(-robotPoseWeirdM.getPosition().x * 39.37,
                            -robotPoseWeirdM.getPosition().y * 39.37,
                            Math.toRadians(robotPoseWeirdM.getOrientation().getYaw() - 180));
                    averagingList.add(pose);
                }
            }
        }
        return pose;
    }
    public void clearPoseList() {
        averagingList.clear();
    }
    Pose previousPose = new Pose();
    public Pose averageRobotPose() {
        //Creating a 3d array to store the distances of each block for comparison
        LLResult result = limelight.getLatestResult();
        Pose pose = null;
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                Pose3D robotPoseWeirdM = fr.getRobotPoseFieldSpace();
                pose = new Pose(-robotPoseWeirdM.getPosition().x * 39.37,
                        -robotPoseWeirdM.getPosition().y * 39.37,
                        Math.toRadians(robotPoseWeirdM.getOrientation().getYaw() - 180));
                if (pose.distanceFrom(previousPose) > 1e-6) {averagingList.add(pose);}
                previousPose = pose;
            }
        }
        if (!averagingList.isEmpty()) {
            return averagePoses(averagingList);
        }
        return null;
    }
    private final List<Pose> averagingList = new ArrayList<>();
    public int listLength() {
        return averagingList.size();
    }
    public static Pose averagePoses(List<Pose> poses) {

        if (poses == null || poses.isEmpty()) return null;

        double sumX = 0;
        double sumY = 0;

        double sumSin = 0;
        double sumCos = 0;

        int count = 0;

        for (Pose p : poses) {
            if (p == null) continue;

            sumX += p.getX();
            sumY += p.getY();

            sumSin += Math.sin(p.getHeading());
            sumCos += Math.cos(p.getHeading());

            count++;
        }

        if (count == 0) return null;

        double avgX = sumX / count;
        double avgY = sumY / count;

        double avgHeading = Math.atan2(sumSin / count, sumCos / count);

        return new Pose(avgX, avgY, avgHeading);
    }

    public BallColor[] getMotif() {
        //Creating a 3d array to store the distances of each block for comparison
        BallColor[] motif = null;
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    if (fr.getFiducialId() == 21){
                        motif = new BallColor[]{BallColor.Green, BallColor.Purple, BallColor.Purple};
                    } else if (fr.getFiducialId() == 22){
                        motif = new BallColor[]{BallColor.Purple, BallColor.Green, BallColor.Purple};
                    } else if (fr.getFiducialId() == 23){
                        motif = new BallColor[]{BallColor.Purple, BallColor.Purple, BallColor.Green};
                    } else {
                        motif = new BallColor[]{BallColor.Purple, BallColor.Green, BallColor.Purple};
                    }
                }
            }
        }
        return motif;
    }

    /**
     * @return a list of ball poses relative to the center of the robot chassis, with the color, and if you
     */
    public List<DetectedBall> searchForBalls() {
        List<DetectedBall> detectedBalls = new ArrayList<>();
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
            for (LLResultTypes.DetectorResult detection : detections) {
                String className = detection.getClassName(); // What was detected
                BallColor ballColor;
                if (className.equals("green")){
                    ballColor = BallColor.Green;
                } else if (className.equals("purple")) {
                    ballColor = BallColor.Purple;
                } else {
                    ballColor = BallColor.Any;
                }
                double tx = detection.getTargetXDegrees(); // Where it is (left-right)
                double ty = detection.getTargetYDegrees(); // Where it is (up-down)
                Pose ballPose = poseEstimation(tx, ty);
                long timePhotoWasTaken = result.getControlHubTimeStamp();
                detectedBalls.add(new DetectedBall(ballPose, ballColor, timePhotoWasTaken));
            }
        }
        return detectedBalls;
    }
    public Pose poseEstimation(double targetX,double targetY) {
        //todo: double check these numbers
            double cameraPitch = Math.toRadians(110); // zero facing straight down 180 facing straight up
        double ballRadius = 2.5; // radius of the ball in inches
        double cameraHeight = 12; // distance from the camera to the ground in inches
        double cameraOffsetX = 0; // distance X to center of the chassis
        double cameraOffsetY = 8; // distance Y to center of the chassis
        
        double cameraToBallAngle = cameraPitch - targetY;
        double heightOfPointOnBall = (ballRadius * Math.cos(cameraToBallAngle)) + ballRadius;
        double heightDifference = cameraHeight - heightOfPointOnBall;
        double distanceToBallY = (ballRadius * Math.sin(cameraToBallAngle)) 
                + (heightDifference * Math.tan(cameraToBallAngle)); // Y is forward backward
        double distanceToBallX = distanceToBallY * Math.tan(targetX);
        return new Pose(distanceToBallX + cameraOffsetX, distanceToBallY + cameraOffsetY);
    }
    public static class DetectedBall {
        public Pose ballPose;
        public BallColor ballColor;
        public long timePhotoWasTaken;
        DetectedBall(Pose ballPose, BallColor ballColor, long timePhotoWasTaken){
            this.ballColor = ballColor;
            this.ballPose = ballPose;
            this.timePhotoWasTaken = timePhotoWasTaken;
        }
    }

    public void stop() {}
}