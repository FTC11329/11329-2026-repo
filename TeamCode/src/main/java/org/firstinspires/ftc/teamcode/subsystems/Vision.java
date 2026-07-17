package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.math.Vector;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.RobotSide;
import org.firstinspires.ftc.teamcode.util.ShapeDetection;
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
        pipelineSwitch(2);
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
        Pose pose;
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
                        // index 0 is touching lever
                        return new BallColor[] {BallColor.Green, BallColor.Purple, BallColor.Purple,
                                                BallColor.Green, BallColor.Purple, BallColor.Purple,
                                                BallColor.Green, BallColor.Purple, BallColor.Purple};


                    } else if (fr.getFiducialId() == 22){
                        // index 0 is touching lever
                        return new BallColor[] {BallColor.Purple, BallColor.Green, BallColor.Purple,
                                                BallColor.Purple, BallColor.Green, BallColor.Purple,
                                                BallColor.Purple, BallColor.Green, BallColor.Purple};


                    } else if (fr.getFiducialId() == 23){
                        // index 0 is touching lever
                        return new BallColor[] {BallColor.Purple, BallColor.Purple, BallColor.Green,
                                                BallColor.Purple, BallColor.Purple, BallColor.Green,
                                                BallColor.Purple, BallColor.Purple, BallColor.Green};


                    } else {
                        // index 0 is touching lever
                        return new BallColor[] {BallColor.Purple, BallColor.Green, BallColor.Purple,
                                                BallColor.Purple, BallColor.Green, BallColor.Purple,
                                                BallColor.Purple, BallColor.Green, BallColor.Purple};


                    }
                }
            }
        }
        return motif;
    }

    /**
     * @return a list of ball poses on the field, with the color, and the time we took the photo
     */
    public List<DetectedBall> searchForBalls(Pose curPose) {
        return searchForBalls(curPose, false, false);
    }
    public List<DetectedBall> searchForBalls(Pose curPose, boolean limitZone, boolean farZone) {
        List<DetectedBall> detectedBalls = new ArrayList<>();
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
            for (LLResultTypes.DetectorResult detection : detections) {
                double tx = detection.getTargetXDegrees(); // Where it is (left-right)
                double ty = detection.getTargetYDegrees(); // Where it is (up-down)
                String className = detection.getClassName(); // What was detected
                BallColor ballColor;
                if (className.equals("green")){
                    ballColor = BallColor.Green;
                } else if (className.equals("purple")) {
                    ballColor = BallColor.Purple;
                } else {
                    ballColor = BallColor.Any;
                }
                long timePhotoWasTaken = result.getControlHubTimeStampNanos();
                Pose ballPose = poseEstimation(tx, ty, curPose);
                if (Math.abs(ballPose.getX()) > 90 || Math.abs(ballPose.getY()) > 90) {
                    continue;
                }
                if (limitZone && ballPose.getX() > -4) {
                    continue;
                }
                detectedBalls.add(new DetectedBall(ballPose, ballColor, timePhotoWasTaken));
            }
        }
        return detectedBalls;
    }

    public Pose poseEstimation(double targetX,double targetY, Pose curpose) {
        return poseEstimation(targetX, targetY, curpose, new Vector(), 0, 0);
    }

    public Pose poseEstimation(double targetX,double targetY, Pose curpose, Vector robotVelocity, double headingVelocity, double latency) {
        //todo: double check these numbers
        double cameraPitch = 0; // zero facing straight forward 90 facing straight up
        double ballRadius = 2.5; // radius of the ball in inches
        double cameraHeight = 10.375; // distance from the camera to the ground in inches
        double cameraOffsetY = 5; // distance Y to center of the chassis
        double cameraOffsetX = 0; // distance X to center of the chassis

        double cameraToBallAngle = Math.toRadians(cameraPitch + targetY);
        double heightDifference = cameraHeight - ballRadius;

        double forwardDistance = heightDifference / Math.tan(cameraToBallAngle); // Y is forward backward
        double lateralDistance = forwardDistance * Math.tan(Math.toRadians(targetX));

        forwardDistance += cameraOffsetY;
        lateralDistance += cameraOffsetX;

        Vector robotToBallVector = new Vector(new Pose(- forwardDistance, lateralDistance));
        robotToBallVector.rotateVector(curpose.getHeading());
        return curpose.plusVector(robotToBallVector);
    }

    public Pose rampPoseEstimation(double targetX,double targetY, Pose curpose) {
        //todo: double check these numbers
        double cameraPitch = 0; // zero facing straight forward 90 facing straight up
        double ballRadius = 2.5; // radius of the ball in inches
        double cameraHeight = 10.375; // distance from the camera to the ground in inches
        double cameraOffsetY = 5; // distance Y to center of the chassis
        double cameraOffsetX = 0; // distance X to center of the chassis
        //This basically reflects the robot to the Red Side so we only need to calculate it assuming its red.
        curpose = (robotSide == RobotSide.Red ? curpose : new Pose(- curpose.getX(), curpose.getY(), Math.PI - curpose.getHeading()));

        double cameraYAngle = Math.toRadians(cameraPitch + targetY);
        double cameraXAngle = curpose.getHeading() + Math.toRadians(targetX);
        //The following computes the Camera distance from a plane spanned by the Z and Y axis on x=72, so its the Ramp plane.
        double robotDstToRamp = 72 - (new Pose(0, cameraOffsetY).rotate(curpose.getHeading(), false).plus(curpose).getY());

        double rightDistance = robotDstToRamp * Math.tan(cameraXAngle); // Y is forward backward
        double upDistance = robotDstToRamp * Math.tan(cameraYAngle);

        return new Pose(rightDistance, upDistance);
    }

    public boolean rampPoseCondition(double targetX,double targetY){
        //todo: double check these numbers
        double cameraPitch = 0; // zero facing straight forward 90 facing straight up
        double ballRadius = 2.5; // radius of the ball in inches
        double cameraHeight = 10.375; // distance from the camera to the ground in inches
        double cameraOffsetY = 5; // distance Y to center of the chassis
        double cameraOffsetX = 0; // distance X to center of the chassis
        double minRampAngle = 0;

        double cameraToBallAngle = Math.toRadians(cameraPitch + targetY);
        return cameraToBallAngle >= minRampAngle;
    }

    public List<BallColor> GetBallsOnRamp(Pose curPose){
        //Get the Balls, copied from the SearchForBalls function
        List<DetectedBall> detectedBalls = new ArrayList<>();
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
            for (LLResultTypes.DetectorResult detection : detections) {
                double tx = detection.getTargetXDegrees(); // Where it is (left-right)
                double ty = detection.getTargetYDegrees(); // Where it is (up-down)
                String className = detection.getClassName(); // What was detected
                BallColor ballColor;
                if (className.equals("green")){
                    ballColor = BallColor.Green;
                } else if (className.equals("purple")) {
                    ballColor = BallColor.Purple;

                } else {
                    ballColor = BallColor.Any;
                }
                long timePhotoWasTaken = result.getControlHubTimeStampNanos();
                //Here we pick only the balls that are on the Ramp
                if (rampPoseCondition(tx,ty)){
                    //Here we calculate their pose based on the fact that their on Ramp.
                    Pose ballPose = rampPoseEstimation(tx, ty, curPose);
                    detectedBalls.add(new DetectedBall(ballPose, ballColor, timePhotoWasTaken));
                }
            }
        }
        //We sort balls according to their x Position, to make sure the colors are ordered
        detectedBalls.sort((a, b) -> {
            double aOrder = a.ballPose.getX();
            double bOrder = b.ballPose.getX();
            // Sorts: Smallest Y to Largest Y
            return Double.compare(aOrder, bOrder);
        });
        //Here we convert the List<DetectedBall> to List<BallColor>
        List<BallColor> ballsOnRamp = new ArrayList<>();
        for (DetectedBall ball : detectedBalls){
            ballsOnRamp.add(ball.ballColor);
        }
        return ballsOnRamp;
    }

    public boolean isInField(Pose pose){
        boolean inX = pose.getX() < 92 && pose.getX() > -92;
        boolean inY = pose.getY() < 92 && pose.getY() > -92;
        return inX && inY;
    }

    public static class DetectedBall {
        public Pose ballPose;
        public BallColor ballColor;
        public long timePhotoWasTaken;
        public Vector velocity;
        DetectedBall(Pose ballPose, BallColor ballColor, long timePhotoWasTaken){
            this.ballColor = ballColor;
            this.ballPose = ballPose;
            this.timePhotoWasTaken = timePhotoWasTaken;
        }
        @Override
        public String toString() {
            return ballPose.toString();
        }
    }

    public void stop() {}
}