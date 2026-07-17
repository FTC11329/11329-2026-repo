package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.modularAutos.CommonCRI;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.Path;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;

public class VisionSpline {

    public static final int STRENGTH = 3;
    /**
     * Main pipeline:
     * Selects up to 3 closest balls in sequence from a target pose
     */
    public static Path getSplinePathForVision(List<Vision.DetectedBall> detections, Pose robotPose) {

        // Defensive copy so we don’t modify original list
        List<Vision.DetectedBall> remaining = new ArrayList<>(detections);

        Pose firstBall = findAndRemoveClosest(robotPose, remaining);
        Pose secondBall = findAndRemoveClosest(firstBall, remaining);
        Pose thirdBall = findAndRemoveClosest(secondBall, remaining);

        if (firstBall == null) {
            return null;
        }

        List<Pose> poses = new ArrayList<>();
        poses.add(robotPose);

        for (int i = 0; i < STRENGTH; i++) {
            poses.add(firstBall.plus(new Pose(-3, 0)));
        }
        
        if (secondBall != null) {
            for (int i = 0; i < STRENGTH; i++) {
                poses.add(secondBall.plus(new Pose(-2, 0)));
            }
        }

        if (thirdBall != null) {
            for (int i = 0; i < STRENGTH; i++) {
                poses.add(thirdBall.plus(new Pose(-2, 0)));
            }
        }
        Path visionPath = new Path(new BezierCurve(poses));
        visionPath.setConstantHeadingInterpolation(Math.toRadians(CommonCRI.wasLastRed ? -45 : 45));
        return visionPath;
    }

    /**
     * Finds closest pose and removes it from the list
     */
    private static Pose findAndRemoveClosest(Pose target, List<Vision.DetectedBall> poses) {

        if (target == null || poses == null || poses.isEmpty()) {
            return null;
        }

        Pose closest = null;
        double minDistance = Double.MAX_VALUE;
        int closestIndex = -1;
        double height = target.getX();

        for (int i = 0; i < poses.size(); i++) {
            Pose p = poses.get(i).ballPose;

            double distance = target.distanceFrom(p);
            if (p.getX() < height) {
                continue;
            }
            if (distance < minDistance) {
                minDistance = distance;
                closest = p;
                closestIndex = i;
            }
        }

        // Remove so it won’t be picked again
        if (closestIndex != -1) {
            poses.remove(closestIndex);
        }

        return closest;
    }
}
