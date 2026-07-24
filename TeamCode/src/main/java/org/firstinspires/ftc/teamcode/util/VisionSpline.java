package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.modularAutos.Common;
import org.firstinspires.ftc.teamcode.modularAutos.CommonCRI;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.Path;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;

public class VisionSpline {

    public static final int STRENGTH = 4;
    public static final double zHeight = -69;
    public static final double zVel = -28;
    public static final double zFree = 3;
    public static final double aRandomvariable = -85;
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
            if (firstBall.getX() > zHeight) {
                poses.add(firstBall.plus(new Pose(zVel, Common.wasLastRed ? zFree : -zFree)));
            } else {
                poses.add(new Pose(aRandomvariable, firstBall.getY(), firstBall.getHeading()));
            }
        }
        
        if (secondBall != null) {
            for (int i = 0; i < STRENGTH; i++) {
                if (secondBall.getX() > zHeight) {
                    poses.add(secondBall.plus(new Pose(zVel, Common.wasLastRed ? zFree : -zFree)));
                } else {
                    poses.add(new Pose(aRandomvariable, secondBall.getY(), secondBall.getHeading()));
                }
            }
        }

        if (thirdBall != null) {
            for (int i = 0; i < STRENGTH; i++) {
                if (thirdBall.getX() > zHeight) {
                    poses.add(thirdBall.plus(new Pose(zVel, Common.wasLastRed ? zFree : -zFree)));
                } else {
                    poses.add(new Pose(aRandomvariable, thirdBall.getY(), thirdBall.getHeading()));
                }
            }
        }
        Path visionPath = new Path(new BezierCurve(poses));
        visionPath.setTangentHeadingInterpolation();
//        visionPath.setConstantHeadingInterpolation(Math.toRadians(CommonCRI.wasLastRed ? -45 : 45));
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

        for (int i = 0; i < poses.size(); i++) {
            Pose p = poses.get(i).ballPose;

            double distance = target.distanceFrom(p);
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
