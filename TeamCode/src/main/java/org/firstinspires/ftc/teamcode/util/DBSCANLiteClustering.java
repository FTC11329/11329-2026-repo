package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;

public class DBSCANLiteClustering {

    // Radius to consider points part of the same cluster (tune this!)
    public static double radius = 9.0;

    // Minimum number of points to consider a valid cluster
//    public int minPoints = 3;

    static public Pose findLargestCluster(List<Vision.DetectedBall> poseList, boolean sort) {
        return findLargestCluster(poseList, sort, 3);
    }
    static public Pose findLargestCluster(List<Vision.DetectedBall> ballList, boolean sort, int minPoints) {

        // Handle empty input
        if (ballList == null || ballList.isEmpty()) {
            throw new RuntimeException("wow");
//            return null;
        }

        int bestCount = 0;
        boolean bestVeriety = false;
        Pose bestCenter = null;

        // For each point, treat it as a potential cluster center
        for (Vision.DetectedBall seed : ballList) {

            List<Vision.DetectedBall> neighbors = new ArrayList<>();

            // Find all neighbors within radius
            for (Vision.DetectedBall other : ballList) {
                if (seed.ballPose.distanceFrom(other.ballPose) <= radius) {
                    neighbors.add(other);
                }
            }

            // Skip small/noisy clusters
            if (neighbors.size() < minPoints) {
                continue;
            }

            // Compute centroid of this cluster
            Pose sum = new Pose();
            for (Vision.DetectedBall p : neighbors) {
                sum = sum.plus(p.ballPose);
            }
            Pose center = sum.times(1.0 / neighbors.size());

            // Use density (count + tightness)
            double mse = 0.0;
            for (Vision.DetectedBall p : neighbors) {
                double d = center.distanceFrom(p.ballPose);
                mse += d * d;
            }
            mse /= neighbors.size();

            double score = neighbors.size() / (mse + 1e-6);

            // Define if there is variety
            int numPurp = 0;
            for (Vision.DetectedBall ball : ballList) {
                if (ball.ballColor == BallColor.Purple) {
                    numPurp ++;
                }
            }
            boolean hasVariety = numPurp == 2;

            // Pick best cluster
            if (bestCenter == null || score > bestCount) {
                bestCount = (int) score;
                bestCenter = center;
            }
        }

        // Fallback: if no cluster met minPoints, pick closest point to origin (or return null)
        if (bestCenter == null) {
            if (minPoints > 1) {
                return findLargestCluster(ballList, sort, minPoints - 1);
            } else {
                return ballList.get(0).ballPose;
            }
        }

        return bestCenter;
    }
}