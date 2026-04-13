package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;

import java.util.ArrayList;
import java.util.List;

public class DBSCANLiteClustering {

    // Radius to consider points part of the same cluster (tune this!)
    public double radius = 5.0;

    // Minimum number of points to consider a valid cluster
    public int minPoints = 2;

    public Pose findLargestCluster(List<Pose> poseList) {

        // Handle empty input
        if (poseList == null || poseList.isEmpty()) {
            return null;
        }

        int bestCount = 0;
        Pose bestCenter = null;

        // For each point, treat it as a potential cluster center
        for (Pose seed : poseList) {

            List<Pose> neighbors = new ArrayList<>();

            // Find all neighbors within radius
            for (Pose other : poseList) {
                if (seed.distanceFrom(other) <= radius) {
                    neighbors.add(other);
                }
            }

            // Skip small/noisy clusters
            if (neighbors.size() < minPoints) {
                continue;
            }

            // Compute centroid of this cluster
            Pose sum = new Pose();
            for (Pose p : neighbors) {
                sum = sum.plus(p);
            }
            Pose center = sum.times(1.0 / neighbors.size());

            // Use density (count + tightness)
            double mse = 0.0;
            for (Pose p : neighbors) {
                double d = center.distanceFrom(p);
                mse += d * d;
            }
            mse /= neighbors.size();

            double score = neighbors.size() / (mse + 1e-6);

            // Pick best cluster
            if (bestCenter == null || score > bestCount) {
                bestCount = (int) score;
                bestCenter = center;
            }
        }

        // Fallback: if no cluster met minPoints, pick closest point to origin (or return null)
        if (bestCenter == null) {
            return poseList.get(0);
        }

        return bestCenter;
    }
}