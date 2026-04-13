package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class Clustering {

    public int maxIterations = 20;
    public double convergenceEpsilon = 1e-3;

    // Fixed seed for deterministic behavior (prevents jitter)
    private final Random random = new Random(42);

    public Pose findLargestCluster(List<Pose> poseList, int k) {

        // Handle empty input
        if (poseList == null || poseList.isEmpty()) {
            return null;
        }

        // Clamp k to valid range
        k = Math.max(1, Math.min(k, poseList.size()));

        Pose[] clusters = new Pose[k];

        // Initialize cluster centers randomly
        for (int i = 0; i < k; i++) {
            clusters[i] = poseList.get(random.nextInt(poseList.size()));
        }

        List<List<Pose>> clusterAssignments = new ArrayList<>();

        for (int iteration = 0; iteration < maxIterations; iteration++) {

            clusterAssignments.clear();
            for (int i = 0; i < k; i++) {
                clusterAssignments.add(new ArrayList<>());
            }

            // Assign points to nearest cluster
            for (Pose point : poseList) {
                int nearestIndex = 0;
                double minDistance = Double.MAX_VALUE;

                for (int j = 0; j < k; j++) {
                    double distance = point.distanceFrom(clusters[j]);
                    if (distance < minDistance) {
                        minDistance = distance;
                        nearestIndex = j;
                    }
                }

                clusterAssignments.get(nearestIndex).add(point);
            }

            Pose[] newClusters = new Pose[k];
            boolean converged = true;

            for (int j = 0; j < k; j++) {
                List<Pose> cluster = clusterAssignments.get(j);

                if (cluster.isEmpty()) {
                    //Reinitialize empty clusters instead of freezing
                    newClusters[j] = poseList.get(random.nextInt(poseList.size()));
                    converged = false;
                    continue;
                }

                Pose sum = new Pose();
                for (Pose point : cluster) {
                    sum = sum.plus(point);
                }

                newClusters[j] = sum.times(1.0 / cluster.size());

                //Convergence check
                if (clusters[j].distanceFrom(newClusters[j]) > convergenceEpsilon) {
                    converged = false;
                }
            }

            clusters = newClusters;

            // Stop early if converged
            if (converged) {
                break;
            }
        }

        //Select best cluster using density (size / MSE)
        double bestScore = -1;
        Pose bestCenter = clusters[0];

        for (int j = 0; j < k; j++) {
            List<Pose> cluster = clusterAssignments.get(j);

            if (cluster.isEmpty()) continue;

            Pose center = clusters[j];

            double mse = 0.0;
            for (Pose point : cluster) {
                double d = center.distanceFrom(point);
                mse += d * d;
            }
            mse /= cluster.size();

            // Avoid divide-by-zero
            double score = cluster.size() / (mse + 1e-6);

            if (score > bestScore) {
                bestScore = score;
                bestCenter = center;
            }
        }

        return bestCenter;
    }
}
