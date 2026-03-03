package org.firstinspires.ftc.teamcode.util;


import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.math.MathFunctions;

import java.util.ArrayList;
import java.util.Dictionary;
import java.util.List;
import java.util.Random;

public class Clustering {

    public int maxIterations = 20;

    public Pose findLargestCluster(List<Pose> poseList, int k){
        k = Math.min(k, poseList.size());

        Random random = new Random();
        Pose[] clusters = new Pose[k];

        for (int i = 0; i < k; i++) {
            clusters[i] = poseList.get(random.nextInt(poseList.size()));
        }

        List<List<Pose>> clusterAssignments = new ArrayList<>(k);

        for (int iteration = 0; iteration < maxIterations; iteration++) {
            clusterAssignments.clear();
            for (int i = 0; i < k; i++) {
                clusterAssignments.add(new ArrayList<>());
            }

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

            for (int j = 0; j < k; j++) {
                List<Pose> cluster = clusterAssignments.get(j);

                if (cluster.isEmpty()) {
                    newClusters[j] = clusters[j];
                    continue;
                }

                Pose sum = new Pose();
                for (Pose point : cluster) {
                    sum = sum.plus(point);
                }

                newClusters[j] = sum.times(1.0 / cluster.size());
            }

            clusters = newClusters;
        }

        int largestSize = -1;
        Pose largestClusterCenter = clusters[0];
        int largestClusterIndex = 0;

        for (int j = 0; j < k; j++) {
            int size = clusterAssignments.get(j).size();
            if (size > largestSize) {
                largestSize = size;
                largestClusterCenter = clusters[j];
                largestClusterIndex = j;
            }
        }

        double meanSqaureError = 0.0f;

        for (Pose point: clusterAssignments.get(largestClusterIndex)){
            meanSqaureError += Math.pow(largestClusterCenter.distanceFrom(point),2);
        }
        meanSqaureError = meanSqaureError / clusterAssignments.get(largestClusterIndex).size();

        return largestClusterCenter;
    }
}
