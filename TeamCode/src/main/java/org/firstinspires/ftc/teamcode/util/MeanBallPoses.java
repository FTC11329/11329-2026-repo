package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.math.Vector;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class MeanBallPoses {
    List<Vision.DetectedBall> detectedBalls;

    public MeanBallPoses() {
    }

    static public Pose getIntakeTarget(List<Vision.DetectedBall> detectedBalls) {
        List<Pose> ballPoses = new ArrayList<>();
        for (Vision.DetectedBall ball : detectedBalls) {
            if (Math.abs(ball.ballPose.getX()) < 75 && Math.abs(ball.ballPose.getY()) < 75) {
                ballPoses.add(ball.ballPose);
            }
        }

        double xTot = 0;
        double yTot = 0;
        for (Pose ballPose : ballPoses) {
            xTot += ballPose.getX();
            yTot += ballPose.getY();
        }

        Pose allMean = new Pose(xTot / ballPoses.size(), yTot / ballPoses.size());

        ballPoses.sort(Comparator.comparingDouble(p -> p.distanceFrom(allMean)));

        List<Pose> ball3 = ballPoses.subList(0, Math.min(3, ballPoses.size()));

        xTot = 0;
        yTot = 0;
        for (Pose ballPose : ball3) {
            xTot += ballPose.getX();
            yTot += ballPose.getY();
        }

        return new Pose(xTot / ball3.size(), yTot / ball3.size());


    }
}