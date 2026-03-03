package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.math.Vector;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;

public class MeanBallPoses {
    List<Vision.DetectedBall> detectedBalls;
    MeanBallPoses(List<Vision.DetectedBall> detectedBalls) {
        this.detectedBalls = detectedBalls;
    }

    Pose getIntakeTarget() {
        List<Vector> points = new ArrayList<>();
        for (Vision.DetectedBall ball : detectedBalls) {
            points.add(new Vector(ball.ballPose.getX(), ball.ballPose.getY()));
        }

        Vector mean = new Vector(0, 0);
        for (Vector p : points) {
            mean = mean.plus(p);
        }
        mean = mean.divide(points.size());

        return new Pose(mean.getXComponent(), mean.getYComponent());
    }
}
