package org.firstinspires.ftc.teamcode.util.ShootOnTheFly;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.math.Vector;
import org.firstinspires.ftc.teamcode.util.RobotSide;

public class ShotContext {
    public Pose robotPose;
    public Pose goalPose;

    public Vector velocity;
    public Vector acceleration;

    public RobotSide side;

    public double rpmRatio;
}

