package org.firstinspires.ftc.teamcode.modularAutos.runnableWrappers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.modularAutos.CommonPoses;
import org.firstinspires.ftc.teamcode.modularAutos.FromStartClosePosition;
import org.firstinspires.ftc.teamcode.modularAutos.PathPlanner;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.RobotSide;

import java.util.List;

@Autonomous
public class ExampleRunnableWrapper extends OpMode {
    Pose startPose;
    RobotSide robotSide;
    Robot robot;
    private List<PathPlanner> steps;
    private int currentStep = 0;

    @Override
    public void init() {
        // todo Set These Before Creating
        robotSide = null;
        robot = new Robot(telemetry, hardwareMap, robotSide, 0,0,
                new BallColor[]{
                        BallColor.Green,
                        BallColor.Purple,
                        BallColor.Purple
                });
        // todo Set These Before Creating
        startPose = null;

        steps.add(new FromStartClosePosition.ShootAndGoToMidShootPos(robot, lastPose()));
        steps.add(new FromStartClosePosition.ShootAndGoToMidShootPos(robot, lastPose()));

        robot.follower.setPose(startPose);
    }

    @Override
    public void loop() {

    }

    private Pose lastPose() {
        if (steps.isEmpty()) {
            return startPose;
        } else {
            return steps.get(steps.size() - 1).getEndPoseEst();
        }
    }
}
