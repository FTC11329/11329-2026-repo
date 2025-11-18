package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.RobotSide;

import java.util.ArrayList;

public class Robot {
    ArrayList<BallColor> queuedBalls = new ArrayList<>();
    int oneBallCase = 0;
    int burst = 3;


    public Stilts stilts;
    public Intake intake;
    public Turret turret;
    public Vision vision;
    public Indexer indexer;
    public Shooter shooter;
    public Follower follower;
    public Drivetrain drivetrain;
    public ElapsedTime time;
    public RobotSide robotSide;

    Telemetry telemetry;
    public Robot(Telemetry telemetry, HardwareMap hardwareMap, RobotSide robotSide) {
        this.telemetry = telemetry;
        this.robotSide = robotSide;
        stilts = new Stilts(hardwareMap);
        intake = new Intake(hardwareMap);
        vision = new Vision(hardwareMap, robotSide);
        indexer = new Indexer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        time = new ElapsedTime();
        follower.setStartingPose(new Pose(0,0,0));

    }

    double startTime;

    //Shoots one ball of BallColor and returns true when done
    public boolean singleBallShoot(BallColor colorOfBall) {
        switch (oneBallCase) {
            case 0:
                if (indexer.spinTill(colorOfBall)) {
                    oneBallCase = 1;
                }
                return false;
            case 1:
                turret.turnTo(Constants.PlacholdereDouble);
                shooter.setHoodDeg(Constants.PlacholdereDouble);
                if (shooter.spinUp()) {
                    startTime = time.milliseconds();
                    oneBallCase = 2;
                }
                return false;
            case 2:
                indexer.ballToShooter();
                if (time.milliseconds() - startTime < 300) {
                    shooter.lightSpin();
                    oneBallCase = 0;
                    return true;
                }
                return false;
        }
        return false;
    }

    // Loops through and removes balls from queuedBalls after firing them
    public void shootQueue() {
        if (queuedBalls.isEmpty()) {
            return;
        }
        if (singleBallShoot(queuedBalls.get(0))) {
            queuedBalls.remove(0);
        }
    }

    // Adds a ball of color ball color to queuedBalls list
    public void QBall(BallColor qdColor) {
        queuedBalls.add(qdColor);
    }

    //function to intake and spin until sees any ball
    public boolean Intakenspindex() {
        intake.setintakePower(Constants.Intake.intakePower);
        return indexer.spinTill(BallColor.Any);
    }

    public Pose getCurrentPose() {
        return follower.getPose();
    }

    double pictureTime = 0;
    public void update() {
        if (pictureTime + 500 < time.milliseconds()) {
            pictureTime = time.milliseconds();
            Pose visionPose = vision.getRobotPose();
            if (visionPose.distanceFrom(new Pose(0,0,0)) > 0.001) {
                follower.setPose(visionPose);
            }
        }
        shootQueue();
        follower.update();
        Drawing.drawDebug(follower);
    }

}
