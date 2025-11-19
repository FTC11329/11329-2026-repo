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
import java.util.Arrays;

public class Robot {
    ArrayList<BallColor> queuedBalls = new ArrayList<>();
    public BallColor[] motif = new BallColor[3];
    public ArrayList<BallColor> ramp = new ArrayList<BallColor>();
    int oneBallCase = 0;
    int burst = 3;
    double[] shootingParams;


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
    public boolean singleBallShoot() {
        switch (oneBallCase) {
            case 0:
                shootingParams = vision.getRPM(3500, Vision.InitialCondition.RPM, new Pose(follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent()));
                if (indexer.spinTill(nextArtifact())) {
                    oneBallCase = 1;
                }
                shooter.spinUp(shootingParams[2]);
                return false;
            case 1:
                shootingParams = vision.getRPM(3500, Vision.InitialCondition.RPM, new Pose(follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent()));
                turret.turnTo(shootingParams[0]);
                shooter.setHoodDeg(shootingParams[1]);
                if (shooter.spinUp(shootingParams[2])) {
                    startTime = time.milliseconds();
                    oneBallCase = 2;
                }
                return false;
            case 2:
                indexer.ballToShooter();
                if (time.milliseconds() - startTime < 300) {
                    oneBallCase = 0;
                    return true;
                }
                return false;
        }
        return false;
    }

    // Loops through and removes balls from queuedBalls after firing them
    public boolean shootQueue() {
        BallColor current = nextArtifact();
        if (queuedBalls.isEmpty()) {
            return true;
        }
        if (singleBallShoot()) {
            queuedBalls.remove(current);
            ramp.add(current);
        }
        return false;
    }

    public BallColor nextArtifact(){
        if (motif != null){
            if (queuedBalls.contains(motif[(ramp.size() - 1) % 3])){
                return motif[(ramp.size() - 1) % 3];
            }
        }
        return queuedBalls.get(0);
    }

    public void getMotif(){
        if (motif != null){
            return;
        }
        motif = vision.getMotif();
    }

    // Adds a ball of color ball color to queuedBalls list
    public void QBall(BallColor qdColor) {
        queuedBalls.add(qdColor);
    }
    public void QBall(BallColor[] qdColors) {
        queuedBalls.addAll(Arrays.asList(qdColors));
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
