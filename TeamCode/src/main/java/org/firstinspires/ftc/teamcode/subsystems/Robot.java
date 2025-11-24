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

    public ElapsedTime shooterTimer;

    double startTime;

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

        shooterTimer = new ElapsedTime();
    }


    //Shoots one ball of BallColor and returns true when done
    public boolean shootArtifact(boolean stopWhenFinished) {
        switch (oneBallCase) {
            case 0:
                shootingParams = vision.getRPM(getCurrentPose(), 3500, Vision.InitialCondition.RPM, new Pose(follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent()));
                if (indexer.spinUntil(nextArtifact())) {
                    oneBallCase = 1;
                    shooter.spinUp(shootingParams[2]);
                    turret.turnTo(shootingParams[0]);
                }
                return false;
            case 1:
                shootingParams = vision.getRPM(getCurrentPose(),3500, Vision.InitialCondition.RPM, new Pose(follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent()));
                shooter.setHoodDeg(shootingParams[1]);
                turret.updateTurret(shootingParams[0]);
                if (shooter.updateShooter(shootingParams[2])) {
                    startTime = time.milliseconds();
                    oneBallCase = 2;
                }
                return false;
            case 2:
                indexer.startTransfer();
                if (time.milliseconds() - startTime < 300) {
                    oneBallCase = 0;
                    indexer.stopTransfer();
                    if (stopWhenFinished){
                        indexer.stopIndexer();
                    }
                    return true;
                }
                return false;
        }
        return false;
    }

    //Shoots one ball of BallColor and returns true when done
    public void passiveShoot(double RPM, boolean intake) {
        indexer.setIndexerPower(Constants.Indexer.spindexPower);
        indexer.startTransfer();
        shooter.spinUp(RPM);
        if (intake){
            startIntake();
        }

        shooter.updateShooter();
    }

    public void stopAllSubsystems(){
        indexer.setIndexerPower(0);
        indexer.stopTransfer();
        shooter.setPower(0);
        oneBallCase = 0;
        stopIntake();
    }

    // Loops through and removes balls from queuedBalls after firing them
    public boolean shootQueue() {
        if (queuedBalls.isEmpty()) {
            indexer.setIndexerPower(0);
            return true;
        }
        BallColor current = nextArtifact();
        if (shootArtifact(false)) {
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
        return queuedBalls.isEmpty() ? BallColor.Any : queuedBalls.get(0);
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
    public boolean intakeAndIndex() {
        intake.setIntakePower(Constants.Intake.intakePower);
        return indexer.spinUntil(BallColor.Any);
    }

    public void startIntake(){
        intake.setIntakePower(Constants.Intake.intakePower);
    }

    public void stopIntake(){
        intake.setIntakePower(0);
    }

    public Pose getCurrentPose() {
        Pose pos = vision.getRobotPose();
        if (pos != null){
            follower.setPose(pos);
        }
        return follower.getPose();
    }

    double pictureTime = 0;
    public void update() {
        if (pictureTime + 500 < time.milliseconds()) {
            pictureTime = time.milliseconds();
            getCurrentPose();
        }

        follower.update();
        telemetry.update();
        Drawing.drawDebug(follower);
    }

}
