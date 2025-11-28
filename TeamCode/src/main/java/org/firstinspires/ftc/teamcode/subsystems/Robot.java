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
    boolean doAutoIntake = false;



    // Todo make private
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
        turret = new Turret(hardwareMap);
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        time = new ElapsedTime();
        follower.setStartingPose(new Pose(0,0,0));

        shooterTimer = new ElapsedTime();
    }
    // VISION**************************************************************************************~
    public void getMotif(){
        if (motif != null){
            return;
        }
        motif = vision.getMotif();
    }

    // SHOOTER*************************************************************************************~
    // Adds a ball of color ball color to queuedBalls list
    public void qBall(BallColor qdColor) {
        queuedBalls.add(qdColor);
    }

    public void qBall(BallColor[] qdColors) {
        queuedBalls.addAll(Arrays.asList(qdColors));
    }

    //Shoots one ball of BallColor and returns true when done
    public boolean shootArtifact() {
        switch (oneBallCase) {
            case 0:
                shootingParams = vision.getRPM(getCurrentPose(), 3500, Vision.InitialCondition.RPM, new Pose(follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent()));
                if (indexer.spinUntil(nextArtifact())) {
                    oneBallCase = 1;
                    shooter.resetShooter();
                    turret.turnTo(shootingParams[0]);
                }
                return false;
            case 1:
                shootingParams = vision.getRPM(getCurrentPose(),3500, Vision.InitialCondition.RPM, new Pose(follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent()));
                shooter.setHoodDeg(shootingParams[1]);
                turret.updateTurret(shootingParams[0]);
                if (shooter.closeEnough()) {
                    shootingParams = vision.getRPM(getCurrentPose(),shooter.getRPM(), Vision.InitialCondition.RPM, new Pose(follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent()));
                    shooter.setHoodDeg(shootingParams[1]);
                    startTime = time.milliseconds();
                    oneBallCase = 2;
                }
                return false;
            case 2:
                indexer.transfer(true);
                if (time.milliseconds() - startTime < 300) {
                    oneBallCase = 0;
                    indexer.transfer(false);
                    if (queuedBalls.size() <= 1){
                        indexer.stopIndexer();
                    }
                    return true;
                }
                return false;
        }
        return false;
    }

    // Loops through and removes balls from queuedBalls after firing them
    public boolean shootQueue() {
        if (queuedBalls.isEmpty()) {
            return true;
        }
        BallColor current = nextArtifact();
        if (shootArtifact()) {
            queuedBalls.remove(current);
            ramp.add(current);
        }
        return false;
    }

    double pictureTime = 0;
    public void shooterUpdate() {
        // Takes Picture every ___ ms
        if (pictureTime + 500 < time.milliseconds()) {
            pictureTime = time.milliseconds();
            autoSetCurrentPose();
        }
    }

    // Uses curent ramp state and current motif to get the next color of ball we shoot
    public BallColor nextArtifact(){
        if (motif != null){
            if (queuedBalls.contains(motif[(ramp.size() - 1) % 3])){
                return motif[(ramp.size() - 1) % 3];
            }
        }
        return queuedBalls.isEmpty() ? BallColor.Any : queuedBalls.get(0);
    }


    public void autoSetCurrentPose() {
        Pose pos = vision.getRobotPose();
        if (pos != null){
            follower.setPose(pos);
        }
    }
    public Pose getCurrentPose() {
        return follower.getPose();
    }


    // INTAKE SYSTEM*******************************************************************************~
    public void spinIntake() {
        intake.setIntakePower(Constants.Intake.intakePower);
    }
    public void intakeUpdate() {

    }

    // SPINDEXER***********************************************************************************~
    public void spinIndexer() {
        indexer.setIndexerPower(Constants.Indexer.spindexPower);
    }

    public void spindexerUpdate() {

    }

    // TELE-OP*************************************************************************************~
    public void intakeManual() {
        spinIntake();
        spinIndexer();
    }

    public void autoIntake3() {
        doAutoIntake = true;
    }

    public void spitIntake() {
        intake.setIntakePower(Constants.Intake.spitPower);
    }

    public void teleopUpdate() {
        if (doAutoIntake) {
            spitIntake();
            spinIndexer();
            queuedBalls = new ArrayList<>(indexer.scanIndexer(queuedBalls));
            doAutoIntake = queuedBalls.size() < 3;
        }
    }

    public void stopIntake() {
        intake.setIntakePower(0);

    }


    // SYSTEM**************************************************************************************~
    public void update() {
        update(false);
    }
    public void update(boolean debug) {
        shooterUpdate();
        intakeUpdate();
        spindexerUpdate();
        teleopUpdate();
        shootQueue();
        follower.update();
        telemetry.update();
        if (debug) {
            Drawing.drawDebug(follower);
        }
    }

    public void stopAllSubsystems() {
        indexer.setIndexerPower(0);
        indexer.transfer(false);
        shooter.setPower(0);
        oneBallCase = 0;
        stopIntake();
    }

    // TESTING*************************************************************************************~
    //Shoots one ball
    public void passiveShoot(double RPM, boolean intake) {
        indexer.setIndexerPower(Constants.Indexer.spindexPower);
        indexer.transfer(true);
        shooter.setTargetRPM(RPM);
        if (intake) {
            intakeManual();
        }
    }
}
