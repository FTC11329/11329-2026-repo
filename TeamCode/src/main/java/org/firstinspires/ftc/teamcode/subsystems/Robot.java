package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.math.Vector;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.RobotSide;

import java.util.ArrayList;
import java.util.Arrays;

public class Robot {
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
    TelemetryManager panelsTelemetry;

    double startTime;

    //This is the balls that the shooter prepares to shoot
    public ArrayList<BallColor> queuedBalls = new ArrayList<>();
    //This is an array of the 3 special colors of the games MOTIF
    public BallColor[] motif = null;
    //This is SPECIFICALLY for auto, it is the balls in the ramp
    public ArrayList<BallColor> ramp = new ArrayList<BallColor>();
    //This is a varibale used in the ShootArtifact function to keep track of the shooting Phase
    int oneBallCase = 0;
    //This is a PUBLIC variable: [turret anlge, hood angle, shooter RPM]
    double[] shootingParams;
    boolean doAutoIntake = false;

    Telemetry telemetry;
    public Robot(Telemetry telemetry, HardwareMap hardwareMap, RobotSide robotSide) {
        this.telemetry = telemetry;
        this.robotSide = robotSide;
        panelsTelemetry = Panels.getTelemetry();
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
    public void getMotif() {
        if (motif == null){
            motif = vision.getMotif();
        }
    }

    public boolean lineSide(Pose a, Pose b, Pose r) {
        double s = (b.getX() - a.getX()) * (r.getY() - a.getY()) - (b.getY() - a.getY()) * (r.getY() - a.getY());
        s = s / a.distanceFrom(b);
        telemetry.addData("distance", s);
        return s + 5 >= 0;
    }

    public boolean inTriangle(Pose a, Pose b, Pose c) {
        boolean line1 = lineSide(b, a, getCurrentPose());
        boolean line2 = lineSide(c, b, getCurrentPose());

        return line1 && line2;
    }

    public boolean inShootingZone() {
        boolean triangle1 = inTriangle(Constants.ShootingZone.bigLeft, Constants.ShootingZone.bigCenter, Constants.ShootingZone.bigRight);
        boolean triangle2 = lineSide(Constants.ShootingZone.smallRight, Constants.ShootingZone.smallCenter, Constants.ShootingZone.smallLeft);

        return triangle1 || triangle2;
    }
    // TURRET**************************************************************************************~

    public void updateTurret() {
        Pose curPose = getCurrentPose();
        double dx = 72 - curPose.getX();
        double dy = 72 - curPose.getY();
        double angle = Math.toDegrees(Math.atan2(dy, dx));
        turret.setTargetDeg(angle - Math.toDegrees(curPose.getHeading()));
    }

    // SHOOTER*************************************************************************************~
    // Adds a ball of color ball color to queuedBalls list
    public void qBall(BallColor qdColor) {
        queuedBalls.add(qdColor);
    }

    public void qBall(BallColor[] qdColors) {
        queuedBalls.addAll(Arrays.asList(qdColors));
    }

    //corrects the hood, turret, and shooter rpm
    public void prepareShooter() {
        shooter.setPower(0.9);
        Vector vel = follower.getVelocity();
        double[] params = vision.getShooterParams(getCurrentPose(), new Pose (vel.getXComponent(), vel.getYComponent()));
        shooter.setHood(params[0]);
    }

    public boolean shootArtifact(BallColor ballColor) {
        switch (oneBallCase) {
            case 0:
                if (indexer.spinUntil(ballColor)) {
                    oneBallCase = 1;
                }
                return false;
            case 1:
                if (shooter.closeEnoughToTarget()) {
                    startTime = time.milliseconds();
                    oneBallCase = 2;
                }
                return false;
            case 2:
                indexer.transfer(true);
                if (time.milliseconds() - startTime < 500) {
                    oneBallCase = 0;
                    indexer.transfer(false);
                    return true;
                }
                return false;
        }
        return false;
    }

    // Loops through and removes balls from quedballs after firing them
    public boolean shootQueueInMotif() {
        if (!inShootingZone()) {
            return false;
        }
        if (queuedBalls.isEmpty()) {
            return true;
        }
        BallColor current = nextArtifactInMotif();
        if (shootArtifact(current)) {
            if (queuedBalls.contains(current)) {
                queuedBalls.remove(current);
            } else {
                queuedBalls.remove(0);
            }
            ramp.add(current);
        }
        return false;
    }

    // shoots balls in queue or any ball
    public void shootQueue(boolean override) {
        if (!inShootingZone() && !override){
            return;
        }
        BallColor soonToBeShotBall;
        if (queuedBalls.isEmpty()) {
            soonToBeShotBall = BallColor.Any;
        } else {
            soonToBeShotBall = queuedBalls.get(0);
        }
        if (shootArtifact(soonToBeShotBall)) {
            if (!queuedBalls.isEmpty()) {
                queuedBalls.remove(0);
            }
        }
    }

    public void setShooterTargetRPM(double set) {
        shooter.setTargetRPM(set);
    }

    double pictureTime = 0;
    public void shooterUpdate() {
        // Takes Picture every ___ ms
        if (pictureTime + 500 < time.milliseconds()) {
            pictureTime = time.milliseconds();
            autoSetCurrentPose();
        }

//        shooter.update();
    }

    // Uses curent ramp state and current motif to get the next color of ball we shoot
    public BallColor nextArtifactInMotif(){
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
//            follower.setPose(pos);
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

    public void stopIndexer() {
        indexer.index(false);
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
        /* todo remove after 2nd comp
        if (doAutoIntake) {
            spinIntake();
            spinIndexer();
            hasBalls = new ArrayList<>(indexer.scanIndexer(hasBalls));
            doAutoIntake = hasBalls.size() < 3;
        }
         */
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
        updateTurret();
        teleopUpdate();
        turret.update();
        follower.update();
        if (debug) {
            Drawing.drawDebug(follower);

            telemetry.addLine("=== VISION ===");
            telemetry.addData("Motif", motif);

            telemetry.addLine("=== SHOOTER ===");
            telemetry.addData("Shooter RPM", shooter.getRPM());
            telemetry.addData("Hood Angle", shooter.getHoodPosDeg());

            telemetry.addLine("=== Turret ===");
            telemetry.addData("Turret Degrees", turret.getAngle());
            telemetry.addData("Turret Ticks  ", turret.getTicks());
            telemetry.addData("Turret Tar Deg", turret.turretPID.getTargetPosition());
            telemetry.addData("Turret power", turret.turretPID.run());

            telemetry.addLine("=== COLOR ===");
            telemetry.addData("indexer r", indexer.getColorRGBA().red);
            telemetry.addData("indexer g", indexer.getColorRGBA().green);
            telemetry.addData("indexer b", indexer.getColorRGBA().blue);
            telemetry.addData("indexer a", indexer.getColorRGBA().alpha);
            telemetry.addData("indexer col", indexer.getColor());
            telemetry.addData("indexer dis", indexer.getDistance());

            telemetry.addLine("=== POSITION ===");
            telemetry.addData("guess pose", getCurrentPose());
            telemetry.addData("last cam pose", getCurrentPose());
            telemetry.addData("PP Pose", getCurrentPose());

            telemetry.addLine("=== QUEUE ===");
            if (queuedBalls.isEmpty()) {
                telemetry.addLine("Nothing in queue");
            }
            for (BallColor ball : queuedBalls) {
                telemetry.addData("qball", ball);
            }
            panelsTelemetry.debug("wave: $wave");
            panelsTelemetry.debug("wave2: $wave2");
            panelsTelemetry.graph("wave", shooter.getRPM());
            panelsTelemetry.graph("wave2", shooter.shooterPID.getTargetPosition());
            panelsTelemetry.update(telemetry);

        }
    }

    public void stopAllSubsystems() {
        oneBallCase = 0;
        queuedBalls = new ArrayList<>();
        indexer.transfer(false);
        shooter.stopShooter();
        shooter.setHoodDeg(0);
        stopIndexer();
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
