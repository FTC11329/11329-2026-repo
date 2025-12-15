package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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
import org.firstinspires.ftc.teamcode.util.shooterInterpolation.NewShooterTestValues;
import org.firstinspires.ftc.teamcode.util.shooterInterpolation.ShooterState;
import org.firstinspires.ftc.teamcode.util.shooterInterpolation.ShooterTestValues;
import org.firstinspires.ftc.teamcode.util.shooterInterpolation.ShooterValuesParent;
//todo import java.awt.Shape to make the inShootingZone() better 

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Function;

public class Robot {
    // Todo make private exept follower
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


    ShooterTestValues shooterTestValues;
    NewShooterTestValues newShooterTestValues;
    public ElapsedTime shooterTimer;
    TelemetryManager panelsTelemetry;

    double startTime;

    Pose lastCamPose = new Pose(0,0,0);
    // Offset pose to aim for
    public Pose offsetPose = new Pose(0,0,0);
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
    Pose goal = new Pose(0,0,0);


    Telemetry telemetry;
    public Robot(Telemetry telemetry, HardwareMap hardwareMap, RobotSide robotSide, int startTurretTicks) {
        this.telemetry = telemetry;
        this.robotSide = robotSide;
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        stilts = new Stilts(hardwareMap);
        intake = new Intake(hardwareMap);
        vision = new Vision(hardwareMap, robotSide);
        indexer = new Indexer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap, startTurretTicks, robotSide);
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        time = new ElapsedTime();

        follower.setStartingPose(new Pose(0,0,0));

        shooterTimer = new ElapsedTime();
        shooterTestValues = new ShooterTestValues();
        newShooterTestValues = new NewShooterTestValues();
        follower.resetIMU();
        shooter.resetController();
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


    // Checks if the robot center is within the
    public boolean inShootingZone() {
        // idk if this logic works
//        boolean triangle1 = inTriangle(Constants.ShootingZone.bigLeft, Constants.ShootingZone.bigCenter, Constants.ShootingZone.bigRight);
//        boolean triangle2 = lineSide(Constants.ShootingZone.smallRight, Constants.ShootingZone.smallCenter, Constants.ShootingZone.smallLeft);
//        return triangle1 || triangle2;

        // Robot radius
        double r = 6;

        Pose cur = getCurrentPose();
        double Rx = cur.getX();
        double Ry = cur.getY();

        // --- FAST REJECT: robot is far from both triangles ---
        if (Rx < -r && Rx > r - 48) return false;

        // --- FAST ACCEPT (big triangle) ---
        if (Rx > Ry && Rx > -Ry) return true;

        // --- FAST ACCEPT (small triangle) ---
        if (Rx < Ry - 48 && Rx < -Ry - 48) return true;


        // ======================================================
        // SELECT TRIANGLE
        // ======================================================

        Pose start;
        Pose end;
        int steps;


        if (Rx >= 0) {
            // ===========================
            // BIG TRIANGLE (top)
            // ===========================

            start = new Pose(0, 0);    // apex
            steps = 100;

            if (Ry >= 0) {
                end = new Pose(72, 72);
            } else {
                end = new Pose(72, -72);
            }

        } else {
            // ===========================
            // SMALL TRIANGLE (bottom)
            // ===========================

            start = new Pose(-48, 0);   // apex
            steps = 34;

            if (Ry >= 0) {
                end = new Pose(-72, 24);
            } else {
                end = new Pose(-72, -24);
            }
        }


        // ======================================================
        // SAMPLE ALONG THE TRIANGLE EDGE
        // ======================================================

        for (int i = 0; i <= steps; i++) {

            double t = i / (double) steps;

            double px = start.getX() * (1 - t) + end.getX() * t;
            double py = start.getY() * (1 - t) + end.getY() * t;

            double dx = Rx - px;
            double dy = Ry - py;

            if (dx * dx + dy * dy <= r * r)
                return true;
        }

        return false;  // No sample point was within radius r
    }
    // TURRET**************************************************************************************~

    public void turretUpdate() {
        turret.update();
    }

    // SHOOTER*************************************************************************************~
    public boolean readyToShoot() {
        return shooter.closeEnoughToTarget() && turret.closeEnoughToTarget(getCurrentPose());
    }

    // Adds a ball of color ball color to queuedBalls list
    public void qBall(BallColor qdColor) {
        queuedBalls.add(qdColor);
    }

    public void qBall(BallColor[] qdColors) {
        queuedBalls.addAll(Arrays.asList(qdColors));
    }

    public void casualShooterModeOn() {
        shooter.casualModeOn();
    }

    public void prepareShooter() {
        prepareShooter(true);
    }

    //corrects the hood, turret, and shooter rpm
    public void prepareShooter(boolean deleteMeIfYouWant) {

        // Gets current Pose
        Pose curPose = getCurrentPose();

        // Gets goal Pose
        if (robotSide == RobotSide.Blue)  {
            goal = Constants.Vision.blueGoal;
        } else {
            goal = Constants.Vision.redGoal;
        }

        ShooterValuesParent stv = shooterTestValues;

        goal = goal.plus(offsetPose); //Todo: get rid of

        Pose futrGoal = goal;

        for (int i = 0; i <= 3; i++) {
            // Est Time In Flight for ball at current pose
            double timeInFlight = stv.get(curPose.distanceFrom(futrGoal)).timeInFlight;

            // Logic for future pose
            futrGoal = goal.plusVector(follower.getVelocity(), - timeInFlight);
        }

        // Logic for heading to goal
        double deltaX = futrGoal.getX() - curPose.getX();
        double deltaY = futrGoal.getY() - curPose.getY();
        double angleToGoal = Math.toDegrees(Math.atan2(deltaY, deltaX));

        // Sets Turret angle
        turret.setTargetDeg(angleToGoal - Math.toDegrees(curPose.getHeading()));

        // Gets shooter params based on future pose
        ShooterState futureShooterParams = stv.get(curPose.distanceFrom(futrGoal));

        // Sets shooter rpm
        shooter.adjustTargetRPM(futureShooterParams.rpm);

        // gets hood angle
        shooter.setHoodDeg(futureShooterParams.hoodAngle);

    }

    public void shootOnTheFly (boolean oldValues) {
        Pose robotPose = getCurrentPose();

        if (robotSide == RobotSide.Blue)  {
            goal = Constants.Vision.blueGoal;
        } else {
            goal = Constants.Vision.redGoal;
        }
        goal = goal.plus(offsetPose);

        ShooterValuesParent shooterTestValues;
        if (oldValues) {shooterTestValues = this.shooterTestValues;}
        else {shooterTestValues = newShooterTestValues;}

        Vector robotVelocity = follower.getVelocity();
        Vector robotAcceleration = follower.getAcceleration();

        double goalPositionIterations = 3; // increase for accuracy decrease for efficiency
        double accelerationCompensationFactor = 0; //tune to account for change in velocity as ball is shot

        double shotTime = shooterTestValues.get(robotPose.distanceFrom(goal)).timeInFlight;

        Pose correctedGoal = new Pose();
        for (int i = 0; i < goalPositionIterations; i++) { //todo: ensure that velocity and acceleration units match
            double virtualGoalX = goal.getX()
                    - shotTime * (robotVelocity.getXComponent()
                    + robotAcceleration.getXComponent() * accelerationCompensationFactor);
            double virtualGoalY = goal.getY()
                    - shotTime * (robotVelocity.getYComponent()
                    + robotAcceleration.getYComponent() * accelerationCompensationFactor);

            correctedGoal = new Pose(virtualGoalX, virtualGoalY);

            double newShotTime = shooterTestValues.get(robotPose.distanceFrom(correctedGoal)).timeInFlight;

            if (Math.abs(newShotTime - shotTime) <= 0.010) {
                break;
            }

            shotTime = newShotTime;
        }

        ShooterState futureShooterParams = shooterTestValues.get(getCurrentPose().distanceFrom(correctedGoal));

        // Logic for heading to goal
        double deltaX = correctedGoal.getX() - robotPose.getX();
        double deltaY = correctedGoal.getY() - robotPose.getY();
        double angleToGoal = Math.toDegrees(Math.atan2(deltaY, deltaX));

        // Sets Turret angle
        turret.setTargetDeg(angleToGoal - Math.toDegrees(angleToGoal));

        // Sets shooter rpm
        shooter.adjustTargetRPM(futureShooterParams.rpm);

        // gets hood angle
        shooter.setHoodDeg(futureShooterParams.hoodAngle);
        telemetry.addData("HoodAngle", futureShooterParams.hoodAngle);
    }

    public void shootAny() {
        indexer.spinIndexer(true);
        indexer.transfer(readyToShoot());
    }

    public boolean shootArtifact(BallColor ballColor) {
        switch (oneBallCase) {
            case 0:
                indexer.transfer(false);
                if (indexer.spinUntil(ballColor)) {
                    oneBallCase = 1;
                }
                return false;
            case 1:
                indexer.transfer(false);
                if (readyToShoot()) {
                    startTime = time.milliseconds();
                    oneBallCase = 2;
                }
                return false;
            case 2:
                indexer.transfer(true);
                indexer.spinIndexer(true);
                if (time.milliseconds() - startTime < 500) {
                    oneBallCase = 0;
                    indexer.transfer(false);
                    indexer.spinIndexer(false);
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
            indexer.transfer(false);
            return;
        }

        

        if (queuedBalls.isEmpty()) {
            shootAny();
            return;
        }
        BallColor soonToBeShotBall = queuedBalls.get(0);

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
        // if (pictureTime + 500 < time.milliseconds()) {
            // pictureTime = time.milliseconds();
            // autoSetCurrentPose();
        // }

        shooter.update();
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
        lastCamPose = vision.getRobotPose();
        if (lastCamPose != null){
           follower.setPose(lastCamPose);
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
        indexer.update(getCurrentPose().distanceFrom(goal));
    }

    public void stopIndexer() {
        indexer.spinIndexer(false);
        indexer.transfer(false);
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
        turretUpdate();
        teleopUpdate();
        follower.update();
        telemetry.addData("distance", getCurrentPose().distanceFrom(goal));
        if (debug) {
            Drawing.drawDebug(follower);
            telemetry.addLine("=== VISION ===");
            telemetry.addData("Motif", motif);

            telemetry.addLine("=== SHOOTER ===");
            telemetry.addData("Shooter RPM", shooter.getRPM());
            telemetry.addData("Tar Shooter VEL", shooter.shooterPID.getTargetPosition());
            telemetry.addData("Shooter VEL", shooter.getVelocity());
            telemetry.addData("Shooter ERR", shooter.shooterPID.getError());
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
            telemetry.addData("last cam pose", lastCamPose);
            telemetry.addData("offset", offsetPose);
            telemetry.addData("distance", getCurrentPose().distanceFrom(goal));
            telemetry.addData("in shooting zone", inShootingZone());

            telemetry.addLine("=== QUEUE ===");
            if (queuedBalls.isEmpty()) {
                telemetry.addLine("Nothing in queue");
            }
            for (BallColor ball : queuedBalls) {
                telemetry.addData("qball", ball);
            }


//            telemetry.addData("in shooting zone", inShootingZone());
//            telemetry.addData("hood", shooter.getHoodPosDeg());
//            telemetry.addData("rpm", shooter.getRPM());

             panelsTelemetry.addData("turret target", turret.turretPID.getTargetPosition());
             panelsTelemetry.addData("turret actual", turret.getAngle());
             panelsTelemetry.addData("shooter target", shooter.shooterPID.getTargetPosition());
             panelsTelemetry.addData("shooter actual", shooter.getVelocity());
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
