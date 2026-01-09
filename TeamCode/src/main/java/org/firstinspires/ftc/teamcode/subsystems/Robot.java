package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.modularAutos.CommonPoses;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.FieldShapes;
import org.firstinspires.ftc.teamcode.util.IndexerLogic;
import org.firstinspires.ftc.teamcode.util.RobotSide;
import org.firstinspires.ftc.teamcode.util.ShapeDetection;
import org.firstinspires.ftc.teamcode.util.SuperDuperPID;
import org.firstinspires.ftc.teamcode.util.shooterInterpolation.ShooterTestValuesV2;
import org.firstinspires.ftc.teamcode.util.shooterInterpolation.ShooterState;
import org.firstinspires.ftc.teamcode.util.shooterInterpolation.ShooterTestValuesV1;
import org.firstinspires.ftc.teamcode.util.shooterInterpolation.ShooterValuesParent;
//todo import java.awt.Shape to make the inShootingZone() better 

import java.util.ArrayList;

public class Robot {
    // Todo make private exept follower
    public Stilts stilts;
    public Intake intake;
    public Turret turret;
    public Vision vision;
    public IndexerLogic indexer;
    public Shooter shooter;
    public Follower follower;
    public Drivetrain drivetrain;
    public ElapsedTime time;
    public RobotSide robotSide;


    ShooterTestValuesV1 shooterTestValuesV1;
    ShooterTestValuesV2 shooterTestValuesV2;
    public ElapsedTime shooterTimer;
    TelemetryManager panelsTelemetry;

    double startTime;

    boolean intakeToggle = false;
    boolean spitIntake = false;
    boolean isIntaking = false;

    Pose lastCamPose = new Pose(0,0,0);
    // Offset pose to aim for
    public Pose offsetPose = new Pose(0,0,0);
    //This is an array of the 3 special colors of the games MOTIF
    public BallColor[] motif = null;
    //This is SPECIFICALLY for auto, it is the balls in the ramp
    public ArrayList<BallColor> ramp = new ArrayList<>();
    //This is a varibale used in the ShootArtifact function to keep track of the shooting Phase
    int oneBallCase = 0;
    //This is a PUBLIC variable: [turret anlge, hood angle, shooter RPM]
    double[] shootingParams;
    boolean doAutoIntake = false;
    int thingies = 0;
    Pose goal = new Pose(0,0,0);
    int i = 0;
    long lastTime;


    Telemetry telemetry;
    public Robot(Telemetry telemetry, HardwareMap hardwareMap, RobotSide robotSide, int startTurretTicks, int startIndexerTicks) {
        this(telemetry, hardwareMap, robotSide, startTurretTicks, startIndexerTicks, new BallColor[]{BallColor.None, BallColor.None, BallColor.None});
    }
    public Robot(Telemetry telemetry, HardwareMap hardwareMap, RobotSide robotSide, int startTurretTicks, int startIndexerTicks, BallColor[] ballsInIndexer) {
        Drawing.init();
        CommonPoses.init(robotSide);
        this.telemetry = telemetry;
        this.robotSide = robotSide;
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        stilts = new Stilts(hardwareMap);
        intake = new Intake(hardwareMap);
        vision = new Vision(hardwareMap, robotSide);
        indexer = new IndexerLogic(hardwareMap, ballsInIndexer, startIndexerTicks);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap, startTurretTicks, robotSide);
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        time = new ElapsedTime();

        follower.setStartingPose(new Pose(0,0,0));

        shooterTimer = new ElapsedTime();
        shooterTestValuesV1 = new ShooterTestValuesV1();
        shooterTestValuesV2 = new ShooterTestValuesV2();
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

    public boolean inShootingZone() {
        Pose curPose = follower.getPose();
        return ShapeDetection.doesRobotIntersect(FieldShapes.closeTriangle, curPose) || ShapeDetection.doesRobotIntersect(FieldShapes.farTriangle, curPose);
    }

            /*
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

             */
    // TURRET**************************************************************************************~

    public void turretUpdate() {
        turret.update(angleToGoalVelocity);
    }

    // SHOOTER*************************************************************************************~
    public boolean readyToShoot() {
        return inShootingZone();
    }

    public boolean readyToShootMotors() {
        return shooter.closeEnoughToTarget() && turret.closeEnoughToTarget(getCurrentPose()) && shooter.getRPM() > 100;
    }


    // Adds a ball of color ball color to queuedBalls list
    public void qBall(BallColor qdColor) {
        indexer.addToBackQueue(qdColor);
    }

    public void casualShooterModeOn() {
        shooter.casualModeOn();
    }

    public void prepareShooter() {
        prepareShooter(0,0);
    }

    //corrects the hood, turret, and shooter rpm
    double lastTimeTurret = 0;
    double lastAngleToGoal = 0;
    double angleToGoalVelocity = 0;
    public void prepareShooter(double rpmOffset, double hoodAngleOffset) {

        // Gets current Pose
        Pose curPose = getCurrentPose();

        // Gets goal Pose
        if (robotSide == RobotSide.Blue)  {
            goal = Constants.Vision.blueGoal;
        } else {
            goal = Constants.Vision.redGoal;
        }



        ShooterValuesParent stv = shooterTestValuesV1;

        Pose futrGoal = goal;
        goal = goal.plus(offsetPose);

        double deltaX = goal.getX() - curPose.getX();
        double deltaY = goal.getY() - curPose.getY();
        double angleToGoal = Math.toDegrees(Math.atan2(deltaY, deltaX));

        angleToGoalVelocity = -((Math.toRadians(angleToGoal - lastAngleToGoal)) / ((System.currentTimeMillis() - lastTimeTurret) * 0.001));
        angleToGoalVelocity += follower.getAngularVelocity();

        lastAngleToGoal = angleToGoal;
        lastTimeTurret = System.currentTimeMillis();


        for (int i = 0; i <= 3; i++) {
            // Est Time In Flight for ball at current pose
            double timeInFlight = stv.get(curPose.distanceFrom(futrGoal)).timeInFlight;

            // Logic for future pose
            futrGoal = goal.plusVector(follower.getVelocity(), - timeInFlight);
        }


        // Logic for heading to goal
        double deltaXFutr = futrGoal.getX() - curPose.getX();
        double deltaYFutr = futrGoal.getY() - curPose.getY();
        double angleToGoalFutr = Math.toDegrees(Math.atan2(deltaYFutr, deltaXFutr));


        // Sets Turret angle
        turret.setTargetDeg(angleToGoalFutr - Math.toDegrees(curPose.getHeading()));

        // Gets shooter params based on future pose
        ShooterState futureShooterParams = stv.get(curPose.distanceFrom(futrGoal));

        // Sets shooter rpm
        shooter.adjustTargetRPM(futureShooterParams.rpm + rpmOffset);

        // gets hood angle
        shooter.setHoodDeg(futureShooterParams.hoodAngle + hoodAngleOffset);
    }

    public void setShooterTargetRPM(double set) {
        shooter.setTargetRPM(set);
    }

    public void prepareTurret() {
        Pose curPose = getCurrentPose();
        if (robotSide == RobotSide.Blue)  {
            goal = Constants.Vision.blueGoal;
        } else {
            goal = Constants.Vision.redGoal;
        }

        double deltaX = goal.getX() - curPose.getX();
        double deltaY = goal.getY() - curPose.getY();
        double angleToGoal = Math.toDegrees(Math.atan2(deltaY, deltaX));

        // Sets Turret angle
        turret.setTargetDeg(angleToGoal - Math.toDegrees(curPose.getHeading()));
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
        spinIntake(true);
    }
    public void spinIntake(boolean set) {
        if (set) {
            spitIntake = false;
        }
        intakeToggle = set;
    }
    long startSpit = 0L;
    boolean setTimeOnce = true;
    boolean autoSpitting = false;
    long intakeEnableTime = 0L;
    static final long SPINUP_IGNORE_MS = 300;  // tune 200–400ms
    private boolean setTimer;

    public void intakeUpdate() { //todo: move most of this logic into the intake class

        long now = System.currentTimeMillis();
        long spitTime = now - startSpit;

        boolean spinupIgnore = (now - intakeEnableTime) < SPINUP_IGNORE_MS;
        boolean highCurrent = intake.intakeMotor.isOverCurrent();
//        telemetry.addData("spinup ignore", spinupIgnore);
//        telemetry.addData("high current", highCurrent);
//        telemetry.addData("auto spitting", autoSpitting);

        if (!spinupIgnore && highCurrent && setTimeOnce) {
            startSpit = now;
            setTimeOnce = false;
            autoSpitting = true;
            setTimer = false;
        }

        // ---------- DURING AUTO-SPIT WINDOW ----------
        if (autoSpitting) {
            intake.setIntakePower(Constants.Intake.spitPower);

            // stop after configured spit time (units now consistent → ms)
            if (spitTime >= Constants.Intake.spitTime) {
                autoSpitting = false;
                setTimeOnce = true;
            }
            return; // prevents falling through to other logic
        }

        // ---------- NORMAL CONTROL ----------
        if (intakeToggle && !spitIntake) {
            if (!setTimer) {
                intakeEnableTime = System.currentTimeMillis();
                setTimer = true;
            }
            intake.setIntakePower(Constants.Intake.intakePower);
        } else if (spitIntake) {
            intake.setIntakePower(Constants.Intake.spitPower);
        } else {
            intake.setIntakePower(0);
        }
    }


    // SPINDEXER***********************************************************************************~

    public void isIntaking(boolean isIntaking) {
        this.isIntaking = isIntaking;
    }

    // for auto
    public void spindexerUpdate(boolean hasShotButton, boolean shootButton) {
        indexer.update(hasShotButton, isIntaking, readyToShootMotors(), inShootingZone(), shootButton);
    }
    public void spindexerUpdate(boolean shootButton) {
        spindexerUpdate(shooter.hasShot(), shootButton);
//        spindexerUpdate(indexer.hasShot(), shootButton);

    }

    // TELE-OP*************************************************************************************~
    public void intakeManual() {
        spinIntake();
    }

    public void autoIntake3() {
        doAutoIntake = true;
    }

    public void spitIntake() {
        spitIntake(true);
    }
    public void spitIntake(boolean set) {
        spitIntake = set;
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
        spinIntake(false);
    }


    // SYSTEM**************************************************************************************~
    public void update() {
        update(false, false);
    }
    double deleteMe = System.currentTimeMillis();
    public void update(boolean debug, boolean shootButton) {
        panelsTelemetry.addData("Distance", getCurrentPose().distanceFrom(goal));
        panelsTelemetry.addData("Tar Shooter RPM", shooter.getTargetRpm());
        panelsTelemetry.addData("Act Shooter RPM", shooter.getRPM());
        panelsTelemetry.addData("Hood Angle", shooter.getHoodPosDeg());
        panelsTelemetry.addData("Turret Degrees", turret.getAngle());
        panelsTelemetry.addData("Turret Ticks  ", turret.getTicks());
        panelsTelemetry.addData("Turret Tar Deg", turret.turretPID.getTargetPosition());
        panelsTelemetry.addData("Turret power", turret.turretPID.run());
        panelsTelemetry.addData("angVel", angleToGoalVelocity);


        long now = System.currentTimeMillis();
        panelsTelemetry.addData("dt", (now - lastTime) * 1e-3);
        lastTime = now;

        shooterUpdate();
        intakeUpdate();
        spindexerUpdate(shootButton);
        turretUpdate();
        teleopUpdate();
        follower.update();
        if (debug) {
            debug();
        }
        panelsTelemetry.update();
    }

    public void debug() {
        telemetry.addLine("=== VISION ===");
        telemetry.addData("Motif", motif);


        telemetry.addLine("=== Turret ===");
        telemetry.addData("Turret Degrees", turret.getAngle());
        telemetry.addData("Turret Ticks  ", turret.getTicks());
        telemetry.addData("Turret Tar Deg", turret.turretPID.getTargetPosition());
        telemetry.addData("Turret power", turret.turretPID.run());

        telemetry.addLine("=== COLOR ===");
        telemetry.addData("indexer r", indexer.indexerState.getColorRGBA().red);
        telemetry.addData("indexer g", indexer.indexerState.getColorRGBA().green);
        telemetry.addData("indexer b", indexer.indexerState.getColorRGBA().blue);
        telemetry.addData("indexer a", indexer.indexerState.getColorRGBA().alpha);
        telemetry.addData("indexer col", indexer.indexerState.getColor());
        telemetry.addData("indexer dis", indexer.indexerState.getDistance());

        telemetry.addLine("=== POSITION ===");
        telemetry.addData("guess pose", getCurrentPose());
        telemetry.addData("last cam pose", lastCamPose);
        telemetry.addData("offset", offsetPose);
        telemetry.addData("distance", getCurrentPose().distanceFrom(goal));
        telemetry.addData("in shooting zone", inShootingZone());

        telemetry.addLine("=== QUEUE ===");
        if (indexer.getQueue().isEmpty()) {
            telemetry.addLine("Nothing in queue");
        }
        for (BallColor ball : indexer.getQueue()) {
            telemetry.addData("qball", ball);
        }


//            telemetry.addData("hood", shooter.getHoodPosDeg());
//            telemetry.addData("rpm", shooter.getRPM());

//        panelsTelemetry.addData("turret target", turret.turretPID.getTargetPosition());
//        panelsTelemetry.addData("turret actual", turret.getAngle());
//            panelsTelemetry.addData("shooter target", shooter.getTargetRpm());
//            panelsTelemetry.addData("shooter actual", shooter.getRPM());
//            panelsTelemetry.update(telemetry);


        panelsTelemetry.addData("P", indexer.indexerState.pidfController.getPOutput());
        panelsTelemetry.addData("I", indexer.indexerState.pidfController.getIOutput());
        panelsTelemetry.addData("D", indexer.indexerState.pidfController.getDOutput());
        panelsTelemetry.addData("Indexer Error", indexer.indexerState.pidfController.getError());
        panelsTelemetry.addData("dt", indexer.indexerState.pidfController.getDeltaTime());
        panelsTelemetry.addData("raw ticks", indexer.indexerState.getEncoderTicks());
        panelsTelemetry.update();
        // PID Telemetry
        int i = 0;
        for (BallColor index : indexer.indexerState.getBallCells()) {
            telemetry.addData("BallCell" + i, index);
            i++;
        }
        telemetry.addData("in shooting zone", inShootingZone());
        telemetry.addData("ready to shoot", readyToShoot());
        telemetry.addData("is intaking", isIntaking);
        telemetry.addData("has shot", thingies);
        panelsTelemetry.addData("derivative", shooter.derivative);
        panelsTelemetry.addData("once shot", shooter.onceShot);
        panelsTelemetry.addData("previous error", shooter.previousError);
        panelsTelemetry.addData("error", shooter.shooterPID.getError());
        panelsTelemetry.addData("target rpm", shooter.getTargetRpm());
        panelsTelemetry.addData("actual rpm", shooter.getRPM());
        panelsTelemetry.update();
        panelsTelemetry.addData("target ticks", indexer.indexerState.pidfController.getTargetPosition());
        panelsTelemetry.addData("is at position", indexer.indexerState.atPosition);
        panelsTelemetry.addData("indexer state", indexer.indexerState.getIndexerPosition());
        panelsTelemetry.addData("isStuck", indexer.indexerState.pidfController.isStuck());
        panelsTelemetry.addData("Integral Error", indexer.indexerState.pidfController.getIOutput());
        panelsTelemetry.addData("Indexer Error", indexer.indexerState.pidfController.getError());
        panelsTelemetry.addData("dt", indexer.indexerState.pidfController.getDeltaTime());
        panelsTelemetry.addData("P", indexer.indexerState.pidfController.getPOutput());
        panelsTelemetry.addData("I", indexer.indexerState.pidfController.getIOutput());
        panelsTelemetry.addData("D", indexer.indexerState.pidfController.getDOutput());
        panelsTelemetry.addData("velocity", indexer.indexerState.pidfController.getVelocity());
        panelsTelemetry.addData("desired velocity", indexer.indexerState.pidfController.getDesiredVelocity());
        panelsTelemetry.addData("PID power", indexer.indexerState.getPower());
        telemetry.addData("average time to hit target ms", indexer.indexerState.avgTimeSec);
        deleteMe = System.currentTimeMillis();
        panelsTelemetry.addData("Turret Degrees", turret.getAngle());
        panelsTelemetry.addData("Turret Ticks  ", turret.getTicks());
        panelsTelemetry.addData("Turret Tar Deg", turret.turretPID.getTargetPosition());
        panelsTelemetry.addData("Turret power", clamp(turret.turretPID.run(), -1, 1));

        panelsTelemetry.addLine("=== SHOOTER ===");
        panelsTelemetry.addData("Shooter RPM", shooter.getRPM());
        panelsTelemetry.addData("Shooter power", shooter.lastPower);
        panelsTelemetry.addData("Tar Shooter VEL", shooter.shooterPID.getTargetPosition());
        panelsTelemetry.addData("Shooter VEL", shooter.getVelocity());
        panelsTelemetry.addData("Shooter ERR", shooter.shooterPID.getError());
        panelsTelemetry.addData("Hood Angle", shooter.getHoodPosDeg());
        panelsTelemetry.update();
        telemetry.addData("current alert", intake.intakeMotor.isOverCurrent());
        telemetry.addData("current", intake.intakeMotor.getCurrent(CurrentUnit.AMPS));
        long now = System.currentTimeMillis();
        telemetry.addData("dt", (now - lastTime) * 1e-3);
        lastTime = now;
        telemetry.update();
    }
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public void stopAllSubsystems() {
        oneBallCase = 0;
        shooter.stopShooter();
        shooter.setHoodDeg(0);
        stopIntake();
    }

    // TESTING*************************************************************************************~

}
