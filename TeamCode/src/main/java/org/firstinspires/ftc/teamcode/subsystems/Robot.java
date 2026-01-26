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
import org.firstinspires.ftc.teamcode.pedroPathing.math.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.FieldShapes;
import org.firstinspires.ftc.teamcode.util.RobotSide;
import org.firstinspires.ftc.teamcode.util.ShapeDetection;
import org.firstinspires.ftc.teamcode.util.shooterInterpolation.ShooterState;
import org.firstinspires.ftc.teamcode.util.shooterInterpolation.ShooterTestValues;
import org.firstinspires.ftc.teamcode.util.shooterInterpolation.ShooterValuesParent;
//todo import java.awt.Shape to make the inShootingZone() better


public class Robot {
    public Turret turret;
    public Lights lights;
    private Intake intake;
    private Vision vision;
    public Shooter shooter;
    public Follower follower;
    public RobotSide robotSide;
    public Drivetrain drivetrain;
    public SmartIndexerButEvenNewer indexer;


    ShooterTestValues shooterTestValues;
    public ElapsedTime shooterTimer;
    TelemetryManager panelsTelemetry;

    double startTime;

    Timer opmodeTimer = new Timer();

    boolean spitIntake = false;
    boolean isIntaking = false;
    boolean smartShoot = false;
    boolean intakeOverride = false;

    Pose lastCamPose = new Pose(0,0,0);
    // Offset pose to aim for
    public Pose offsetPose = new Pose(0,0,0);
    //This is an array of the 3 special colors of the games MOTIF
    public BallColor[] motif = null;
    int thingies = 0;
    public double previousAngleToGoalVelocity;
    public double angleToGoalAcceleration;
    Pose goal = new Pose(0,0,0);
    int i = 0;
    long lastTime = System.nanoTime();
    Telemetry telemetry;
    public Robot(Telemetry telemetry, HardwareMap hardwareMap, RobotSide robotSide, int startTurretTicks, double startIndexerTicks) {
        this(telemetry, hardwareMap, robotSide, startTurretTicks, startIndexerTicks, new BallColor[]{BallColor.None, BallColor.None, BallColor.None});
    }
    public Robot(Telemetry telemetry, HardwareMap hardwareMap, RobotSide robotSide, int startTurretTicks, double startIndexerTicks, BallColor[] ballsInIndexer) {
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        Drawing.init();
        CommonPoses.init(robotSide);
        this.telemetry = telemetry;
        this.robotSide = robotSide;
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        lights = new Lights(hardwareMap);
        intake = new Intake(hardwareMap);
        vision = new Vision(hardwareMap, robotSide);
        indexer = new SmartIndexerButEvenNewer(hardwareMap, ballsInIndexer, startIndexerTicks);
        turret = new Turret(hardwareMap, startTurretTicks, robotSide);
        drivetrain = new Drivetrain(hardwareMap);
        shooter = new Shooter(hardwareMap);

        follower.setStartingPose(new Pose(0,0,0));

        shooterTimer = new ElapsedTime();
        shooterTestValues = new ShooterTestValues();
        shooter.resetController();

        if (robotSide == RobotSide.Blue)  {
            goal = Constants.Vision.blueGoal;
        } else {
            goal = Constants.Vision.redGoal;
        }
    }
    // VISION**************************************************************************************~
    // sets the motif if we havent seen it yet
    // always return the motif
    public BallColor[] getMotif() {
        if (motif == null) {
            motif = vision.getMotif();
        }
//        return motif;
        return new BallColor[]{BallColor.Purple, BallColor.Green, BallColor.Purple};
    }

    public double distanceToGoal() {
        Pose curPose = follower.getPose();
        double deltaX = goal.getX() - curPose.getX();
        double deltaY = goal.getY() - curPose.getY();

        return Math.hypot(deltaX, deltaY);
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
    public boolean inFarZone() {
        Pose curPose = follower.getPose();
        return ShapeDetection.doesRobotIntersect(FieldShapes.farTriangle, curPose);

//        return distanceToGoal() > 107;
    }
    public void reZeroAtCorner() {
        follower.setPose(robotSide == RobotSide.Blue ? new Pose(-64.938, -59.7542, -1.573) : new Pose(-61.9, 60.9674, 1.5688));
    }
    Pose holdPose;
    public void holdPoint(boolean hold) {
        if (holdPose == null){holdPose = getCurrentPose();}
        else {
            follower.holdPoint(holdPose);
        }
        if (!hold){holdPose = null;}
    }
    // TURRET**************************************************************************************~

    double angleToGoalVelocity;
    public void turretUpdate() {
        turret.update(angleToGoalVelocity, angleToGoalAcceleration);
    }

    // SHOOTER*************************************************************************************~
    public boolean readyToShoot() {
        return inShootingZone() && readyToShootMotors();
    }

    public boolean readyToShootMotors() {
        return shooter.closeEnoughToTarget() && turret.closeEnoughToTarget(getCurrentPose()) && shooter.getRPM() > 100 && shooter.getUsePID();
    }


    // Adds a ball of color ball color to queuedBalls list
    public void qBall(BallColor qdColor) {
        indexer.addToQueue(qdColor);
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
    double deltaDeg;
    public void prepareShooter(double rpmOffset, double hoodAngleOffset) {

        // Gets current Pose
        Pose curPose = follower.getCenterOfShooterPose();

        ShooterValuesParent stv = shooterTestValues;

        Pose futrGoal = goal;
        goal = goal.plus(offsetPose);

        double deltaX = goal.getX() - curPose.getX();
        double deltaY = goal.getY() - curPose.getY();
        double angleToGoal = Math.toDegrees(Math.atan2(deltaY, deltaX));

        // this is to pass to the feed forward on the turret to offset for the rate of change of the angle to goal
        previousAngleToGoalVelocity = angleToGoalVelocity;
        angleToGoalVelocity = -((Math.toRadians(angleToGoal - lastAngleToGoal)) / ((System.nanoTime() - lastTimeTurret) * 1e-9)); // todo work in velocity vector
        angleToGoalVelocity += follower.getAngularVelocity();

        angleToGoalAcceleration = (angleToGoalVelocity - previousAngleToGoalVelocity) / ((System.nanoTime() - lastTimeTurret) * 1e-9);
        lastAngleToGoal = angleToGoal;
        lastTimeTurret = System.nanoTime();


        // Est Time In Flight for ball at current pose
        double timeInFlight = stv.get(curPose.distanceFrom(futrGoal)).timeInFlight;

        Vector virtualVelocity = follower.getVelocity().plus(follower.getAcceleration().times(.015));
        // Logic for future pose
        futrGoal = goal.plusVector(virtualVelocity, - timeInFlight);

        Pose goalOffset;
        if (robotSide == RobotSide.Blue) {
            goalOffset = Constants.Vision.blueGoalAimOffset;
        } else {
            goalOffset = Constants.Vision.redGoalAimOffset;
        }

        double deltaXFutr = futrGoal.getX() + goalOffset.getX() - curPose.getX();
        double deltaYFutr = futrGoal.getY() + goalOffset.getY() - curPose.getY();
        double angleToGoalFutr = Math.toDegrees(Math.atan2(deltaYFutr, deltaXFutr));


        // Sets Turret angle
        turret.setTargetDeg(angleToGoalFutr - Math.toDegrees(curPose.getHeading()));

        // Gets shooter params based on future pose
        ShooterState futureShooterParams = stv.get(curPose.distanceFrom(futrGoal));

        // Sets shooter rpm
        shooter.adjustTargetRPM(futureShooterParams.rpm + rpmOffset);

        double hoodDeg = futureShooterParams.hoodAngle;
        double hoodRad = Math.toRadians(hoodDeg);

        double rpmRatio = shooter.getTargetRpm() / shooter.getRPM();

        double correctedRad =
                Math.atan(rpmRatio * Math.tan(hoodRad));

        deltaDeg =
                Math.toDegrees(correctedRad - hoodRad);

        if (Math.abs(deltaDeg) < 0.8) {
            deltaDeg = 0;
        }

        deltaDeg = clamp(deltaDeg, 0.0, 8.0);

        // gets hood angle
        shooter.setHoodDeg(futureShooterParams.hoodAngle + deltaDeg + hoodAngleOffset);
    }

    public void setShooterTargetRPM(double set) {
        shooter.setTargetRPM(set);
    }

    public void calculateIdealShot() {
        calculateIdealShot(0, 0);
    }
    public void calculateIdealShot(double rpmOffset, double hoodAngleOffset) {
        double g = 386.09;

        // Gets current Pose
        Pose curPose = follower.getCenterOfShooterPose();

        // Gets goal Pose
        if (robotSide == RobotSide.Blue)  {
            goal = Constants.Vision.blueGoal;
        } else {
            goal = Constants.Vision.redGoal;
        }
        goal = goal.plus(offsetPose);


        double height = 32; //todo: verify this height (inches)

        double deltaX = goal.getX() - curPose.getX();
        double deltaY = goal.getY() - curPose.getY();
        double angleToGoal = (Math.atan2(deltaY, deltaX));

        // this is to pass to the feed forward on the turret to offset for the rate of change of the angle to goal
        angleToGoalVelocity = -((angleToGoal - lastAngleToGoal) / ((System.nanoTime() - lastTimeTurret) * 1e-9));
        angleToGoalVelocity += follower.getAngularVelocity();

        lastAngleToGoal = angleToGoal;
        lastTimeTurret = System.nanoTime();

        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        double entryAngle = Math.toRadians(-45);

        double hoodAngle = Math.atan(2 * height / distance - Math.tan(entryAngle)); //todo make sure atan returns a number
        double flywheelSpeed = Math.sqrt(g * distance * distance / (2 * Math.pow(Math.cos(hoodAngle), 2) * (distance * Math.tan(hoodAngle) - height)));

        Vector robotVelocity = follower.getVelocity();

        double coordinateTheta = robotVelocity.getTheta() - angleToGoal;

        double parallelComponent = - Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

        double vz = flywheelSpeed * Math.sin(hoodAngle);
        double time = distance / (flywheelSpeed * Math.cos(hoodAngle));
        double ivr = distance / time + parallelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr = nvr * time;

        hoodAngle = Math.atan(vz / nvr);
        flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (distance * Math.tan(hoodAngle) - height)));

        double turretVelocityOffset = Math.atan2(perpendicularComponent, ivr);
        panelsTelemetry.addData("target hood angle", hoodAngle);
        panelsTelemetry.addData("target velocity", flywheelSpeed);

        turret.setTargetRad(angleToGoal - turretVelocityOffset - curPose.getHeading());

        shooter.setHoodRad((Math.toRadians(hoodAngleOffset)) + Math.PI/2 - hoodAngle - hoodAngleOffset);

        double dragCoefficient = .00437 * .6;
        double dragCompensation = Math.pow(Math.E, dragCoefficient * distanceToGoal());
        shooter.setTargetRPM(rpmOffset + shooter.velocityToRPM(flywheelSpeed)); // todo: fix the velocity to rpm function
    }

    double pictureTime = 0;
    public void shooterUpdate() {
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

    public void setIntakeOverride(boolean set) {
        intakeOverride = set;
    }
    public void doIntake() {
        spinIntake();
    }

    public void spitIntake() {
        spitIntake(true);
    }
    public void spitIntake(boolean set) {
        spitIntake = set;
    }

    public void stopIntake() {
        spinIntake(false);
    }
    public void spinIntake() {
        spinIntake(true);
    }
    public void spinIntake(boolean set) {
        isIntaking = set;
    }
//    long startSpit = 0L;
//    boolean setTimeOnce = true;
//    boolean autoSpitting = false;
//    long intakeEnableTime = 0L;
//    static final long SPINUP_IGNORE_MS = 300;  // tune 200–400ms
//    private boolean setTimer;

    public void intakeUpdate() {
        if (spitIntake || indexer.doSpit()) {
            intake.spit(true);
        } else if ((isIntaking && indexer.allowIntaking()) || intakeOverride) {
            intake.intake(true);
        } else if (indexer.shooting){
            intake.setIntakePower(.5);
        } else {
            intake.intake(false);
        }
        // intake jam detection (commented out because we dont want it)

//        long now = System.currentTimeMillis();
//        long spitTime = now - startSpit;
//
//        boolean spinupIgnore = (now - intakeEnableTime) < SPINUP_IGNORE_MS;
//        boolean highCurrent = intake.intakeMotor.isOverCurrent();
//
//        if (!spinupIgnore && highCurrent && setTimeOnce) {
//            startSpit = now;
//            setTimeOnce = false;
//            autoSpitting = true;
//            setTimer = false;
//        }
//
//        if (autoSpitting) {
//             stop after configured spit time
//            if (spitTime >= Constants.Intake.spitTime) {
//                autoSpitting = false;
//                setTimeOnce = true;
//            }
//            return;
//        }
//
//        if (intakeToggle && !spitIntake) {
//            if (!setTimer) {
//                intakeEnableTime = System.currentTimeMillis();
//                setTimer = true;
//            }
//            intake.setIntakePower(Constants.Intake.intakePower);
//        } else if (spitIntake) {
//            intake.setIntakePower(Constants.Intake.spitPower);
//        } else {
//            intake.setIntakePower(0);
//        }

    }


    // SPINDEXER***********************************************************************************~

    public void doSmartShoot(boolean set) {
        smartShoot = set;
    }
    public void isIntaking(boolean isIntaking) {
        this.isIntaking = isIntaking;
    }

    public void shootAll() {
        indexer.shootAll();
    }
    public void spindexerUpdate() {
        indexer.update(isIntaking, readyToShootMotors(), smartShoot, inFarZone());
    }

    // LIGHTS**************************************************************************************~
    public void lightsUpdate() {
        lights.setBallColors(indexer.getBallCells(), indexer.getQueuedBalls(), smartShoot);
        lights.update();
    }

    // TELE-OP*************************************************************************************~

    // AUTONOMOUS**********************************************************************************~

    public void resetTimers() {
        opmodeTimer.resetTimer();
    }

    public double getOpmodeTimeSeconds() {
        return opmodeTimer.getElapsedTimeSeconds();
    }

    // SYSTEM**************************************************************************************~
    public void start() {
        resetTimers();
        indexer.start();
    }
    public void update() {
        update(false);
    }
    double previousHoodAngle;
    double previousTime;
    double maxHoodAngleChange;
    public void update(boolean debug) {
        long now = System.nanoTime();
        panelsTelemetry.addData("dt", (now - lastTime) * 1e-6);
        lastTime = now;

        shooterUpdate();
        intakeUpdate();
        spindexerUpdate();
        turretUpdate();
        follower.update();
        lightsUpdate();

        panelsTelemetry.addData("Hood Angle correction", deltaDeg);
        panelsTelemetry.addData("Shoot overPower", shooter.shooterPID.run() >= 1 ? 1000 : 0);
//        panelsTelemetry.addData("Shoot power", shooter.shooterPID.run());
        panelsTelemetry.addData("RPM", shooter.getRPM());
//        panelsTelemetry.addData("hood angle", shooter.getHoodPosDeg());
        panelsTelemetry.addData("target rpm", shooter.shooterPID.getTargetPosition());
//        panelsTelemetry.addData("rpm error", shooter.shooterPID.getError());
//        panelsTelemetry.addData("shooter velocity", shooter.getRPM());
//        panelsTelemetry.addData("hood pos", shooter.getHoodPosDeg());
//        telemetry.addData("distance to goal", distanceToGoal());
//        telemetry.addData("far zone", inFarZone());
//        telemetry.addData("is at position", indexer.isAtPosition());
//        telemetry.addData("encoder position", indexer.getEncoderPercentage());
//        telemetry.addData("dumbshoot1", indexer.dumbShootState2);
//        telemetry.addData("dumbshoot2", indexer.dumbShootState2);

//        panelsTelemetry.addData("ready to shoot", readyToShootMotors());
//        panelsTelemetry.addData("shooter error", shooter.shooterPID.getError());
//        panelsTelemetry.addData("shooter error derivative", shooter.shooterPID.getErrorDerivative());
//        long shooter = System.currentTimeMillis();
//        long intake = System.currentTimeMillis();
//        long spindexer = System.currentTimeMillis();
//        long turret = System.currentTimeMillis();
//        long follower = System.currentTimeMillis();
//        long lights = System.currentTimeMillis();
//        Drawing.drawShapesDebug(this.follower);
//        Drawing.drawDebug(this.follower);

//        panelsTelemetry.addData("shooter", -(start - shooter));
//        panelsTelemetry.addData("intake", -(shooter - intake));
//        panelsTelemetry.addData("spindexer", -(intake - spindexer));
//        panelsTelemetry.addData("turret", -(spindexer - turret));
//        panelsTelemetry.addData("follower", -(turret - follower));
//        panelsTelemetry.addData("lights", -(follower - lights));
//        panelsTelemetry.addData("all", (lights - lastTime));
//        lastTime = System.currentTimeMillis();


//        panelsTelemetry.addData("spindexer", (spindexer - intake) * 1e-3);
//        panelsTelemetry.addData("turret err", (turret.turretPID.getTargetPosition() - turret.getAngle()));
//        panelsTelemetry.addData("turret pos", turret.getAngle());
//        panelsTelemetry.addData("turret pow", turret.turretPID.run());
//        panelsTelemetry.addData("turret Accel", angleToGoalAcceleration);
//        panelsTelemetry.addData("turret velocity", angleToGoalVelocity);
//        Drawing.drawShapesDebug(this.follower);
        if (debug) {
            debug();
        }
//        telemetry.update();
        panelsTelemetry.update();
    }

    public void debug() {

        panelsTelemetry.addData("Tar Shooter RPM", shooter.getTargetRpm());
        panelsTelemetry.addData("Act Shooter RPM", shooter.getRPM());
        panelsTelemetry.addData("turret error", turret.turretPID.getError());
        panelsTelemetry.addData("spindexer error", Math.abs(indexer.getEncoderPercentage() - indexer.lastIndexerTarget));
        Drawing.drawShapesDebug(follower);

        panelsTelemetry.addData("turret target", turret.turretPID.getTargetPosition());
        panelsTelemetry.addData("turret actual", turret.getAngle());

        telemetry.addData("hood", shooter.getHoodPosDeg());
        telemetry.addData("rpm", shooter.getRPM());
        telemetry.addData("rpm target", shooter.getTargetRpm());

        panelsTelemetry.addData("shooter target", shooter.getTargetRpm());
        panelsTelemetry.addData("shooter actual", shooter.getRPM());
        panelsTelemetry.addData("sotf", angleToGoalVelocity);
        panelsTelemetry.addData("distance", follower.getCenterOfShooterPose().distanceFrom(goal));
        telemetry.addData("Ready to shoot", readyToShootMotors());
        telemetry.addLine("=== VISION ===");
        telemetry.addData("Motif", motif);

                double rateOfChangeOfHoodAngle = (shooter.getHoodPosDeg() - previousHoodAngle) / (System.currentTimeMillis() - previousTime);
        previousTime = System.currentTimeMillis();
        previousHoodAngle = shooter.getHoodPosDeg();
        maxHoodAngleChange = Math.max(rateOfChangeOfHoodAngle, maxHoodAngleChange);
        panelsTelemetry.addData("rate of change of hood angle", rateOfChangeOfHoodAngle);
        panelsTelemetry.addData("Hood Angle", shooter.getHoodPosDeg());
        panelsTelemetry.addData("max rate of change of hood", maxHoodAngleChange);



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
        telemetry.addData("distance", follower.getCenterOfShooterPose().distanceFrom(goal));
        telemetry.addData("in shooting zone", inShootingZone());

        telemetry.addLine("=== QUEUE ===");
        for (BallColor ball : indexer.getBallCells()) {
            telemetry.addData("qball", ball);
        }

        panelsTelemetry.addData("raw percentage", indexer.getEncoderPercentage());
        // PID Telemetry
        int i = 0;
        for (BallColor index : indexer.getBallCells()) {
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
        panelsTelemetry.addData("target ticks", indexer.lastIndexerTarget);
        panelsTelemetry.addData("is at position", indexer.isAtPosition());
        panelsTelemetry.addData("indexer state", indexer.currentIndexerState);
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
        telemetry.addData("current alert", intake.intakeMotor.isOverCurrent());
        telemetry.addData("current", intake.intakeMotor.getCurrent(CurrentUnit.AMPS));
    }
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public void stopAllSubsystems() {
        lights.stop();
        intake.stop();
        turret.stop();
        vision.stop();
        indexer.stop();
        shooter.stop();
        follower.stop();
        drivetrain.stop();

    }

    // TESTING*************************************************************************************~

}
