package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.modularAutos.Common;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.FieldShapes;
import org.firstinspires.ftc.teamcode.util.RobotSide;
import org.firstinspires.ftc.teamcode.util.ShapeDetection;
import org.firstinspires.ftc.teamcode.util.ShootOnTheFly.HoodAngleCompensation;
import org.firstinspires.ftc.teamcode.util.ShootOnTheFly.ShotCalculator;
import org.firstinspires.ftc.teamcode.util.ShootOnTheFly.ShotContext;
import org.firstinspires.ftc.teamcode.util.ShootOnTheFly.ShotSolution;
import org.firstinspires.ftc.teamcode.util.ShootOnTheFly.ShotType;
import org.firstinspires.ftc.teamcode.util.shooterInterpolation.ShooterTestValues;

import java.util.List;
//todo import java.awt.Shape to make the inShootingZone() better


public class Robot {
    List<LynxModule> hubs;
    public Turret turret;
    public Lights lights;
    private Intake intake;
    private Vision vision;
    public Shooter shooter;
    public Follower follower;
    public RobotSide robotSide;
    public Drivetrain drivetrain;
    public Indexer indexer;
    private ShotCalculator shotCalculator;
    private HoodAngleCompensation hoodAngleCompensation;
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
    Pose shootFromPose = null;
    Telemetry telemetry;
    public Robot(Telemetry telemetry, HardwareMap hardwareMap, RobotSide robotSide, int startTurretTicks, double startIndexerTicks) {
        this(telemetry, hardwareMap, robotSide, startTurretTicks, startIndexerTicks, new BallColor[]{BallColor.None, BallColor.None, BallColor.None});
    }
    public Robot(Telemetry telemetry, HardwareMap hardwareMap, RobotSide robotSide, int startTurretTicks, double startIndexerTicks, BallColor[] ballsInIndexer) {
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        Drawing.init();
        Common.init(robotSide);
        this.telemetry = telemetry;
        this.robotSide = robotSide;
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        lights = new Lights(hardwareMap);
        intake = new Intake(hardwareMap);
        vision = new Vision(hardwareMap, robotSide);
        indexer = new Indexer(hardwareMap, ballsInIndexer, startIndexerTicks);
        turret = new Turret(hardwareMap, startTurretTicks, robotSide);
        drivetrain = new Drivetrain(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shotCalculator = new ShotCalculator();
        hoodAngleCompensation = new HoodAngleCompensation();

        follower.setStartingPose(new Pose(0,0,0));

        shooterTimer = new ElapsedTime();
        shooter.resetController();

        if (robotSide == RobotSide.Blue)  {
            goal = Constants.Vision.blueGoal;
        } else {
            goal = Constants.Vision.redGoal;
        }

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
    // VISION**************************************************************************************~
    // sets the motif if we havent seen it yet
    // always return the motif
    public BallColor[] getMotif() {
        return getMotif(false);
    }
    public BallColor[] getMotif(boolean force) {
        if (force || motif == null) {
            motif = vision.getMotif();
        }
        return motif;
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
    }
    public boolean farBack() {
        return distanceToGoal() > 110;
    }
    public void reZeroAtCorner() {
        offsetPose = new Pose();
        follower.setPose(Common.StartPoses.reZeroAtCorner);
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
        turret.update(angleToGoalVelocity + follower.getAngularVelocity(), angleToGoalAcceleration + follower.getAcceleration().getTheta());
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

    public void setShootFromPose(Pose shootFromPose) {
        this.shootFromPose = shootFromPose;
    }

    public void prepareShooter() {
        prepareShooter(ShotType.TABLE);
    }
    double rpmRatio = 1;
    public void prepareShooter(ShotType shotType) {
        ShotContext ctx = new ShotContext();

        ctx.robotPose = follower.getCenterOfShooterPose();
        Pose goalPose;
        if (robotSide == RobotSide.Blue) {
            goalPose = shotType == ShotType.PHYSICAL ? Constants.Vision.blueGoalPhysics : Constants.Vision.blueGoal;
        } else {
            goalPose = shotType == ShotType.PHYSICAL ? Constants.Vision.redGoalPhysics : Constants.Vision.redGoal;
        }
        ctx.goalPose = goalPose.plus(offsetPose);
        ctx.velocity = follower.getVelocity();
        ctx.acceleration = follower.getAcceleration();
        ctx.side = robotSide;
        ctx.rpmRatio = rpmRatio;


        ShotSolution s = shotCalculator.solveShot(ctx, shotType);

        // expose turret feedforward
        angleToGoalVelocity = s.turretVel;
        angleToGoalAcceleration = s.turretAccel;

        turret.setTargetRad(s.turretAngleRad);

        shooter.setTargetRPM(s.rpm);

        double deltaDeg = hoodAngleCompensation.hoodAngleCompensation(s.rpm, shooter.getRPM(), s.hoodDeg);
        rpmRatio = hoodAngleCompensation.getRpmRatio();

        shooter.setHoodDeg(s.hoodDeg + deltaDeg);
    }

    public double deltaDeg;
    public double hoodAngleCompensation(ShooterState futureShooterParams) {
        double hoodDeg = futureShooterParams.hoodAngle;
        double hoodRad = Math.toRadians(hoodDeg);

        rpmRatio = futureShooterParams.rpm / shooter.getRPM();

        double correctedRad =
                Math.atan(rpmRatio * Math.tan(hoodRad));

        double deltaDeg =
                Math.toDegrees(correctedRad - hoodRad);

        if (Math.abs(deltaDeg) < .9) {
            deltaDeg = 0;
        }
        this.deltaDeg = deltaDeg;

        deltaDeg = clamp(deltaDeg, -8.0, 8.0);

        if (smartShoot) {
            deltaDeg = 0;
        }
        return deltaDeg;
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


        double height = 32;
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

        double hoodAngle = Math.atan(2 * height / distance - Math.tan(entryAngle));
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
    boolean intakeInit = false;

    public void intakeUpdate() {
        // don't spin until robot is started
        if (!intakeInit && isIntaking) {
            intakeInit = true;
        } else if (!intakeInit) {
            intake.setIntakePower(0);
            return;
        }
        intake.update(spitIntake, isIntaking, indexer.shooting, indexer.doSpit(), indexer.allowIntaking(), indexer.isPlugged(), intakeOverride);
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
        indexer.update(isIntaking, readyToShootMotors(), smartShoot, farBack(), follower.getPose());
    }
    public void indexerUnjam() {
        indexer.unjam();
    }
    public boolean basicallyHas3() {
        return indexer.isPlugged() && intake.isBeamBroken();
    }

    // LIGHTS**************************************************************************************~
    public void lightsUpdate() {
        lights.setBallColors(indexer.getBallCells(), indexer.getQueuedBalls(), basicallyHas3(), smartShoot);
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
        vision.start();
    }
    public void update() {
        update(false);
    }
    double previousHoodAngle;
    double previousTime;
    double maxHoodAngleChange;
    double lastTime = System.nanoTime();
    public void update(boolean debug) {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        shooterUpdate();
        shooterUpdate();
        intakeUpdate();
        spindexerUpdate();
        turretUpdate();
        follower.update();
        lightsUpdate();

        if (debug) {
            debug();
        }
        long now = System.nanoTime();
        panelsTelemetry.addData("RPM", shooter.getRPM());
        panelsTelemetry.addData("RPM error", shooter.shooterPID.getError());
        panelsTelemetry.addData("dt", (now - lastTime) * 1e-6);
        panelsTelemetry.update();
        lastTime = now;
    }

    public void debug() {


//        panelsTelemetry.addData("Hood Angle correction", deltaDeg);
//        panelsTelemetry.addData("Shoot overPower", shooter.shooterPID.run() >= 1 ? 1000 : 0);
//        panelsTelemetry.addData("Shoot power", shooter.shooterPID.run());
//        panelsTelemetry.addData("RPM", shooter.getRPM());
//        panelsTelemetry.addData("hood angle", shooter.getHoodPosDeg());
//        panelsTelemetry.addData("target rpm", shooter.shooterPID.getTargetPosition());
//        panelsTelemetry.addData("rpm error", shooter.shooterPID.getError());
//        panelsTelemetry.addData("shooter velocity", shooter.getRPM());
//        panelsTelemetry.addData("hood pos", shooter.getHoodPosDeg());
//        telemetry.addData("distance to goal", distanceToGoal());
//        telemetry.addData("far zone", inFarZone());
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
        panelsTelemetry.addData("turret err", (turret.turretPID.getTargetPosition() - turret.getAngle()));
        panelsTelemetry.addData("turret target", turret.turretPID.getTargetPosition());
        panelsTelemetry.addData("turret actual", turret.getAngle());

//        panelsTelemetry.addData("shooter", -(start - shooter));
//        panelsTelemetry.addData("intake", -(shooter - intake));
//        panelsTelemetry.addData("spindexer", -(intake - spindexer));
//        panelsTelemetry.addData("turret", -(spindexer - turret));
//        panelsTelemetry.addData("follower", -(turret - follower));
//        panelsTelemetry.addData("lights", -(follower - lights));
//        panelsTelemetry.addData("all", (lights - lastTime));
//        lastTime = System.currentTimeMillis();


//        panelsTelemetry.addData("spindexer", (spindexer - intake) * 1e-3);
//        panelsTelemetry.addData("turret pos", turret.getAngle());
//        panelsTelemetry.addData("turret pow", turret.turretPID.run());
//        panelsTelemetry.addData("turret Accel", angleToGoalAcceleration);
//        panelsTelemetry.addData("turret velocity", angleToGoalVelocity);
//        Drawing.drawShapesDebug(this.follower);

        telemetry.addData("encoder", indexer.getEncoderPercentage());
        telemetry.addData("Indexer Encoder Offset", indexer.encoderOffsetFromAuto);
        telemetry.addData("servo", indexer.spindexer2.getPosition());
        telemetry.addData("servo", indexer.lastIndexerTarget);


        panelsTelemetry.addData("Tar Shooter RPM", shooter.getTargetRpm());
        panelsTelemetry.addData("Act Shooter RPM", shooter.getRPM());
        panelsTelemetry.addData("turret error", turret.turretPID.getError());
        panelsTelemetry.addData("spindexer error", Math.abs(indexer.getEncoderPercentage() - indexer.lastIndexerTarget));
        Drawing.drawShapesDebug(follower);


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
        telemetry.addData("is at position", indexer.isAtPosition());
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
        follower.update();
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
