package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.modularAutos.Common;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.math.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.DBSCANLiteClustering;
import org.firstinspires.ftc.teamcode.util.FieldShapes;
import org.firstinspires.ftc.teamcode.util.MeanBallPoses;
import org.firstinspires.ftc.teamcode.util.RobotSide;
import org.firstinspires.ftc.teamcode.util.ShapeDetection;
import org.firstinspires.ftc.teamcode.util.ShootOnTheFly.HoodAngleCompensation;
import org.firstinspires.ftc.teamcode.util.ShootOnTheFly.ShotCalculator;
import org.firstinspires.ftc.teamcode.util.ShootOnTheFly.ShotContext;
import org.firstinspires.ftc.teamcode.util.ShootOnTheFly.ShotSolution;
import org.firstinspires.ftc.teamcode.util.ShootOnTheFly.ShotType;
import org.firstinspires.ftc.teamcode.util.VisionSpline;
import org.firstinspires.ftc.teamcode.util.VisionTypes;

import java.util.List;


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
    public Climber climber;
    private ShotCalculator shotCalculator;
    private HoodAngleCompensation hoodAngleCompensation;
    public ElapsedTime shooterTimer;
    TelemetryManager panelsTelemetry;
    HardwareMap.DeviceMapping<VoltageSensor> voltageSensors;


    double startTime;

    Timer opmodeTimer = new Timer();

    boolean spitIntake = false;
    boolean isIntaking = false;
    boolean smartShoot = false;
    boolean intakeOverride = false;
    boolean panicShoot = false;
    boolean panicShootingButton = false;
    Pose lastCamPose = new Pose(0,0,0);
    // Offset pose to aim for
    public Pose offsetPose = new Pose(0,0,0);
    double closeDistanceOffset = 0;
    double closeTurretOffset = 0;
    double farDistanceOffset = 0;
    double farTurretOffset = 0;
    //This is an array of the 3 special colors of the games MOTIF
    public BallColor[] motif = null;
    int thingies = 0;
    public double previousAngleToGoalVelocity;
    public double angleToGoalAcceleration;
    Pose goal;
    public Pose shootFromPose = null;
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
        climber = new Climber(hardwareMap);
        intake = new Intake(hardwareMap);
        vision = new Vision(hardwareMap, robotSide);
        indexer = new Indexer(hardwareMap, ballsInIndexer, startIndexerTicks);
        turret = new Turret(hardwareMap, startTurretTicks, robotSide);
        drivetrain = new Drivetrain(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shotCalculator = new ShotCalculator();
        hoodAngleCompensation = new HoodAngleCompensation();
        voltageSensors = hardwareMap.voltageSensor;


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
        lastTimeLoop = System.nanoTime();
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
        return ShapeDetection.isRobotInside(FieldShapes.closeTriangle, curPose) || ShapeDetection.isRobotInside(FieldShapes.farTriangle, curPose);
    }
    public boolean inFarZone() {
        Pose curPose = follower.getPose();
        return ShapeDetection.isRobotInside(FieldShapes.farTriangle, curPose);
    }
    public boolean farBack() {
        return distanceToGoal() > 110;
    }
    public void reZeroAtCorner() {
        offsetPose = new Pose();
        follower.setPose(Common.StartPoses.closeOuter);
    }
    public void setAveragePose() {
        if (pipelineIndex != 0) {
            setPipelineIndex(0);
        }
        if (set) {
            Pose newPose = vision.getRobotPose();
            if (newPose != null) {
                lights.printBigRed();
                follower.setPose(newPose);
            }
        }
    }
    public void clearAveragePose() {
        vision.clearPoseList();
    }
    public void setPipelineIndex(int index) {
        set = false;
        pipelineIndex = index;
        vision.pipelineSwitch(index);
    }
    public int pipelineIndex;
    public boolean set = false;
    public void visionUpdate() {
        if (!set && vision.getPipeline() != pipelineIndex) {
            vision.pipelineSwitch(pipelineIndex);
        } else {
            set = true;
        }
    }

    public Pose getIntakeBallPoseFromCam() {
        return getIntakeBallPoseFromCam(VisionTypes.MeanPose);
    }
    public Pose getIntakeBallPoseFromCam(VisionTypes type) {
        if (pipelineIndex != 2) {
            setPipelineIndex(2);
        }
        if (set){
            List<Vision.DetectedBall> detectedBalls = vision.searchForBalls(getCurrentPose());
            if (!detectedBalls.isEmpty()) {
                for (Vision.DetectedBall ball: detectedBalls) {
                    telemetry.addData("detected balls", ball);
                }
                switch (type) {
                    case Spline:
                    case Predetermined:
                        throw new RuntimeException("spline Pred 0 error alert");
                    case DBScan:
                        return DBSCANLiteClustering.findLargestCluster(detectedBalls, false);
                    case MeanPose:
                        return MeanBallPoses.getIntakeTarget(detectedBalls);
                }
            }
        }
        return null;
    }

    public PathChain getIntakeBallPathFromCam(VisionTypes type) {
        return getIntakeBallPathFromCam(type, false, false);
    }
    public PathChain getIntakeBallPathFromCam(VisionTypes type, boolean limitZone, boolean farZone) {
        if (pipelineIndex != 2) {
            setPipelineIndex(2);
        }
        if (set){
            List<Vision.DetectedBall> detectedBalls = vision.searchForBalls(getCurrentPose(), limitZone, farZone);
            if (!detectedBalls.isEmpty()) {
                for (Vision.DetectedBall ball: detectedBalls) {
                    telemetry.addData("detected balls", ball);
                }
                switch (type) {
                    case DBScan:
                    case MeanPose:
                        throw new RuntimeException("DB Mean error alert");
                    case Spline:
                        Path tempPath = VisionSpline.getSplinePathForVision(detectedBalls, getCurrentPose());
                        if (tempPath != null) {
                            return follower.pathBuilder()
                                    .addPath(tempPath)
                                    .setTangentHeadingInterpolation()
                                    .build();
                        } else {
                            throw new RuntimeException("PathNull");
                        }
                    case Predetermined:
                        throw new RuntimeException("Pred 1 error alert");
//                        return follower.pathBuilder()
//                                .addPath(new BezierCurve(Common.IntakeBallPoses.intakeHuman, Common.IntakeBallPoses.intakeSTunnelAfterHumanControl, Common.IntakeBallPoses.intakeSTunnelAfterHuman))
//                                .setLinearHeadingInterpolation(Common.IntakeBallPoses.intakeHuman, Common.IntakeBallPoses.intakeSTunnelAfterHuman)
//                                .addPath(new BezierCurve(Common.IntakeBallPoses.intakeSTunnelAfterHuman, Common.IntakeBallPoses.intakeFromSTunnel))
//                                .build();

                }
            }
        }
        return null;
    }

/// Returns whether succeeded to Queue te right ball
    public boolean ImmediatelyQueueNextBallOnRampToMatchMotif(){
        List<BallColor> rampBalls = vision.GetBallsOnRamp(getCurrentPose());
        int num = rampBalls.size();
        if (num >= 9){
            return  false;
        }
        int nextIndex = num % 3; //No need to add 1 because num is size while nextIndex is index.
        getMotif(false);
        if (motif != null) {
            qBall(motif[nextIndex]);
            return true;
        }else {
            return false;
        }
    }

    // TURRET**************************************************************************************~

    public double angleToGoalVelocity;
    public boolean usePID = true;
    public void turretUpdate() {
        turret.update(angleToGoalVelocity + follower.getAngularVelocity(), follower.getAngularAcceleration(), getVoltageCompensation(), usePID);
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
    public void reZeroIndexer() {
        indexer.reZeroIndexer();
    }

    public void casualShooterModeOn() {
        turret.setTargetDeg(180);
        shooter.casualModeOn();
    }

    public void setShootFromPose(boolean shootFromPose) {
        if (!shootFromPose) {
            this.shootFromPose = null;
        }
    }

    public void setShootFromPose(Pose shootFromPose) {
        this.shootFromPose = shootFromPose;
    }

    public void prepareShooter() {
        prepareShooter(ShotType.TABLE);
    }
    public void prepareShooter(boolean doSOTF) {
        prepareShooter(ShotType.TABLE, doSOTF);
    }
    public void prepareShooter(ShotType shotType) {
        prepareShooter(shotType, true);
    }
    double rpmOffset;
    double hoodAngleOffset;
    public void setShooterOffset(double rpmOffset, double hoodAngleOffset) {
        this.hoodAngleOffset = hoodAngleOffset;
        this.rpmOffset = rpmOffset;
    }
    double rpmRatio = 1;
    public void prepareShooter(ShotType shotType, boolean useSOTF) {
        prepareShooter(shotType, useSOTF, false);
    }

    public void prepareShooter(ShotType shotType, boolean useSOTF, boolean swapGoal) {
        usePID = true;
        ShotContext ctx = new ShotContext();

        if (shootFromPose == null) {
            ctx.robotPose = follower.getCenterOfShooterPose();
        } else {
            ctx.robotPose = shootFromPose;
        }
        Pose goalPose;
        if (smartShoot) {
            if (!swapGoal) {
                if (robotSide == RobotSide.Blue) {
                    goalPose = Constants.Vision.blueGoalSort;
                } else {
                    goalPose = Constants.Vision.redGoalSort;
                }
            } else {
                if (robotSide == RobotSide.Blue) {
                    goalPose = Constants.Vision.redGoalSort;
                } else {
                    goalPose = Constants.Vision.blueGoalSort;
                }
            }
        } else {
            if (!swapGoal) {
                if (robotSide == RobotSide.Blue) {
                    goalPose = shotType == ShotType.PHYSICAL ? Constants.Vision.blueGoalPhysics : Constants.Vision.blueGoal;
                } else {
                    goalPose = shotType == ShotType.PHYSICAL ? Constants.Vision.redGoalPhysics : Constants.Vision.redGoal;
                }
            } else {
                if (robotSide == RobotSide.Blue) {
                    goalPose = shotType == ShotType.PHYSICAL ? Constants.Vision.redGoalPhysics : Constants.Vision.redGoal;
                } else {
                    goalPose = shotType == ShotType.PHYSICAL ? Constants.Vision.blueGoalPhysics : Constants.Vision.blueGoal;
                }
            }
        }
        ctx.goalPose = goalPose.plus(offsetPose);
        ctx.velocity = follower.getVelocity();
        ctx.acceleration = follower.getAcceleration();
        ctx.side = robotSide;
        ctx.rpmRatio = rpmRatio;
        if (lastShape == FieldShapes.closeTriangle) {
            ctx.distanceOffset = closeDistanceOffset;
        } else {
            ctx.distanceOffset = farDistanceOffset;
        }

        ShotSolution s = shotCalculator.solveShot(ctx, shotType, useSOTF);

        // expose turret feedforward
        angleToGoalVelocity = s.turretVel;

        if (lastShape == FieldShapes.closeTriangle) {
            turret.setTargetRad(s.turretAngleRad + Math.toRadians(closeTurretOffset));
        } else {
            turret.setTargetRad(s.turretAngleRad + Math.toRadians(farTurretOffset));
        }


        shooter.setTargetRPM(s.rpm + rpmOffset);

        shooter.setHoodDeg(s.hoodDeg + hoodAngleOffset);
    }

    double OTHER_ZONE_MULT = 0.5;

    public void shooterTrim(boolean up, boolean down, boolean left, boolean right, boolean reset) {
        if (up) {
            if (lastShape == FieldShapes.closeTriangle) {
                closeDistanceOffset += 1.4;
                farDistanceOffset += 1.4 * OTHER_ZONE_MULT;
            } else {
                farDistanceOffset += 1.4;
                closeDistanceOffset += 1.4 * OTHER_ZONE_MULT;
            }
        }
        if (down) {
            if (lastShape == FieldShapes.closeTriangle) {
                closeDistanceOffset -= 1.4;
                farDistanceOffset -= 1.4 * OTHER_ZONE_MULT;
            } else {
                farDistanceOffset -= 1.4;
                closeDistanceOffset -= 1.4 * OTHER_ZONE_MULT;
            }
        }
        if (left) {
            if (lastShape == FieldShapes.closeTriangle) {
                closeTurretOffset += 1;
                farTurretOffset += 1 * OTHER_ZONE_MULT * 2;
            } else {
                farTurretOffset += 1;
                closeTurretOffset += 1 * OTHER_ZONE_MULT * 2;
            }
        }
        if (right) {
            if (lastShape == FieldShapes.closeTriangle) {
                closeTurretOffset -= 1;
                farTurretOffset -= 1 * OTHER_ZONE_MULT;
            } else {
                farTurretOffset -= 1;
                closeTurretOffset -= 1 * OTHER_ZONE_MULT;
            }
        }
        if (reset) {
            reZeroAtCorner();
            closeDistanceOffset = 0;
            closeTurretOffset = 0;
            farDistanceOffset = 0;
            farTurretOffset = 0;
        }
    }

    FieldShapes lastShape = FieldShapes.closeTriangle;
    public void shooterUpdate() {
        if (ShapeDetection.isRobotInside(FieldShapes.closeTriangle, getCurrentPose())) {
            lastShape = FieldShapes.closeTriangle;
        } else if (ShapeDetection.isRobotInside(FieldShapes.farTriangle, getCurrentPose())) {
            lastShape = FieldShapes.farTriangle;
        }
        shooter.update(getVoltageCompensation(), panicShoot, panicShootingButton);
    }

    public void autoSetCurrentPose() {
        lastCamPose = vision.getRobotPose();
        if (lastCamPose != null) {
            Pose addPose = new Pose();
            switch (robotSide) {
                case Red:
                    addPose = new Pose(-5, 3);
                    break;
                case Blue:
                    addPose = new Pose(-5, -3);
                    break;
            }
            follower.setPose(lastCamPose.plus(addPose));
        }
    }
    public Pose getCurrentPose() {
        return follower.getPose();
    }


    // INTAKE SYSTEM*******************************************************************************~

    public void setIntakeOverride(boolean set) {
        intakeOverride = set;
    }
    public boolean isIntakeOverride() {
        return intakeOverride;
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
            intake.setIntakeMotorPower(0);
            return;
        }

        boolean dontAllowIntaking = indexer.shooting && intake.isBeamBroken();
        intake.update(spitIntake || isIndexerUnjamming(), isIntaking, indexer.shooting, indexer.doSpit(), indexer.allowIntaking() && !dontAllowIntaking, indexer.isPlugged(), intakeOverride);
    }


    // SPINDEXER***********************************************************************************~

    public void doSmartShoot() {
        doSmartShoot(true);
    }
    public void doSmartShoot(boolean set) {
        if (smartShoot != set) {
            smartShoot = set;
            indexer.emptyQueue();
        }
    }
    public void isIntaking(boolean isIntaking) {
        this.isIntaking = isIntaking;
    }

    public void shootAll() {
        indexer.shootAll();
    }
    public void spindexerUpdate() {
        boolean readyToShoot = panicShoot ? true : readyToShootMotors();
        indexer.update(isIntaking, readyToShoot, smartShoot, farBack(), follower.getPose());
    }
    public void indexerUnjam() {
        indexer.unjam();
    }
    public boolean isIndexerUnjamming() {
        return indexer.unjam;
    }

    public boolean basicallyHas3() {
        return indexer.isPlugged() && intake.isBeamBroken();
    }

    // CLIMB***************************************************************************************~
    public void climb() {
        climber.enableClimb();
        lights.setClimbLights(true);
        intake.climb(true);
    }
    public void storeClimber() {
        climber.disableClimb();
        lights.setClimbLights(false);
        intake.climb(false);
    }
    // LIGHTS**************************************************************************************~
    public void lightsUpdate() {
        lights.setBallColors(indexer.getBallCells(), indexer.getQueuedBalls(), basicallyHas3(), smartShoot);
        lights.update();
    }

    // VOLTAGE COMPENSATION************************************************************************~
    public double getVoltageCompensation() {
        return 13 / getBatteryVoltage();
    }
    double lastTime = System.currentTimeMillis();
    double lastVolt = 0;
    public double getBatteryVoltage() {
        if (System.currentTimeMillis() - lastTime < 300) {
            return lastVolt;
        }
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : voltageSensors) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        lastTime = System.currentTimeMillis();
        lastVolt = result;
        return result;
    }
    // TELE-OP*************************************************************************************~

//    boolean brakeAllowSotf = true;
//    boolean brakeAllowDebounce = true;
    //returns if we want to allow sotf
    public void notBrakeDriveTrain() {
//        brakeAllowSotf = true;
//        brakeAllowDebounce = true;
//        return brakeAllowSotf;
    }
    public void breakDrivetrain() {
        Vector velocity = follower.getVelocity().copy();
        double heading = getCurrentPose().getHeading() + Math.toRadians(90);

        velocity.rotateVector(-heading);

        double speed = velocity.getMagnitude();

        // ----- TRANSLATIONAL PID -----
        Constants.Drivetrain.stopPID.updateError(-speed);
        double output = Constants.Drivetrain.stopPID.run();

        Vector brake;
        if (speed < 0.01) {
            brake = new Vector(0, 0);
        } else {
            brake = velocity.normalize().times(output);
        }

        double strafe = -brake.getXComponent();
        double forward = -brake.getYComponent();

        // ----- TURN PID -----
        double headingVel = follower.getAngularVelocity();
        Constants.Drivetrain.turnPID.updateError(-headingVel);
        double turn = -Constants.Drivetrain.turnPID.run();

        // ----- SCALING -----
        double minVel = 20;
        double maxVel = 50;

        double minVelPow = 1;
        double maxVelPow = 0.17;

        double t = (speed - minVel) / (maxVel - minVel);
        t = Math.max(0.0, Math.min(1.0, t));

        double scale = 1.0 - t * (minVelPow - maxVelPow);

        double scaleFront = 1.0;
        double scaleBack = 1.0;

        if (Math.abs(velocity.getYComponent()) > Math.abs(velocity.getXComponent())) {
            if (velocity.getYComponent() > 0) {
                scaleBack = scale;
            } else {
                scaleFront = scale;
            }
        }

        // ----- APPLY POWERS -----
        double leftFrontPower  = scaleFront * (forward + strafe + turn);
        double leftBackPower   = scaleBack  * (forward - strafe + turn);
        double rightFrontPower = scaleFront * (forward - strafe - turn);
        double rightBackPower  = scaleBack  * (forward + strafe - turn);

        drivetrain.setLeftFrontPower(leftFrontPower);
        drivetrain.setLeftBackPower(leftBackPower);
        drivetrain.setRightFrontPower(rightFrontPower);
        drivetrain.setRightBackPower(rightBackPower);
//        if (speed < 0.1) {
//            brakeAllowDebounce = false;
//            brakeAllowSotf = true;
//        } else if (brakeAllowDebounce) {
//            brakeAllowSotf = false;
//        }
//        return !brakeAllowSotf;
    }

    // Does not set panicShootingButton if panic shoot is false
    public void setPanicShoot(boolean panicShoot, boolean panicShootingButton) {
        this.panicShoot = panicShoot;
        this.panicShootingButton = panicShootingButton;
        if (panicShoot) {
            setShootFromPose(Common.ShootPoses.panicShoot);
        } else {
            setShootFromPose(null);
            this.panicShootingButton = false;
        }
    }

    // AUTONOMOUS**********************************************************************************~

    public void resetTimers() {
        opmodeTimer.resetTimer();
    }

    public double getOpmodeTimeSeconds() {
        return opmodeTimer.getElapsedTimeSeconds();
    }

    public boolean movingSlowEnoughToShoot(boolean close) {
        if (close) {
            return follower.getVelocity().getMagnitude() < Common.Timings.shootVelocityClose;
        } else {
            return follower.getVelocity().getMagnitude() < Common.Timings.shootVelocityFar && follower.getAcceleration().getMagnitude() < Common.Timings.shootAccelerationFar;
        }
    }



    // SYSTEM**************************************************************************************~
    double lastTimeLoop;
    public void start() {
        lastTimeLoop = System.nanoTime();
        resetTimers();
        indexer.start();
        setPipelineIndex(1);
    }
    public void update() {
        update(false);
    }
    double previousHoodAngle;
    double previousTime;
    double maxHoodAngleChange;
    public void update(boolean debug) {

        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        shooterUpdate();
        intakeUpdate();
        spindexerUpdate();
        turretUpdate();
        follower.update();
        visionUpdate();
        lightsUpdate();

        if (debug) {
            debug();
        }

        panelsTelemetry.update();
    }

    public void debug() {

        Drawing.drawShapesDebug(this.follower);
        panelsTelemetry.update();

        /*
        long start = System.currentTimeMillis();
        shooterUpdate();
        long shooter = System.currentTimeMillis();
        intakeUpdate();
        long intake = System.currentTimeMillis();
        spindexerUpdate();
        long spindexer = System.currentTimeMillis();
        turretUpdate();
        long turret = System.currentTimeMillis();
        follower.update();
        long followa = System.currentTimeMillis();
        visionUpdate();
        lightsUpdate();
        long lights = System.currentTimeMillis();

        panelsTelemetry.addData("shooter", -(start - shooter));
        panelsTelemetry.addData("intake", -(shooter - intake));
        panelsTelemetry.addData("spindexer", -(intake - spindexer));
        panelsTelemetry.addData("turret", -(spindexer - turret));
        panelsTelemetry.addData("follower", -(turret - followa));
        panelsTelemetry.addData("lights", -(followa - lights));
        panelsTelemetry.addData("all", (lights - lastTime));
        panelsTelemetry.addData("hue", indexer.getHue());
         */

        panelsTelemetry.addData("turret pos", turret.getAngle());
        panelsTelemetry.addData("turret target", turret.turretPID.getTargetPosition());
        panelsTelemetry.addData("turret error", turret.turretPID.getError());
        panelsTelemetry.addData("turret velocity", turret.getVelocity());
        panelsTelemetry.addData("turret power", clamp(turret.turretPID.run(), -1, 1) * 360);
        panelsTelemetry.addData("turret Acceleration", follower.getAngularAcceleration());


        panelsTelemetry.addData("RPM", shooter.getRPM());
        panelsTelemetry.addData("RPM target", shooter.shooterPID.getTargetPosition());
        panelsTelemetry.addData("Hood angle", shooter.getHoodPosDeg());
        panelsTelemetry.addData("distance to goal", distanceToGoal());
        panelsTelemetry.addData("RPM error", shooter.shooterPID.getError());


//        panelsTelemetry.addData("Hood Angle correction", deltaDeg);
//        panelsTelemetry.addData("Shoot overPower", shooter.shooterPID.run() >= 1 ? 1000 : 0);
//        panelsTelemetry.addData("Shoot power", shooter.shooterPID.run());
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
        telemetry.addData("indexer pos", indexer.getEncoderPercentage());

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
        telemetry.addData("indexer col", indexer.getColorSlow());

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
