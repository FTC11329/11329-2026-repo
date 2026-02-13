package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.RobotSide;
import org.firstinspires.ftc.teamcode.util.ShootOnTheFly.HoodAngleCompensation;
import org.firstinspires.ftc.teamcode.util.ShootOnTheFly.ShotCalculator;
import org.firstinspires.ftc.teamcode.util.ShootOnTheFly.ShotContext;
import org.firstinspires.ftc.teamcode.util.ShootOnTheFly.ShotSolution;
import org.firstinspires.ftc.teamcode.util.ShootOnTheFly.ShotType;
// pass in all variables
// create a function to run during reggular loops
public class ShooterLogicThread implements Runnable {

    volatile Turret turret;
    volatile Shooter shooter;
    volatile Follower follower;

    ShotCalculator shotCalculator;
    HoodAngleCompensation hoodAngleCompensation;

    RobotSide robotSide;
    Pose goal;
    Pose offsetPose = new Pose();
    double rpmRatio = 1;
    double angleToGoalVelocity = 0;
    double angleToGoalAcceleration = 0;

    boolean running = false;
    boolean requestStop = false;

    public ShooterLogicThread(Pose goal, Turret turret, Shooter shooter, Follower follower, RobotSide robotSide) {
        this.goal = goal;
        this.turret = turret;
        this.shooter = shooter;
        this.follower = follower;
        this.robotSide = robotSide;

        shotCalculator = new ShotCalculator();
        hoodAngleCompensation = new HoodAngleCompensation();
    }

    public boolean isAlive() {
        return running;
    }

    public void start() {
        start(true);
    }

    public void start(boolean set) {
        running = set;
    }

    public void setOffsetPose(Pose offsetPose) {
        this.offsetPose = offsetPose;
    }

    public void requestStop() {
        this.requestStop = true;
    }



    private double distanceToGoal() {
        Pose curPose = follower.getPose();
        double deltaX = goal.getX() - curPose.getX();
        double deltaY = goal.getY() - curPose.getY();

        return Math.hypot(deltaX, deltaY);
    }

    private boolean farBack() {
        return distanceToGoal() > 110;
    }

    private void prepareShooter(ShotType shotType) {
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

        double deltaDeg;
        if (shotType == ShotType.TABLE && !farBack()) {
            deltaDeg = hoodAngleCompensation.hoodAngleCompensation(s.rpm, shooter.getRPM(), s.hoodDeg);
            rpmRatio = hoodAngleCompensation.getRpmRatio();
        } else {
            rpmRatio = 1;
            deltaDeg = 0;
        }

        shooter.setHoodDeg(s.hoodDeg + deltaDeg);
    }

    public void updateSubSystems() {
        follower.update();
        shooter.update();
        turret.update(angleToGoalVelocity + follower.getAngularVelocity(), angleToGoalAcceleration);
    }


    @Override
    public void run() {
        while (!requestStop) {
            if (isAlive()) {
                updateSubSystems();
                prepareShooter(ShotType.TABLE);
            } else {
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        }
    }
}
