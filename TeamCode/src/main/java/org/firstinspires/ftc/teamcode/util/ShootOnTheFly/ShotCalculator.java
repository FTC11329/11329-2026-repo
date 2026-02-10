package org.firstinspires.ftc.teamcode.util.ShootOnTheFly;

import static java.lang.Math.PI;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.math.Vector;
import org.firstinspires.ftc.teamcode.util.RobotSide;
import org.firstinspires.ftc.teamcode.util.shooterInterpolation.ShooterState;
import org.firstinspires.ftc.teamcode.util.shooterInterpolation.ShooterTestValues;
import org.firstinspires.ftc.teamcode.util.shooterInterpolation.ShooterValuesParent;

public class ShotCalculator {
    public ShotSolution solveShot(ShotContext ctx, ShotType mode) {
        switch (mode) {
            case TABLE:
                return solveTableShot(ctx);
            case PHYSICAL:
                return solveIdealShot(ctx);
        }
        return null;
    }
    public ShotCalculator() {
        shooterTestValues = new ShooterTestValues();
        lastTOFtime = System.nanoTime();
    }
    double lastTOFtime;
    ShooterTestValues shooterTestValues;
    double previousTOF;
    private ShotSolution solveTableShot(ShotContext ctx) {

        ShooterValuesParent stv = shooterTestValues;
        //todo: calculate RPM ratio in here

        double timeInFlight =
                (1 / ctx.rpmRatio) *
                        stv.get(ctx.robotPose.distanceFrom(ctx.goalPose)).timeInFlight;

        Pose futrGoal = ctx.goalPose.plusVector(ctx.velocity, -timeInFlight);

        timeInFlight =
                (1 / ctx.rpmRatio) *
                        stv.get(ctx.robotPose.distanceFrom(futrGoal)).timeInFlight;

        futrGoal = ctx.goalPose.plusVector(ctx.velocity, -timeInFlight);

        double dt = (System.nanoTime() - lastTOFtime) / 1e9;
        double rateOfChangeOfTOF = (timeInFlight - previousTOF) / dt;
        lastTOFtime = System.nanoTime();
        previousTOF = timeInFlight;

        Pose goalOffset = (ctx.side == RobotSide.Blue)
                ? Constants.Vision.blueGoalAimOffset
                : Constants.Vision.redGoalAimOffset;

        double dX = futrGoal.getX() + goalOffset.getX() - ctx.robotPose.getX();
        double dY = futrGoal.getY() + goalOffset.getY() - ctx.robotPose.getY();

        double angleToGoalRad = Math.atan2(dY, dX);

        Vector virtualGoalVelocity =
                ctx.acceleration.times(-timeInFlight)
                        .minus(ctx.velocity.times(rateOfChangeOfTOF));

        Vector correctedVelocity = ctx.velocity.minus(virtualGoalVelocity);

        double r2 = dX*dX + dY*dY;

        ShotSolution shotSolution = new ShotSolution();

        shotSolution.turretAngleRad = angleToGoalRad - ctx.robotPose.getHeading();

        shotSolution.turretVel =
                (dX * correctedVelocity.getYComponent()
                        - dY * correctedVelocity.getXComponent()) / r2;

        shotSolution.turretAccel =
                (dX * ctx.acceleration.getYComponent()
                        - dY * ctx.acceleration.getXComponent()) / r2
                        - 2 * shotSolution.turretVel *
                        ((dX * correctedVelocity.getXComponent()
                                + dY * correctedVelocity.getYComponent()) / r2);

        ShooterState params = stv.get(ctx.robotPose.distanceFrom(futrGoal));

        shotSolution.rpm = params.rpm;
        shotSolution.hoodDeg = params.hoodAngle;
        shotSolution.tof = timeInFlight;
        shotSolution.futureGoal = futrGoal;

        return shotSolution;
    }
    private ShotSolution solveIdealShot(ShotContext ctx) {

        ShotSolution shotSolution = new ShotSolution();

        double g = 386.09;
        double height = Constants.Shooter.entryHeight;
        double entryAngle = Constants.Shooter.entryAngle;

        double deltaX = ctx.goalPose.getX() - ctx.robotPose.getX();
        double deltaY = ctx.goalPose.getY() - ctx.robotPose.getY();
        double angleToGoal = Math.atan2(deltaY, deltaX);

        double distance = Math.hypot(deltaX, deltaY);

        double hoodAngle = Math.atan(2 * height / distance - Math.tan(entryAngle));
        double flywheelSpeed =
                Math.sqrt(g * distance * distance /
                        (2 * Math.pow(Math.cos(hoodAngle), 2) *
                                (distance * Math.tan(hoodAngle) - height)));

        double coordinateTheta = ctx.velocity.getTheta() - angleToGoal;
        double parallel = -Math.cos(coordinateTheta) * ctx.velocity.getMagnitude();
        double perp = Math.sin(coordinateTheta) * ctx.velocity.getMagnitude();

        double vz = flywheelSpeed * Math.sin(hoodAngle);
        double time = distance / (flywheelSpeed * Math.cos(hoodAngle));

        double ivr = distance / time + parallel;
        double nvr = Math.hypot(ivr, perp);
        double ndr = nvr * time;

        hoodAngle = Math.atan(vz / nvr);

        flywheelSpeed =
                Math.sqrt(g * ndr * ndr /
                        (2 * Math.pow(Math.cos(hoodAngle), 2) *
                                (distance * Math.tan(hoodAngle) - height)));

        double turretVelocityOffset = Math.atan2(perp, ivr);

        Pose futrGoal = ctx.goalPose.plusVector(ctx.velocity, -time);

        double dX = futrGoal.getX() - ctx.robotPose.getX();
        double dY = futrGoal.getY() - ctx.robotPose.getY();
        double r2 = dX*dX + dY*dY;

        shotSolution.turretAngleRad = angleToGoal - turretVelocityOffset - ctx.robotPose.getHeading();
        shotSolution.turretVel =
                (dX * ctx.velocity.getYComponent() - dY * ctx.velocity.getXComponent()) / r2;

        shotSolution.turretAccel =
                (dX * ctx.acceleration.getYComponent() - dY * ctx.acceleration.getXComponent()) / r2
                        - 2 * shotSolution.turretVel *
                        ((dX * ctx.velocity.getXComponent() + dY * ctx.velocity.getYComponent()) / r2);

        shotSolution.hoodDeg = Math.toDegrees(Math.PI/2 - hoodAngle);
        shotSolution.rpm = velocityToRPM(flywheelSpeed);
        shotSolution.tof = time;
        shotSolution.futureGoal = futrGoal;

        return shotSolution;
    }
    private static double velocityToRPM(double exitVelocity) {
        // exitVelocity in in/s
        double wheelDiameter = 1.75;

        double surfaceVelocity = PI * wheelDiameter;

        double rotationsPerSec = exitVelocity / surfaceVelocity;

        return rotationsPerSec * 60 + 140;
    }
}
