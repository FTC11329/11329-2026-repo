package org.firstinspires.ftc.teamcode.pedroPathing.geometry;

import org.firstinspires.ftc.teamcode.pedroPathing.math.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.math.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathConstraints;

import java.util.ArrayList;

@Deprecated
public class FinetunedBezierLine extends BezierLine {
    private final Pose endPoint;
    private double crossingThreshold;
    private final BezierLine modifiedCurve;
    private final double pathEndTValueConstraint;
    private double unmodifiedSegmentLength;
    private PathConstraints constraints;

    public FinetunedBezierLine(ArrayList<Pose> controlPoints, Pose endPoint, int searchLimit) {
        this(controlPoints, endPoint, searchLimit, PathConstraints.defaultConstraints);
    }

    public FinetunedBezierLine(ArrayList<Pose> controlPoints, Pose endPoint, int searchLimit, PathConstraints constraints) {
        super(controlPoints.get(0), controlPoints.get(1), false);
        this.constraints = constraints;
        this.endPoint = endPoint;
        this.pathEndTValueConstraint = this.constraints.getTValueConstraint();
        crossingThreshold = getClosestPoint(endPoint, searchLimit, 1.0);
        if (crossingThreshold == 0) crossingThreshold += 0.001;

        if (crossingThreshold < pathEndTValueConstraint) {
            modifiedCurve = new BezierLine(controlPoints.get(0), endPoint);
        } else {
            modifiedCurve = new BezierLine(getLastControlPoint(), endPoint);
        }
        modifiedCurve.initialize();

        initialize();
    }

    public FinetunedBezierLine(ArrayList<Pose> controlPoints, Pose endPoint) {
        this(controlPoints, endPoint, PathConstraints.defaultConstraints);
    }

    public FinetunedBezierLine(ArrayList<Pose> controlPoints, Pose endPoint, PathConstraints constraints) {
        super(controlPoints.get(0), controlPoints.get(1), false);
        this.constraints = constraints;
        this.endPoint = endPoint;
        this.pathEndTValueConstraint = this.constraints.getTValueConstraint();
        crossingThreshold = getClosestPoint(endPoint, this.constraints.getBEZIER_CURVE_SEARCH_LIMIT(), 1.0);
        if (crossingThreshold == 0) crossingThreshold += 0.001;

        if (crossingThreshold < pathEndTValueConstraint) {
            modifiedCurve = new BezierLine(controlPoints.get(0), endPoint);
        } else {
            modifiedCurve = new BezierLine(getLastControlPoint(), endPoint);
        }
        modifiedCurve.initialize();

        initialize();
    }

    public Pose getEndPoint() {
        return endPoint;
    }

    /**
     * Sets the first t-value where the endpoint starts taking effect instead of the last control point. This allows the user to control how local their finetuning changes are.
     * @param crossingThreshold the first t-value where the endpoint starts taking effect
     */
    public void setCrossingThreshold(double crossingThreshold) {
        this.crossingThreshold = crossingThreshold;
    }

    private double convertT(double t) {
        return t/crossingThreshold;
    }

    @Override
    public Pose getPose(double t) {
        if (t < crossingThreshold) {
            return super.getPose(t);
        } else if (crossingThreshold < pathEndTValueConstraint) {
            double scale = MathFunctions.scale(t, crossingThreshold, 1.0, 0.0, 1.0);
            return super.getPose(t).linearCombination(modifiedCurve.getPose(t), scale, 1-scale);
        } else {
            double scale = MathFunctions.scale(t, crossingThreshold, 1.0, 0.0, 1.0);
            return modifiedCurve.getPose(scale);
        }
    }

    @Override
    public Vector getDerivative(double t) {
        if (t < crossingThreshold) {
            return super.getDerivative(t);
        } else if (crossingThreshold < pathEndTValueConstraint) {
            double scale = MathFunctions.scale(t, crossingThreshold, 1.0, 0.0, 1.0);
            return super.getDerivative(t).linearCombination(modifiedCurve.getDerivative(t), scale, 1-scale);
        } else {
            double scale = MathFunctions.scale(t, crossingThreshold, 1.0, 0.0, 1.0);
            return modifiedCurve.getDerivative(scale);
        }
    }

    @Override
    public Vector getSecondDerivative(double t) {
        if (t < crossingThreshold) {
            return super.getSecondDerivative(t);
        } else if (crossingThreshold < pathEndTValueConstraint) {
            double scale = MathFunctions.scale(t, crossingThreshold, 1.0, 0.0, 1.0);
            return super.getSecondDerivative(t).linearCombination(modifiedCurve.getSecondDerivative(t), scale, 1-scale);
        } else {
            double scale = MathFunctions.scale(t, crossingThreshold, 1.0, 0.0, 1.0);
            return modifiedCurve.getSecondDerivative(scale);
        }
    }

    @Override
    public boolean atParametricEnd(double t) {
        if (t < crossingThreshold) {
            return super.atParametricEnd(t);
        }

        double tChange = t - crossingThreshold;
        return unmodifiedSegmentLength + tChange * (length() - unmodifiedSegmentLength) >= constraints.getTValueConstraint();
    }

    @Override
    public double approximateLength() {
        super.approximateLength();

        if (crossingThreshold < pathEndTValueConstraint) return super.length() * crossingThreshold + (1 - crossingThreshold) * (super.length()
                + modifiedCurve.length());

        return super.length() + modifiedCurve.length();
    }

    @Override
    public double getPathCompletion(double t) {
        if (t < crossingThreshold) return super.getPathCompletion(t);
        return unmodifiedSegmentLength + (t - crossingThreshold) * (length() - unmodifiedSegmentLength);
    }

    @Override
    public BezierLine getReversed() {
        BezierLine reversed = new BezierLine(endPoint, getFirstControlPoint());
        reversed.initialize();
        return reversed;
    }
}
