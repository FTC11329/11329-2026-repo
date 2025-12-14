package org.firstinspires.ftc.teamcode.pedroPathing.geometry;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Curve;
import org.firstinspires.ftc.teamcode.pedroPathing.math.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.math.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathConstraints;
import org.firstinspires.ftc.teamcode.util.CubicSpline1D;
import org.firstinspires.ftc.teamcode.util.PathSpline;

import static org.firstinspires.ftc.teamcode.pedroPathing.math.AbstractBijectiveMap.NumericBijectiveMap;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class SplineCurve implements Curve {
    private final List<Pose> controlPoints;
    private final List<Pose> controlVelocities;
    private final List<Double> times;
    private final ElapsedTime timer;

    private double totalTime;
    private PathConstraints pathConstraints;

    public PathSpline spline;

    private boolean initialized = false;

    private final int APPROXIMATION_STEPS = 1000;
    private final int DASHBOARD_DRAWING_APPROXIMATION_STEPS = 100;

    private double[][] panelsDrawingPoints;

    private final NumericBijectiveMap completionMap = new NumericBijectiveMap();
    private double length = 0;

    public SplineCurve(List<Pose> points, List<Pose> velocities, List<Double> times, PathConstraints constraints) {
        if (points.size() < 2) {
            throw new RuntimeException("SplineCurve requires at least 2 points.");
        }
        this.controlPoints = new ArrayList<>(points);
        this.controlVelocities = new ArrayList<>(velocities);
        this.times = new ArrayList<>(times);
        this.pathConstraints = constraints;
        this.timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        totalTime = times.get(times.size() - 1);
        initialize();
    }

    public SplineCurve(List<Pose> points, List<Pose> velocities, List<Double> times) {
        this(points, velocities, times, PathConstraints.defaultConstraints);
    }

    // Build cubic splines for x, y, and heading
    public void initialize() {
        if (initialized) return;

        int n = controlPoints.size();
        double[] t = new double[n];
        for (int i = 0; i < n; i++) t[i] = times.get(i) / totalTime;

        double[] xs = new double[n];
        double[] ys = new double[n];
        double[] xv = new double[n];
        double[] yv = new double[n];
        double[] headings = new double[n];
        double[] dheadings = new double[n];

        for (int i = 0; i < n; i++) {
            xs[i] = controlPoints.get(i).getX();
            ys[i] = controlPoints.get(i).getY();
            xs[i] = controlVelocities.get(i).getX();
            ys[i] = controlVelocities.get(i).getY();
            headings[i] = controlPoints.get(i).getHeading();
            dheadings[i] = controlVelocities.get(i).getHeading();
        }

        spline = new PathSpline();
        spline.xSpline = new CubicSpline1D(t, xs, xv);
        spline.ySpline = new CubicSpline1D(t, ys, yv);
        spline.headingSpline = new CubicSpline1D(t, headings, dheadings);

        length = approximateLength();
        initializePanelsDrawingPoints();

        initialized = true;
    }

    private void initializePanelsDrawingPoints() {
        panelsDrawingPoints = new double[2][DASHBOARD_DRAWING_APPROXIMATION_STEPS + 1];
        for (int i = 0; i <= DASHBOARD_DRAWING_APPROXIMATION_STEPS; i++) {
            double u = i / (double) DASHBOARD_DRAWING_APPROXIMATION_STEPS;
            Pose p = getPose(u);
            panelsDrawingPoints[0][i] = p.getX();
            panelsDrawingPoints[1][i] = p.getY();
        }
    }

    public double[][] getPanelsDrawingPoints() {
        return panelsDrawingPoints;
    }

    public Pose getPose(double t) {
        return spline.evaluate(t);
    }

    public Vector getDerivative(double t) {
        return spline.velocity(t);
    }

    public Vector getSecondDerivative(double t) {
        return spline.acceleration(t);
    }

    public double getCurvature(double t) {
        return spline.curvature(t);
    }

    public Vector getNormalVector(double t) {
        double a = getDerivative(t).getTheta();
        double b = getDerivative(t + 0.0001).getTheta();
        return new Vector(1, b - a);
    }

    // Perform numeric length approximation
    public double approximateLength() {
        Pose prev = getPose(0);
        double total = 0;

        for (int i = 1; i <= APPROXIMATION_STEPS; i++) {
            double t = i / (double) APPROXIMATION_STEPS;
            Pose cur = getPose(t);
            total += prev.distanceFrom(cur);
            prev = cur;

            completionMap.put(t, total);
        }
        return total;
    }

    public double length() {
        return length;
    }

    public ArrayList<Pose> getControlPoints() {
        return new ArrayList<>(controlPoints);
    }

    public Pose getFirstControlPoint() { return controlPoints.get(0); }
    public Pose getSecondControlPoint() { return controlPoints.get(1); }
    public Pose getSecondToLastControlPoint() { return controlPoints.get(controlPoints.size()-2); }
    public Pose getLastControlPoint() { return controlPoints.get(controlPoints.size()-1); }

    public PathConstraints getPathConstraints() {
        return pathConstraints;
    }

    public void setPathConstraints(PathConstraints c) { this.pathConstraints = c; }

    public boolean atParametricEnd(double t) {
        return t >= pathConstraints.getTValueConstraint();
    }

    public String pathType() { return "spline"; }

    public SplineCurve getReversed() {
        ArrayList<Pose> reversedControl = new ArrayList<>(controlPoints);
        ArrayList<Pose> reversedVelocities = new ArrayList<>(controlVelocities);
        ArrayList<Double> reversedTimes = new ArrayList<>(times);
        Collections.reverse(reversedControl);
        Collections.reverse(reversedVelocities);
        Collections.reverse(reversedTimes);
        return new SplineCurve(reversedControl, reversedVelocities, reversedTimes, pathConstraints);
    }

    public double getClosestPoint(Pose pose, int searchLimit, double initialGuess) {
        return timer.time() / totalTime;
    }

    public double getClosestPoint(Pose pose, double guess) {
        return getClosestPoint(pose, pathConstraints.getBEZIER_CURVE_SEARCH_LIMIT(), guess);
    }

    public double getPathCompletion(double t) {
        if (length == 0) return 0;
        return completionMap.interpolateKey(t) / length;
    }

    public double getT(double pathCompletion) {
        return completionMap.interpolateValue(pathCompletion);
    }

    public boolean isInitialized() { return initialized; }
}
