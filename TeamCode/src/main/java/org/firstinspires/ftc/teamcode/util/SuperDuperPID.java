package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.Constants.Indexer.kV;
import static org.firstinspires.ftc.teamcode.pedroPathing.math.MathFunctions.clamp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.control.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class SuperDuperPID {

    /* -------------------- Tunables -------------------- */

    // Stuck detection
    private static final double STUCK_ERROR_TICKS = 50;
    private static final double STUCK_VEL_EPS = 5;          // ticks/sec
    private static final long   STUCK_TIME_NS = 40_000_000; // 40 ms
    private static final double UNSTICK_POWER = 0.4;

    // Motion profiling
    private static final double PROFILE_FADE_TICKS = 75;
    private double Vmax = 0.4;
    private double Amax = 0.05;

    // Feedforward
    private double frictionBlendTicks = 20;
    private double kV = 1.0;
    private double kF_CW = 0.06;
    private double kF_CCW = 0.07;

    // Integral clamp
    private static final double I_MAX = 0.1;

    /* -------------------- State -------------------- */

    private PIDFCoefficients coeffs;

    private double position;
    private double prevPosition;
    private double velocity;

    private double targetPosition;
    private double error;
    private double dt;

    private double errorIntegral;
    private double errorDerivative;

    private double desiredVelocity;

    private long prevTimeNs;
    private boolean stuck;
    private long stuckStartNs = -1;

    /* -------------------- Constructor -------------------- */

    public SuperDuperPID(PIDFCoefficients coeffs) {
        this.coeffs = coeffs;
        reset();
    }

    /* -------------------- Update -------------------- */

    public void update(double currentPosition) {
        long now = System.nanoTime();
        dt = (now - prevTimeNs) * 1e-9;
        if (dt <= 0) return;
        prevTimeNs = now;

        prevPosition = position;
        position = currentPosition;

        velocity = (position - prevPosition) / dt;
        error = targetPosition - position;

        /* ---------- Motion profile ---------- */

        double dx = error;
        double stopDist = (desiredVelocity * desiredVelocity) / (2 * Amax);

        if (Math.abs(dx) > stopDist) {
            desiredVelocity += Amax * dt * Math.signum(dx);
        } else {
            desiredVelocity -= Amax * dt * Math.signum(desiredVelocity);
        }

        desiredVelocity = clamp(desiredVelocity, -Vmax, Vmax);

        /* ---------- PID terms ---------- */

        errorIntegral += error * dt;
        errorIntegral = clamp(errorIntegral, -I_MAX, I_MAX);

        errorDerivative = -velocity; // velocity damping

        /* ---------- Stuck detection ---------- */

        boolean far = Math.abs(error) > STUCK_ERROR_TICKS;
        boolean slow = Math.abs(velocity) < STUCK_VEL_EPS;

        if (far && slow) {
            if (stuckStartNs < 0) stuckStartNs = now;
            else if (now - stuckStartNs > STUCK_TIME_NS) stuck = true;
        } else {
            stuck = false;
            stuckStartNs = -1;
        }
    }

    /* -------------------- Output -------------------- */

    public double run() {

        if (Math.abs(error) <= Constants.Indexer.indexerTolerance) {
            return 0;
        }

        if (stuck) {
            return UNSTICK_POWER * Math.signum(error);
        }

        /* ---------- Feedforward ---------- */

        double kF = (error > 0) ? kF_CCW : kF_CW;
        double staticFF = kF * Math.tanh(error / frictionBlendTicks);

        double profileWeight = clamp(Math.abs(error) / PROFILE_FADE_TICKS, 0, 1);
        double velocityFF = profileWeight * desiredVelocity * kV;

        /* ---------- PID ---------- */

        double output =
                coeffs.P * error +
                        coeffs.I * errorIntegral +
                        coeffs.D * errorDerivative +
                        staticFF +
                        velocityFF;

        return clamp(output, -1.0, 1.0);
    }

    /* -------------------- Control -------------------- */

    public void setTargetPosition(double target) {
        targetPosition = target;
        errorIntegral = 0;
    }

    public void reset() {
        position = prevPosition = velocity = 0;
        error = errorIntegral = errorDerivative = 0;
        desiredVelocity = 0;
        stuck = false;
        stuckStartNs = -1;
        prevTimeNs = System.nanoTime();
    }

    /* -------------------- Telemetry -------------------- */

    public double getError() { return error; }
    public double getVelocity() { return velocity; }
    public double getDesiredVelocity() { return desiredVelocity; }

    public double getPOutput() { return coeffs.P * error; }
    public double getIOutput() { return coeffs.I * errorIntegral; }
    public double getDOutput() { return coeffs.D * errorDerivative; }
    public double getDeltaTime() { return dt;}
    public double getTargetPosition() { return targetPosition;}

    public boolean isStuck() { return stuck; }

    public PIDFCoefficients getCoefficients() { return coeffs; }

    /* -------------------- Utils -------------------- */

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
