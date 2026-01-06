package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.control.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Configurable
public class SuperDuperPID {

    /* -------------------- Tunables -------------------- */
    // todo: figure out how to add @Configurable so we can tune without having to restart the bot
    // Motion profiling

    private static double MAX_ACHIEVABLE_VELOCITY = 7500;
    private static double DECEL_MAX = 1000.0;  // ticks/sec^2

    // Feedforward
    private static double frictionBlendTicks = 20;
    private static double kV = 1.0 / MAX_ACHIEVABLE_VELOCITY;
    private static double kF_CW = 0.054;
    private static double kF_CCW = 0.065;
    // PID
    private static double kP = 0.00008; // .00005 < x < .00015
    private static double kI = 0.0;
    private static double kD = 0.000006;


    // Integral clamp
    private static double I_MAX = 0.1;
    //derivative filtering
    private double filteredVelocity = 0;
    private static double VEL_ALPHA = 0.15; // 0–1, lower = smoother


    /* -------------------- State -------------------- */

    private double position;
    private double prevPosition;
    private double velocity;

    private double targetPosition = 0;
    private double error;
    private double dt;

    private double errorIntegral;
    private double errorDerivative;

    private double desiredVelocity;

    private long prevTimeNs;
    private boolean stuck;

    /* -------------------- Constructor -------------------- */

    public SuperDuperPID() {
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

        double rawVelocity = (position - prevPosition) / dt;
        filteredVelocity += VEL_ALPHA * (rawVelocity - filteredVelocity);
        velocity = filteredVelocity;
        error = targetPosition - position;

        /* ---------- Motion profile (braking-limited) ---------- */

        // Maximum velocity that still allows stopping at target
        double maxVelForStop =
                Math.sqrt(2.0 * DECEL_MAX * Math.abs(error));

        // Desired velocity is always toward the target,
        // but capped by stopping distance
        desiredVelocity =
                Math.signum(error) *
                        Math.min(MAX_ACHIEVABLE_VELOCITY, maxVelForStop);

        /* ---------- PID terms ---------- */

        errorIntegral += error * dt;
        errorIntegral = clamp(errorIntegral, -I_MAX, I_MAX);

        errorDerivative = -velocity; // velocity damping
    }

    /* -------------------- Output -------------------- */

    public double run() {

        if (Math.abs(error) <= Constants.Indexer.indexerTolerance) {
            return 0;
        }

        /* ---------- Feedforward ---------- */

        double kF = (error > 0) ? kF_CCW : kF_CW;
        double staticFF = kF * Math.tanh(error / frictionBlendTicks);

        /* ---------- PID ---------- */

        double output =
                kP * error +
                kI * errorIntegral +
                kD * errorDerivative +
                staticFF +
                desiredVelocity * kV;

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
        prevTimeNs = System.nanoTime();
    }

    /* -------------------- Telemetry -------------------- */

    public double getError() { return error; }
    public double getVelocity() { return velocity; }
    public double getDesiredVelocity() { return desiredVelocity; }

    public double getPOutput() { return kP * error; }
    public double getIOutput() { return kI * errorIntegral; }
    public double getDOutput() { return kD * errorDerivative; }
    public double getDeltaTime() { return dt;}
    public double getTargetPosition() { return targetPosition;}

    public boolean isStuck() { return stuck; }

    /* -------------------- Utils -------------------- */

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
