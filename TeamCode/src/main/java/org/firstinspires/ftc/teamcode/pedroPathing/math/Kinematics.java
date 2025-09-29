package org.firstinspires.ftc.teamcode.pedroPathing.math;

/**
 * Implementation of kinematic equations that work with both positive and negative "distances."
 * Used for decelerating the robot.
 *
 * <pre>
 * vf² = vi² + 2 * a * d
 * </pre>
 * Where:<br>
 * <ul>
 *   <li><b>vf</b> = final velocity</li>
 *   <li><b>vi</b> = initial velocity</li>
 *   <li><b>a</b> = acceleration (or deceleration; always negative)</li>
 *   <li><b>d</b> = distance (equations arranged in code for positive and negative)</li>
 * </ul>
 *
 * @author Jacob Ophoven - 18535 Frozen Code
 * @version 1.1.0, 5/6/2025
 */
public final class Kinematics {
    /**
     * Get the final velocity at a given distance with the deceleration.
     *
     * <pre>
     * vf² = vi² + 2 * a * d
     * Solve for vi
     * vi = sqrt(vf² - 2 * a * d)
     * Set vf to 0 to stop
     * vi = sqrt(0 - 2 * a * d)
     * vi = sqrt(-2 * a * d)
     * </pre>
     */
    public static double getVelocityToStopWithDeceleration(
            double directionalDistance,
            double deceleration
    ) {
        return Math.signum(directionalDistance)
                * Math.sqrt(Math.abs(-2 * deceleration * directionalDistance));
    }

    /**
     * Calculate the velocity the robot would be going after traveling the given distance
     * as it decelerates.
     *
     * <pre>
     * vf² = vi² + 2 * a * d
     * Solve for vf
     * vf = sqrt(vi² + 2 * a * d)
     * </pre>
     */
    public static double getFinalVelocityAtDistance(
            double currentVelocity,
            double deceleration,
            double directionalDistance
    ) {
        return Math.signum(directionalDistance) * Math.sqrt(Math.abs(currentVelocity * currentVelocity
                + 2 * deceleration * Math.abs(directionalDistance)));
    }

    /**
     * Predict the next loop's velocity.
     *
     * <pre>
     * v_next = 2 * v_current - v_previous
     * </pre>
     */
    public static double predictNextLoopVelocity(double currentVelocity, double previousVelocity) {
        return 2 * currentVelocity - previousVelocity;
    }

    /**
     * Calculate the distance needed to reach a velocity with the deceleration.
     *
     * <pre>
     * vf² = vi² + 2 * a * d
     * Solve for d
     * d = -(vf² - vi²) / (2 * a)
     * </pre>
     *
     * @param velocity Negative or positive.
     * @param deceleration Negative.
     */
    public static double getDistanceToVelocity(
            double velocity,
            double deceleration,
            double targetVelocity
    ) {
        return Math.abs((targetVelocity * targetVelocity - velocity * velocity) / (2 * deceleration));
    }

    /**
     * Calculate the distance needed to stop at a given velocity with the deceleration.
     *
     * <pre>
     * vf² = vi² + 2 * a * d
     * Solve for d
     * d = -(vf² - vi²) / (2 * a)
     * distanceToTarget > d
     * </pre>
     *
     * @param velocity Negative or positive.
     * @param deceleration Negative.
     */
    public static double getStoppingDistance(
            double velocity,
            double deceleration
    ) {
        return getDistanceToVelocity(velocity, deceleration, 0);
    }
}