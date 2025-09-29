package org.firstinspires.ftc.teamcode.pedroPathing.paths.callbacks;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Curve;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;

/**
 * This is the PoseCallback class. This class handles callbacks that occur after when the follower has reached or crossed a certain point on a path.
 * @author Havish Sripada - 12808 RevAmped Robotics
 */
public class PoseCallback implements PathCallback {
    private final Follower follower;
    private final int pathIndex;
    private final Pose targetPoint;
    private final Runnable runnable;
    private final Curve curve;
    private final double TValue;

    /**
     * Constructor for the PoseCallback class.
     * @param follower The follower that is running the path
     * @param pathIndex The index of the path that this callback is for
     * @param targetPoint The target point that the follower should reach in order to run this action
     * @param runnable The runnable that will be run when the callback is triggered
     * @param initialTValueGuess A guess for the initial t-value of the pose on this path
     * @param curve The curve that the follower is following
     */
    public PoseCallback(Follower follower, int pathIndex, Pose targetPoint, Runnable runnable, double initialTValueGuess, Curve curve) {
        this.follower = follower;
        this.pathIndex = pathIndex;
        this.targetPoint = targetPoint;
        this.runnable = runnable;
        this.curve = curve;
        TValue = curve.getClosestPoint(targetPoint, curve.getPathConstraints().getBEZIER_CURVE_SEARCH_LIMIT(), initialTValueGuess);
    }

    /**
     * This method runs the callback.
     * @return true if the action was successful
     */
    @Override
    public boolean run() {
        runnable.run();
        return true;
    }

    /**
     * This method checks if the callback is ready to run.
     * @return if the callback is ready to run
     */
    @Override
    public boolean isReady() {
        return follower.getCurrentTValue() >= TValue;
    }

    /**
     * This method returns the index of the path that this callback is for.
     * @return the index of the path that this callback is for
     */
    @Override
    public int getPathIndex() {
        return pathIndex;
    }
}
