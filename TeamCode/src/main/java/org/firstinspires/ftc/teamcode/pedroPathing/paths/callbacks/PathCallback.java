package org.firstinspires.ftc.teamcode.pedroPathing.paths.callbacks;

/**
 * This is the PathCallback class. This class handles callbacks of Runnables in PathChains.
 * Basically, this allows you to run non-blocking code in the middle of PathChains.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Havish Sripada - 12808 RevAmped Robotics
 * @version 2.0, 5/6/2025
 */
public interface PathCallback {
    /**
     * This runs the callback.
     * @return true if the action was successful
     */
    boolean run();

    /**
     * This checks if the callback is ready to run.
     * @return true if the callback is ready to run
     */
    boolean isReady();

    /**
     * This initializes the callback.
     */
    default void initialize() {}

    /**
     * This resets the callback.
     */
    default void reset() {}

    /**
     * This checks if the callback is completed.
     * @return true if the callback is completed
     */
    default boolean isCompleted() {return false;}

    /**
     * This returns the index of the path that this callback is for.
     * @return the index of the path that this callback is for
     */
    int getPathIndex();
}
