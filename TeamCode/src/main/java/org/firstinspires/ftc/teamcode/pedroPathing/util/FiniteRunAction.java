package org.firstinspires.ftc.teamcode.pedroPathing.util;

import org.firstinspires.ftc.teamcode.pedroPathing.paths.callbacks.PathCallback;

/**
 * This is the FiniteRunAction class. It handles running Runnables once (or a specified number of times), until a reset is called.
 * It also forms the basis of the PathCallback class. Basically, if you want to run a certain action
 * once despite looping through a section of code multiple times, then this is for you.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Havish Sripada - 12808 RevAmped Robotics
 * @version 2.0, 5/6/2025
 */
public class FiniteRunAction implements PathCallback {
    private int runCount = 0;
    private int targetCount = 1;
    private boolean initialized = false;

    private final PathCallback callback;

    /**
     * This creates a new FiniteRunAction with a Runnable containing the code to be run for this action.
     * The name is a slight bit misleading, as this can actually be run multiple times. However, the
     * run() method only runs once before the reset() method needs to be called to allow the
     * FiniteRunAction to run again.
     *
     * @param callback This is a Runnable containing the code to be run. Preferably, use lambda statements.
     */
    public FiniteRunAction(PathCallback callback) {
        this.callback = callback;
    }

    /**
     * This creates a new FiniteRunAction with a Runnable containing the code to be run for this action.
     * The name is a slight bit misleading, as this can actually be run multiple times. However, the
     * run() method only runs once before the reset() method needs to be called to allow the
     * FiniteRunAction to run again.
     *
     * @param callback This is a Runnable containing the code to be run. Preferably, use lambda statements.
     */
    public FiniteRunAction(PathCallback callback, int targetCount) {
        this(callback);
        this.targetCount = targetCount;
    }

    /**
     * This returns if the FiniteRunAction has been run as many times as specified in the constructor. Running reset() will reset this.
     *
     * @return This returns if it has been run.
     */
    public boolean isCompleted() {
        return runCount >= targetCount;
    }

    /**
     * This returns if the FiniteRunAction has been run yet. Running reset() will reset this.
     *
     * @return This returns if it has been run.
     */
    public boolean hasBeenRunOnce() {
        return runCount > 0;
    }

    /**
     * This runs the Runnable of the FiniteRunAction. It will only run once before requiring a reset.
     *
     * @return This returns if the Runnable was successfully run once.
     */

    @Override
    public boolean run() {
        if (!isCompleted()) {
            runCount++;
            callback.run();
            return true;
        }
        return false;
    }

    /**
     * This resets the FiniteRunAction and makes it able to run again. The FiniteRunAction is set
     * to "has not been run", allowing for multiple uses of the Runnable.
     */
    public void reset() {
        runCount = 0;
    }

    /**
     * This completes the FiniteRunAction so it cannot be run again until reset() is called.
     */
    public void complete() {
        runCount = targetCount;
    }

    @Override
    public boolean isReady() {
        return callback.isReady() && initialized;
    }

    @Override
    public void initialize() {
        callback.initialize();
        initialized = true;
    }

    public boolean isInitialized() {
        return initialized;
    }

    @Override
    public int getPathIndex() {
        return callback.getPathIndex();
    }
}
