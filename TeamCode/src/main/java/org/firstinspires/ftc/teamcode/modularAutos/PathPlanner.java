package org.firstinspires.ftc.teamcode.modularAutos;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;

public interface PathPlanner {
    default boolean hasComms() {
        return false;
    }
    default boolean useSOTF() {
        return false;
    }
    default void setOptimalEndPose(Pose optimalEndPose) {}
    default Pose getOptimalStartPose() {
        return null;
    }

    // Builds all the paths with the previous offset
    void buildPaths();

    // Run the step, return true if it's done
    boolean run();

    // Used to connect paths
    Pose getEndPoseEst();

    // Gets the name of the module
    @NonNull
    String toString();

}
