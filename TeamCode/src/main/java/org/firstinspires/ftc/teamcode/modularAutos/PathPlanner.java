package org.firstinspires.ftc.teamcode.modularAutos;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;

public interface PathPlanner {
    // Run the step, return true when it's done
    boolean run();

    // Builds all the paths with the previous offset
    void buildPaths(Pose offset, boolean blueSide);

    // Used to connect paths
    Pose getEndPoseEst();

    // Gets the name of the module
    String getName();

}
