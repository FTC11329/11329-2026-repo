package org.firstinspires.ftc.teamcode.pedroPathing.geometry;

/**
 * Interface representing a future pose of a robot. This interface is used to retrieve the pose of the robot at any given time.
 * @author Havish Sripada - 12808 RevAmped Robotics
 */
@FunctionalInterface
public interface FuturePose {
    /**
     * Gets the pose of the robot at any given time
     * @return the pose of the robot
     */
    Pose getPose();

    /**
     * A default method to distinguish between a Pose and a FuturePose.
     * @return false if this is a FuturePose, true if it is a Pose
     */
    default boolean initialized() {
        return false;
    }
}