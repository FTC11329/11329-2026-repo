package org.firstinspires.ftc.teamcode.pedroPathing.paths;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.math.Vector;

/**
 * This is the PathPoint class. This class represents a point in a path. It contains information about the t-value, pose, and tangent vector of the point.
 * @author Havish Sripada - 12808 RevAmped Robotics
 */
public class PathPoint {
    /** The parametric t-value of the point on the path. */
    public final double tValue;
    /** The pose (position and orientation) at this point on the path. */
    public final Pose pose;
    /** The tangent vector at this point on the path. */
    public final Vector tangentVector;

    /**
     * Default constructor for the PathPoint class. Creates a point with a t-value of 0, a pose of (0, 0, 0), and a tangent vector of (0, 0).
     */
    public PathPoint() {
        this(0, new Pose(), new Vector());
    }

    /**
     * Constructor for the PathPoint class.
     * @param tValue The t-value of the point
     * @param pose The pose of the point
     * @param tangentVector The tangent vector of path at the point
     */
    public PathPoint(double tValue, Pose pose, Vector tangentVector) {
        this.tValue = tValue;
        this.pose = pose;
        this.tangentVector = tangentVector;
    }

    /**
     * Returns the t-value of this path point.
     * @return the t-value
     */
    public double getTValue() {
        return tValue;
    }

    /**
     * Returns the pose at this path point.
     * @return the pose
     */
    public Pose getPose() {
        return pose;
    }

    /**
     * Returns the tangent vector at this path point.
     * @return the tangent vector
     */
    public Vector getTangentVector() {
        return tangentVector;
    }
}
