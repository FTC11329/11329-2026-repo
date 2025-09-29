package org.firstinspires.ftc.teamcode.pedroPathing.core;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.CoordinateSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PoseConverter {

    /**
     * Converts a Pose to a Pose2D in the desired coordinate system.
     *
     * @param pose the Pose object
     * @param desiredCoordinateSystem the desired coordinate system
     * @return a Pose2D object with the x and y coordinates in inches and the heading in radians
     */
    public static Pose2D poseToPose2D(Pose pose, CoordinateSystem desiredCoordinateSystem) {
        return new Pose2D(DistanceUnit.INCH, pose.getAsCoordinateSystem(desiredCoordinateSystem).getX(), pose.getAsCoordinateSystem(desiredCoordinateSystem).getY(), AngleUnit.RADIANS, pose.getAsCoordinateSystem(desiredCoordinateSystem).getHeading());
    }

    /**
     * Returns a pose from a Pose2D and a coordinate system.
     *
     * @param pose2d the Pose2D object
     * @param coordinateSystem the coordinate system
     * @return a Pose object with the x and y coordinates in inches and the heading in radians
     */
    public static Pose pose2DToPose(Pose2D pose2d, CoordinateSystem coordinateSystem) {
        return new Pose(pose2d.getX(DistanceUnit.INCH), pose2d.getY(DistanceUnit.INCH), pose2d.getHeading(AngleUnit.RADIANS), coordinateSystem);
    }
}
