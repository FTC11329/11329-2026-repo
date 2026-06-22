package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.geom.Polygon;

public class ShapeDetection {

    private static final GeometryFactory geometryFactory = new GeometryFactory();

    private static final Pose[] closeTriangleCorners = new Pose[]{
            new Pose(96, 96),
            new Pose(96, -96),
            new Pose(0,  0)
    };
    private static final Pose[] farTriangleCorners = new Pose[]{
            new Pose(-96, 24),
            new Pose(-48, 24),
            new Pose(-24, 0),
            new Pose(-48, -24),
            new Pose(-96, -24)
    };
    private static final Pose[] seesObeliskFrontTagCorners =  new Pose[]{
            new Pose(75, 11),
            new Pose(75, -11),
            new Pose(-36,-72),//set x
            new Pose(-72, -72),
            new Pose(-72, 72),
            new Pose(-36,72),//set x
    };
    private static final Pose[] notVisionAreaCorners =  new Pose[]{
            new Pose(-66, -36),
            new Pose(72, -36),
            new Pose(72,36),//set x
            new Pose(-66,36),//set x
    };
    private static final Polygon closeTriangle = createPolygon(closeTriangleCorners);
    private static final Polygon farTriangle = createPolygon(farTriangleCorners);
    private static final Polygon seesObeliskFrontTag = createPolygon(seesObeliskFrontTagCorners);
    private static final Polygon notVisionArea = createPolygon(notVisionAreaCorners);

    private static final double robotSizeMult = 2.25;

    private static final double forwardFromCenter = 10;
    private static final double leftFromCenter = 8.5;
    private static final double rightFromCenter = 8.5;
    private static final double backFromCenter = 6.8;


    private static final Pose[] localCorners = new Pose[] {
            new Pose( forwardFromCenter,  leftFromCenter),
            new Pose( forwardFromCenter, -rightFromCenter),
            new Pose(-backFromCenter,    -rightFromCenter),
            new Pose(-backFromCenter,     leftFromCenter)
    };

    private static final Pose[] arrowPoints = new Pose[] {
            new Pose(7.6, 0),
            new Pose(4.1, -3.3),
            new Pose(4.1, 3.3),
            new Pose(-2.2,0)
    };
    public static boolean isRobotInside(FieldShapes shape, Pose robotPose) {
        return isRobotInside(shape, robotPose, false);
    }

    public static boolean isRobotInside(FieldShapes shape, Pose robotPose, boolean largeRobot) {
        Polygon polyA = null;
        switch (shape) {
            case closeTriangle:
                polyA = closeTriangle;
                break;
            case farTriangle:
                polyA = farTriangle;
                break;
            case seesObeliskFrontTag:
                polyA = seesObeliskFrontTag;
                break;
        }
        return polyA.intersects(createPolygon(createRobotCorners(robotPose, largeRobot)));
    }

    public static boolean doesRobotCrossLine(FieldShapes shape, Pose robotPose) {
        Polygon polyA = null;
        switch (shape) {
            case closeTriangle:
                polyA = closeTriangle;
                break;
            case farTriangle:
                polyA = farTriangle;
                break;
            case seesObeliskFrontTag:
                polyA = seesObeliskFrontTag;
                break;
        }
        return polyA.overlaps(createPolygon(createRobotCorners(robotPose)));
    }

    public static Pose[] createRobotCorners(Pose robotPose) {
        return createRobotCorners(robotPose, false);
    }
    public static Pose[] createRobotCorners(Pose robotPose, boolean large) {
        Pose[] worldCorners = new Pose[4];

        Pose[] actualCorners = localCorners.clone();

        if (large) {
            for (Pose actualCorner : actualCorners) {
                actualCorner.scale(robotSizeMult);
            }
        }

        for (int i = 0; i < 4; i++) {
            // rotate around 0,0
            Pose rotated = actualCorners[i].rotate(robotPose.getHeading(), false);

            // translate into world space
            worldCorners[i] = new Pose(
                    rotated.getX() + robotPose.getX(),
                    rotated.getY() + robotPose.getY(),
                    0
            );
        }

        return worldCorners;
    }

    public static Pose[] createRobotsArrowCorners(Pose robotPose) {
        Pose[] worldCorners = new Pose[4];

        for (int i = 0; i < 4; i++) {
            // rotate around 0,0
            Pose rotated = arrowPoints[i].rotate(robotPose.getHeading(), false);

            // translate into world space
            worldCorners[i] = new Pose(
                    rotated.getX() + robotPose.getX(),
                    rotated.getY() + robotPose.getY(),
                    0
            );
        }

        return worldCorners;
    }

    // creates a polygon from the points, using the first point to close the shape
    private static Polygon createPolygon(Pose[] pts) {
        Coordinate[] coords = new Coordinate[pts.length + 1];

        for (int i = 0; i < pts.length; i++) {
            coords[i] = new Coordinate(pts[i].getX(), pts[i].getY());
        }
        //repeats the first pose
        coords[coords.length - 1] = new Coordinate(pts[0].getX(), pts[0].getY());

        return geometryFactory.createPolygon(coords);
    }

    public static Pose[] getCornersOfShape(FieldShapes shape) {
        switch (shape) {
            case closeTriangle:
                return closeTriangleCorners;
            case farTriangle:
                return farTriangleCorners;
            case seesObeliskFrontTag:
            default:
                return seesObeliskFrontTagCorners;
        }
    }
    public static boolean isPoseInsideOfVisionArea(Pose pose) {
        Point point = geometryFactory.createPoint(new Coordinate(pose.getX(), pose.getY()));
        return notVisionArea.contains(point);
    }
}
