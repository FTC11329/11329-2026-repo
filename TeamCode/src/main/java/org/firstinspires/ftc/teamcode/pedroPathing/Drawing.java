package org.firstinspires.ftc.teamcode.pedroPathing;

import android.provider.Settings;

import com.bylazar.field.CanvasRotation;
import com.bylazar.field.FieldManager;
import com.bylazar.field.FieldPresetParams;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.field.Line;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.math.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PoseHistory;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.FieldShapes;
import org.firstinspires.ftc.teamcode.util.ShapeDetection;

import java.util.List;


/**
 * This is the Drawing class. It handles the drawing of stuff on Panels Dashboard, like the robot.
 *
 * @author Lazar - 19234
 * @version 1.1, 5/19/2025
 */
public class Drawing {
    public static final double ROBOT_RADIUS = 9; // woah
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
    private static final TelemetryManager panelsTelem = PanelsTelemetry.INSTANCE.getTelemetry();

    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.75
    );
    private static final Style historyLook = new Style(
            "", "#096E11", 0.75
    );
    private static final Style shootZoneLook = new Style(
            "", "#50C878", 0.75
    );
    private static final Style seesObeliskLook = new Style(
            "", "#FFBF00", 0.75
    );
    private static final Style purpleBallLook = new Style(
            "", "#71187d", 0.75
    );
    private static final Style greenBallLook = new Style(
            "", "#29c324", 0.75
    );

    /**
     * This prepares Panels Field for using RoadRunner (I think its right)
     */
    public static void init() {
        panelsField.setOffsets(new FieldPresetParams("MYNE", 0, 0, CanvasRotation.DEG_90, true, true, true));
    }

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashbaordDrawingHandler class is not needed.
     *
     * @param follower Pedro Follower instance.
     */
    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook);
        drawRobot(follower.getPose(), historyLook);

        sendPacket();
    }

    public static void drawShapesDebug(Follower follower) {

        for (FieldShapes shapes : FieldShapes.values()) {
            switch (shapes) {
                case farTriangle:
                case closeTriangle:
                    panelsField.setStyle(shootZoneLook);
                    break;
                case seesObeliskFrontTag:
                    panelsField.setStyle(seesObeliskLook);
                    break;
            }
            Pose[] corners = ShapeDetection.getCornersOfShape(shapes);
            drawShapeUsingCorners(corners);
        }

        panelsField.setStyle(robotLook);
        drawShapeUsingCorners(ShapeDetection.createRobotCorners(follower.getPose()));
        drawShapeUsingCornersNoMovingCursor(ShapeDetection.createRobotsArrowCorners(follower.getPose()));

        Pose shooterPose = follower.getCenterOfShooterPose();

        panelsField.moveCursor(shooterPose.getX(), shooterPose.getY());
        panelsField.circle(2.6);
        panelsField.line(60,60);
        sendPacket();
    }

    public static void drawBalls(List<Vision.DetectedBall> detectBallsList) {
        for (Vision.DetectedBall detectedBall : detectBallsList) {
            drawBallNoPacketSend(detectedBall.ballPose, detectedBall.ballColor);
        }
        sendPacket();
    }

    public static void drawBallNoPacketSend(Pose ball, BallColor ballColor) {
        if (ballColor == BallColor.Purple) {
            panelsField.setStyle(purpleBallLook);
        } else if (ballColor == BallColor.Green) {
            panelsField.setStyle(greenBallLook);
        }
        panelsField.moveCursor(ball.getX(), ball.getY());
        panelsField.circle(2.5);
    }
    private static void drawShapeUsingCorners(Pose[] corners) {
        boolean first = true;
        for (Pose corner : corners) {
            if (first) {
                panelsField.moveCursor(corners[0].getX(), corners[0].getY());
                first = false;
            } else {
                panelsField.line(corner.getX(), corner.getY());
                panelsField.moveCursor(corner.getX(), corner.getY());
            }
        }
        panelsField.line(corners[0].getX(), corners[0].getY());
    }

    private static void drawShapeUsingCornersNoMovingCursor(Pose[] corners) {
        boolean first = true;
        for (Pose corner : corners) {
            if (first) {
                panelsField.moveCursor(corners[0].getX(), corners[0].getY());
                first = false;
            } else {
                panelsField.line(corner.getX(), corner.getY());
            }
        }
        panelsField.line(corners[0].getX(), corners[0].getY());
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the Pose to draw the robot at
     * @param style the parameters used to draw the robot with
     */
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    /**
     * This draws a robot at a specified Pose. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /**
     * This draws a Path with a specified look.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    public static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();

        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }

    /**
     * This draws all the Paths in a PathChain with a
     * specified look.
     *
     * @param pathChain the PathChain to draw
     * @param style     the parameters used to draw the PathChain with
     */
    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     * @param style       the parameters used to draw the pose history with
     */
    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);

        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {

            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     */
    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, historyLook);
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     */
    public static void sendPacket() {
        panelsField.update();
    }
}
