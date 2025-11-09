package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.RobotSide;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class AprilTag {
    public AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private RobotSide robotSide;
    private int goalID;
    public AprilTag(HardwareMap hardwareMap, RobotSide robotSide) {
        this.robotSide = robotSide;
        if (robotSide == RobotSide.Red) {
            goalID = 24;
        } else {
            goalID = 20;
        }
        // Setup the processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setCameraPose(new Position(DistanceUnit.INCH, 0, 0, 0, 0), new YawPitchRollAngles(AngleUnit.DEGREES,0,0, 0, 0))
                .setLensIntrinsics(903.79, 903.79, 699.758, 372.872) //todo IMPORTANT
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();
        // Setup the vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(aprilTagProcessor)
                .build();
    }
    // Gets whatever tags the camera sees
    public ArrayList<AprilTagDetection> getAprilTagList() {
        return aprilTagProcessor.getDetections();
    }

    //returns null if id isnt found
    public AprilTagDetection getTagWithID(int id) {
        for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

}
