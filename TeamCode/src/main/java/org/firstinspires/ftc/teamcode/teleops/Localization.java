package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.RobotSide;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "Localization", group = "       group")
public class Localization extends OpMode {
    //This is where we introduce the tele-operated controls
    Robot robot;

    public List<Pose> poses;
    public List<String> titles;
    public int index =0;

    @Override
    public void init() {
        robot = new Robot(telemetry, hardwareMap, RobotSide.Blue);
        titles = Arrays.asList("startPose", "scorePose", "collectI1", "collectF1", "collectI2", "collectF2","collectI3", "collectF3");
        poses = Arrays.asList(null, null, null, null, null, null, null, null);
    }

    @Override
    public void loop() {
        robot.drivetrain.teleopMovement(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, gamepad1.left_bumper);
        robot.update();
        if (gamepad1.aWasReleased()){
            poses.set(index, robot.getCurrentPose());
            index += 1;
            if (index >= 8){
                index = 0;
            }
        }
        telemetry.addData("currentPos", robot.getCurrentPose());

        telemetry.addData("index", index);

        for (int i = 0; i < titles.size(); i++){
            telemetry.addData(titles.get(i), poses.get(i));
        }
    }

}
