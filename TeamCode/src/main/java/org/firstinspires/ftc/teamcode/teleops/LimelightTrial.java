package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.RobotSide;

@TeleOp(name = "Limelight Test", group = "group")
public class LimelightTrial extends OpMode {
    //This is where we introduce the tele-operated controls
    Vision vision;

    ElapsedTime time;
    double deltaTime;
    double lastTime;

    @Override
    public void init() {
        //do stuff init
        vision = new Vision(hardwareMap, RobotSide.Blue);
    }

    @Override
    public void start(){
        time = new ElapsedTime();
        time.reset();
        lastTime = time.milliseconds();
    }

    @Override
    public void loop() {
        Pose pos = vision.getRobotPose();
        if (pos == null){
            pos = new Pose();
        }
        telemetry.addData("heading:", vision.getVelocityMaxHeight(2,pos, new Pose())[0]);
        telemetry.addData("turret angle:", vision.getVelocityMaxHeight(2,pos, new Pose())[1]);
        telemetry.addData("velocity:", vision.getVelocityMaxHeight(2,pos, new Pose())[2]);
        telemetry.addData("pos:", pos);

    }
}
