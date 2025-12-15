package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.ftcontrol.panels.Panels;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.control.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.FancyButton;
import org.firstinspires.ftc.teamcode.util.RobotSide;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "Shooter PIDF tuner", group = "zgroup")
public class PIDFTuner extends OpMode {
    //This is where we introduce the tele-operated controls
    Robot robot;

    public List<Pose> poses;
    public List<String> titles;
    public FancyButton timerP = new FancyButton(FancyButton.PressType.LongPress);
    public FancyButton timerI = new FancyButton(FancyButton.PressType.LongPress);
    public FancyButton timerD = new FancyButton(FancyButton.PressType.LongPress);
    public FancyButton timerF = new FancyButton(FancyButton.PressType.LongPress);

    public double p, i, d, f = 0;

    Panels dashboard;

    @Override
    public void init() {
        robot = new Robot(telemetry, hardwareMap, RobotSide.Blue, 0);
        titles = Arrays.asList("startPose", "scorePose", "collectI1", "collectF1", "collectI2", "collectF2","collectI3", "collectF3");
        poses = Arrays.asList(null, null, null, null, null, null, null, null);
        dashboard = Panels.getInstance();
    }

    @Override
    public void loop() {
        timerP.checkStatus(gamepad1.a);
        timerI.checkStatus(gamepad1.b);
        timerD.checkStatus(gamepad1.x);
        timerF.checkStatus(gamepad1.y);


        if (timerP.isPressed){
            p = timerP.time.seconds()/100;
        }
        if (timerI.isPressed){
            i = timerI.time.seconds()/100;
        }
        if (timerD.isPressed){
            d = timerD.time.seconds()/100;
        }
        if (timerF.isPressed){
            f = timerF.time.seconds()/100;
        }

        if (timerP.endPress || timerI.endPress || timerD.endPress || timerF.endPress){
            robot.shooter.shooterPID.setCoefficients(new PIDFCoefficients(p, i, d, f));
        }

        if (gamepad1.left_bumper){
            robot.shooter.setTargetRPM(3500);
        }else{
            robot.shooter.setPower(0);
        }

        robot.drivetrain.teleopMovement(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, gamepad1.left_bumper);

        telemetry.addData("p", p);
        telemetry.addData("i", i);
        telemetry.addData("d", d);
        telemetry.addData("f", f);

        robot.update();
    }

}
