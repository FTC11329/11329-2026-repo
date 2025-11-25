package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.control.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.PressHold;
import org.firstinspires.ftc.teamcode.util.RobotSide;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "Turret PIDF tuner", group = "       group")
public class PIDFTunerTurret extends OpMode {
    //This is where we introduce the tele-operated controls
    Robot robot;

    public List<Pose> poses;
    public List<String> titles;
    public PressHold timerP = new PressHold(PressHold.PressType.LongPress);
    public PressHold timerI = new PressHold(PressHold.PressType.LongPress);
    public PressHold timerD = new PressHold(PressHold.PressType.LongPress);
    public PressHold timerF = new PressHold(PressHold.PressType.LongPress);

    public double p, i, d, f = 0;

    FtcDashboard dashboard;

    @Override
    public void init() {
        robot = new Robot(telemetry, hardwareMap, RobotSide.Blue);
        titles = Arrays.asList("startPose", "scorePose", "collectI1", "collectF1", "collectI2", "collectF2","collectI3", "collectF3");
        poses = Arrays.asList(null, null, null, null, null, null, null, null);
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        timerP.checkStatus(gamepad1.a);
        timerI.checkStatus(gamepad1.b);
        timerD.checkStatus(gamepad1.x);
        timerF.checkStatus(gamepad1.y);


        if (timerP.isPressed){
            p = timerP.time.seconds()/2;
        }
        if (timerI.isPressed){
            i = timerI.time.seconds()/2;
        }
        if (timerD.isPressed){
            d = timerD.time.seconds()/2;
        }
        if (timerF.isPressed){
            f = timerF.time.seconds()/2;
        }

        if (timerP.endPress || timerI.endPress || timerD.endPress || timerF.endPress){
            robot.turret.turretPID.setCoefficients(new PIDFCoefficients(p, i, d, f));
        }

        if (gamepad1.leftBumperWasPressed()){
            robot.turret.turnTo(30);
        }
        if (gamepad1.leftBumperWasReleased()){
            robot.turret.turnTo(80);
        }

        robot.turret.updateTurret();

        robot.drivetrain.teleopMovement(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, gamepad1.left_bumper);

        telemetry.addData("p", p);
        telemetry.addData("i", i);
        telemetry.addData("d", d);
        telemetry.addData("f", f);
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Motor Velocity", robot.shooter.getRPM()); // motor speed in ticks/sec
        dashboard.sendTelemetryPacket(packet);

        robot.update();
    }

}
