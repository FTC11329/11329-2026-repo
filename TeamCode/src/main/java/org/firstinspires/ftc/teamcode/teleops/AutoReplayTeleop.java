package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.AutoReplayTime;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.FancyButton;
import org.firstinspires.ftc.teamcode.util.RobotSide;

public class AutoReplayTeleop {
    Robot robot;


    FancyButton shoot;

    double angle = 5;

    Gamepad gamepad1;
    Gamepad gamepad2;

    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepadInfo1;
    Gamepad gamepadInfo2;
    AutoReplayTime autoReplay;

    FancyButton intake;
    FancyButton spitIntake;
    FancyButton queueAny;
    FancyButton queueGreen;
    FancyButton queuePurple;

    public AutoReplayTeleop(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepadInfo1 = gamepad1;
        this.gamepadInfo2 = gamepad2;
    }

    public void init() {
        robot = new Robot(telemetry, hardwareMap, RobotSide.Blue);
        autoReplay = new AutoReplayTime(robot.follower, telemetry, gamepadInfo1, gamepad2);

        intake = new FancyButton(FancyButton.PressType.Toggle);
        spitIntake = new FancyButton(FancyButton.PressType.LongPress);
        queueAny = new FancyButton(FancyButton.PressType.LongPress);
        queueGreen = new FancyButton(FancyButton.PressType.LongPress);
        queuePurple = new FancyButton(FancyButton.PressType.LongPress);
    }

    public void loop() {

        robot.drivetrain.teleopMovement(-gamepadInfo1.left_stick_y, gamepadInfo1.left_stick_x, gamepadInfo1.right_stick_x, gamepadInfo1.right_bumper);

        if (autoReplay.IsReplayOn()){
            gamepad1 = autoReplay.getGamepad1();
            gamepad2 = autoReplay.getGamepad2();
        }
        else{
            gamepad1 = gamepadInfo1;
            gamepad2 = gamepadInfo2;
        }

        intake.checkStatus(gamepad1.left_bumper); // Toggle on to intake
        spitIntake.checkStatus(gamepad1.b); // Hold to spit
        queueGreen.checkStatus(gamepad1.y); // Press to queue green
        queuePurple.checkStatus(gamepad1.x); // Press to queue purple
        queueAny.checkStatus(gamepad1.a); // Press to queue any ball

        if (intake.startPress) {
            robot.intakeManual();
        }
        if (intake.endPress) {
            robot.stopIntake();
        }

        if (spitIntake.startPress) {
            robot.spitIntake();
            if (intake.isOn){
                // Makes the intake toggle correct
                intake.checkStatus(false);
                intake.checkStatus(true);
                intake.checkStatus(false);
            }
        }
        if (spitIntake.endPress) {
            robot.stopIntake();
            robot.autoIntake3();
        }

        if (queueAny.startPress) {
            robot.qBall(BallColor.Any);
        }
        if (queuePurple.startPress) {
            robot.qBall(BallColor.Purple);
        }
        if (queueGreen.startPress) {
            robot.qBall(BallColor.Green);
        }

        robot.update();
    }

}
