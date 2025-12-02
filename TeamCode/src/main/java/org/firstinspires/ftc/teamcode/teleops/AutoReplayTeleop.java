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


    Gamepad gamepad1;
    Gamepad gamepad2;

    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepadInfo1;
    Gamepad gamepadInfo2;
    AutoReplayTime autoReplay;

    FancyButton intake;
    FancyButton spitIntake;
    FancyButton autoShoot;
    FancyButton queueGreen;
    FancyButton queuePurple;
    FancyButton overrideShootPosition;
    FancyButton debug;

    public double hoodAngle = 20;
    public double rpm = 3000;

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
        queueGreen = new FancyButton(FancyButton.PressType.LongPress);
        queuePurple = new FancyButton(FancyButton.PressType.LongPress);
        autoShoot = new FancyButton(FancyButton.PressType.Toggle);
        overrideShootPosition = new FancyButton(FancyButton.PressType.LongPress);
        debug = new FancyButton(FancyButton.PressType.LongPress);
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
        queueGreen.checkStatus(gamepad2.y); // Press to queue green
        queuePurple.checkStatus(gamepad2.x); // Press to queue purple
        autoShoot.checkStatus(gamepad2.a); // Toggle to turn on auto shoot
        overrideShootPosition.checkStatus(gamepad2.back); // hold to turn on ignore position
        debug.checkStatus(gamepad1.start); // hold to print telemetry


        robot.drivetrain.teleopMovement(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_bumper);

        if (gamepad2.dpad_up) {
            hoodAngle += 5;
            robot.shooter.setHoodDeg(hoodAngle);
        } else if (hoodAngle >= 5 && gamepad2.dpad_down) {
            hoodAngle -= 5;
            robot.shooter.setHoodDeg(hoodAngle);
        }
        if (gamepad2.dpad_left) {
            rpm += 100;
            robot.shooter.setTargetRPM(rpm);
        } else if (gamepad2.dpad_right) {
            rpm -= 100;
            robot.shooter.setTargetRPM(rpm);
        }
        if (gamepad2.left_bumper) {
            robot.shootArtifact(BallColor.Any);
        }

        if (intake.startPress) {
            robot.intakeManual();
        }
        if (intake.endPress) {
            robot.stopIntake();
            robot.stopIndexer();
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
        }

        if (autoShoot.isOn) {
            robot.prepareShooter();
            robot.shootQueue(overrideShootPosition.isOn);
        } else if (autoShoot.endPress){
            robot.setShooterTargetRPM(0);
        }
        if (queuePurple.startPress) {
            robot.qBall(BallColor.Purple);
        }
        if (queueGreen.startPress) {
            robot.qBall(BallColor.Green);
        }

        robot.update(debug.isOn);

        telemetry.addData("Encoder RPM", robot.shooter.getRPM());
        telemetry.addData("Hood angle", robot.shooter.getHoodPosDeg());
        telemetry.addData("Hood angle", robot.vision.distanceXToGoal(robot.follower.getPose()));
    }

}
