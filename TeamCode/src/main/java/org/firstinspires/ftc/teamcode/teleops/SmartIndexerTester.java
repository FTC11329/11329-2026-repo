package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.SmartIndexerDumbWrapper;
import org.firstinspires.ftc.teamcode.subsystems.SmartIndexerWNoTRev;
import org.firstinspires.ftc.teamcode.util.FancyButton;
import org.firstinspires.ftc.teamcode.util.RobotSide;

@TeleOp(name = "Smart indexer Test", group = "zgroup")
public class SmartIndexerTester extends OpMode {
    TelemetryManager panelsTelemetry;
    SmartIndexerWNoTRev smartIndexer;
    Shooter shooter;
    double indexerIndex = 0;
    double rpm = 0;
    double hood = 5;


    FancyButton shoot;
    FancyButton autoShoot;
    FancyButton hoodUp, hoodDown, rpmUp, rpmDown, rpmFastUp, rpmFastDown, intake;
    SmartIndexerDumbWrapper indexerDumbWrapper;
    Robot robot;

    @Override
    public void init() {
        indexerDumbWrapper = new SmartIndexerDumbWrapper(hardwareMap);
        shoot = new FancyButton(FancyButton.PressType.LongPress);
        autoShoot = new FancyButton(FancyButton.PressType.LongPress);
        robot = new Robot(telemetry, hardwareMap, RobotSide.Blue, 0, 0);

        hoodUp = new FancyButton(FancyButton.PressType.LongPress);
        hoodDown = new FancyButton(FancyButton.PressType.LongPress);
        rpmUp = new FancyButton(FancyButton.PressType.LongPress);
        rpmDown = new FancyButton(FancyButton.PressType.LongPress);
        rpmFastUp = new FancyButton(FancyButton.PressType.LongPress);
        rpmFastDown = new FancyButton(FancyButton.PressType.LongPress);
        intake = new FancyButton(FancyButton.PressType.Toggle);


        /*
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        smartIndexer = new SmartIndexerWNoTRev(hardwareMap);
        shoot = new FancyButton(FancyButton.PressType.Toggle);
        autoShoot = new FancyButton(FancyButton.PressType.Toggle);

         */
    }

    @Override
    public void loop() {
        intake.checkStatus(gamepad2.back);
        hoodUp.checkStatus(gamepad1.dpad_up);
        hoodDown.checkStatus(gamepad1.dpad_down);
        rpmUp.checkStatus(gamepad1.y);
        rpmDown.checkStatus(gamepad1.a);
        rpmFastUp.checkStatus(gamepad1.b);
        rpmFastDown.checkStatus(gamepad1.x);
        shoot.checkStatus(gamepad2.dpad_up);
        autoShoot.checkStatus(gamepad2.dpad_down);

        if (shoot.startPress) {
            indexerDumbWrapper.upOneIndex();
        }
        if (autoShoot.startPress) {
            indexerDumbWrapper.downOneIndex();
        }
        telemetry.addData("distance", robot.getCurrentPose().distanceFrom(Constants.Vision.blueGoal));
        telemetry.addData("Target RPM", robot.shooter.getTargetRpm());
        telemetry.addData("Hood angle", robot.shooter.getHoodPosDeg());

        indexerDumbWrapper.update();

        if (hoodUp.startPress) {
            hood += 0.5;
        }
        if (hoodDown.startPress) {
            hood -= 0.5;
        }
        if (rpmUp.startPress) {
            rpm += 50;
        }
        if (rpmDown.startPress) {
            rpm -= 50;
        }
        if (rpmFastUp.startPress) {
            rpm += 500;
        }
        if (rpmFastDown.startPress) {
            rpm -= 500;
        }
        if (gamepad1.back) {
            robot.indexer.setIndexerToShooterPower(0.8);
        } else {
            robot.indexer.setIndexerToShooterPower(0);
        }
        if (intake.isOn) {
            robot.intake.setIntakePower(0.9);
        } else {
            robot.intake.setIntakePower(0);
        }

        robot.shooter.setHoodDeg(hood);
        robot.shooter.adjustTargetRPM(rpm);
        robot.shooterUpdate();
        robot.turretUpdate();
        robot.prepareTurret();
        /*
        shoot.checkStatus(gamepad1.right_bumper);
        autoShoot.checkStatus(gamepad1.a);
        if (shoot.isOn) {
            shooter.setTargetRPM(1500);
        } else {
            shooter.setTargetRPM(0);
        }

        if (gamepad1.left_bumper) {
            intake.setIntakePower(0.7);
        } else {
            intake.setIntakePower(0);
        }
        //                     Cancel          Ready         Left        auto shoot
        smartIndexer.update(gamepad1.start, gamepad1.back, gamepad1.a, autoShoot.isOn);
        shooter.update();
        panelsTelemetry.update(telemetry);

         */
    }
}
