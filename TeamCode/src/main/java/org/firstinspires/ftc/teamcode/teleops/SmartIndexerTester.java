package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.SmartIndexerWNoTRev;
import org.firstinspires.ftc.teamcode.util.FancyButton;

@TeleOp(name = "Smart indexer Test", group = "zgroup")
public class SmartIndexerTester extends OpMode {
    TelemetryManager panelsTelemetry;
    SmartIndexerWNoTRev smartIndexer;
    Shooter shooter;
    Intake intake;
    double indexerPos = 0;
    FancyButton shoot;
    FancyButton autoShoot;

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        smartIndexer = new SmartIndexerWNoTRev(hardwareMap);
        shoot = new FancyButton(FancyButton.PressType.Toggle);
        autoShoot = new FancyButton(FancyButton.PressType.Toggle);
    }

    @Override
    public void loop() {
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
    }
}
