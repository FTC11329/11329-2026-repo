package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.SmartIndexerButEvenNewer;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.FancyButton;
import org.firstinspires.ftc.teamcode.util.IndexerEnums;
import org.firstinspires.ftc.teamcode.util.IndexerEnumsButEvenNewerThisTime;

@TeleOp(name = "SmartIndexerTester", group = "                                                    group")
public class SmartIndexerTester extends OpMode {
    SmartIndexerButEvenNewer smartIndexer;
    Intake intake;
    Shooter shooter;
    FancyButton intaking = new FancyButton(FancyButton.PressType.Toggle);
    FancyButton shooting = new FancyButton(FancyButton.PressType.Toggle);
    TelemetryManager panelsTelemetry;

    @Override
    public void init() {
        smartIndexer = new SmartIndexerButEvenNewer(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        intaking.checkStatus(gamepad1.left_bumper);
        shooting.checkStatus(gamepad1.right_bumper);

        if (gamepad1.bWasPressed()) {
            smartIndexer.shootAll();
        }

        shooter.setTargetRPM(shooting.isOn ? 1000 : 0);
        intake.intake(intaking.isOn);

        shooter.update();
        smartIndexer.update(intaking.isOn, true);
        telemetry.addData("target enum", smartIndexer.currentIndexerState);
        telemetry.addData("target percent", IndexerEnumsButEvenNewerThisTime.convertEnumToPercentOfRot(smartIndexer.currentIndexerState));
        telemetry.addData("target percent act", smartIndexer.lastIndexerTarget);
        telemetry.addData("actual percent", smartIndexer.getEncoderPercentage());
        telemetry.addData("color", smartIndexer.getColor());
        for (BallColor i : smartIndexer.getBallCells()) {
            telemetry.addData("ballcells", i);
        }
        panelsTelemetry.addData("curr", IndexerEnumsButEvenNewerThisTime.getIndex(smartIndexer.currentIndexerState));
        panelsTelemetry.update();
    }
}
