package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.FancyButton;

import java.util.List;

@TeleOp(name = "TEST", group = "       group")
public class TemporaryClassThatWeAreTotallyGoingToDelete extends OpMode {
    Shooter shooter;
    TelemetryManager panelsTelemetry;
    Robot robot;
    List<LynxModule> hubs;
    Indexer indexer;
    Intake intake;
    FancyButton intakeToggle;
    DigitalChannel color6;
    DigitalChannel color7;
    AnalogInput analog2;
    AnalogInput analog3;

    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;

    CRServo turretServo1;
    CRServo turretServo2;
    DcMotorEx rightBack;
    TouchSensor touchSensor;
    double hoodPos = 0;
    double lastTime = 0;
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        shooter = new Shooter(hardwareMap);
        indexer = new Indexer(hardwareMap);
        intake = new Intake(hardwareMap);
        intakeToggle = new FancyButton(FancyButton.PressType.Toggle);
    }

    @Override
    public void loop() {
        intakeToggle.checkStatus(gamepad1.left_bumper);
        intake.intake(intakeToggle.isOn);
        if (gamepad1.aWasPressed()) {
            indexer.shootAll();
        }

        shooter.setTargetRPM(2500);
        shooter.setHoodDeg(30);
        indexer.update(true, true, new Pose());
        shooter.update();
        panelsTelemetry.addData("Power", gamepad1.left_stick_y);
        panelsTelemetry.addData("Hood", hoodPos);
        panelsTelemetry.addData("Pos", shooter.getRPM());
        panelsTelemetry.addData("Tar", shooter.shooterPID.getTargetPosition() );
        telemetry.update();
    }
}
