package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.List;

@TeleOp(name = "TEST", group = "       group")
public class TemporaryClassThatWeAreTotallyGoingToDelete extends OpMode {
    Shooter shooter;
    List<LynxModule> hubs;
    Indexer indexer;
    TelemetryManager panelsTelemetry;

    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;

    CRServo turretServo1;
    CRServo turretServo2;
    DcMotorEx rightBack;
    double hoodPos = 5;
    TouchSensor touchSensor;
    @Override
    public void init() {
        indexer = new Indexer(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("color", indexer.getColor());
        telemetry.addData("red", indexer.getColorRGBA().red);
        telemetry.addData("green", indexer.getColorRGBA().green);
        telemetry.addData("blue", indexer.getColorRGBA().blue);
        telemetry.addData("alpha", indexer.getColorRGBA().alpha);
        telemetry.addData("dis", indexer.getDistance());

    }
}
