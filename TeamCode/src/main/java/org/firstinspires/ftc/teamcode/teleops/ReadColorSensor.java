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

import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.BallColor;

import java.util.List;

@TeleOp(name = "Color Sensor Reader")
public class ReadColorSensor extends OpMode {
    DigitalChannel color6;
    DigitalChannel color7;
    AnalogInput analog2;
    AnalogInput analog3;
    TelemetryManager panelsTelemetry;


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        analog2 = hardwareMap.get(AnalogInput.class, "spindexerAnalog2");
        analog3 = hardwareMap.get(AnalogInput.class, "spindexerAnalog3");
        color6 = hardwareMap.get(DigitalChannel.class, "spindexerColor6");
        color7 = hardwareMap.get(DigitalChannel.class, "spindexerColor7");
        color6.setMode(DigitalChannel.Mode.OUTPUT);
        color7.setMode(DigitalChannel.Mode.OUTPUT);
    }

    @Override
    public void loop() {

        boolean hue = color6.getState();
        boolean distance = color7.getState();
        panelsTelemetry.addData("Digital Hue", hue);
        panelsTelemetry.addData("Digital Distance", distance);
        boolean green  = hue && distance;
        boolean purple = !hue && distance;
        panelsTelemetry.addData("Green", green);
        panelsTelemetry.addData("Purple", purple);
        panelsTelemetry.addLine("");


        panelsTelemetry.addData("Green T/F", analog2.getVoltage() > 2.0);
        panelsTelemetry.addData("Purple T/F", analog3.getVoltage() > 2.0);

        panelsTelemetry.addData("Analog 2", analog2.getVoltage());
        panelsTelemetry.addData("Analog 2 Hue Converted", analog2.getVoltage() / 3.3 * 360);
        panelsTelemetry.addData("Analog 2 Color Converted", analog2.getVoltage() / 3.3 * 255);
        panelsTelemetry.addData("Analog 2 Distance Converted", analog2.getVoltage() / 3.3 * 100);
        panelsTelemetry.addData("Analog 3", analog3.getVoltage());
        panelsTelemetry.addData("Analog 3 Hue Converted", analog3.getVoltage() / 3.3 * 360);
        panelsTelemetry.addData("Analog 3 Color Converted", analog3.getVoltage() / 3.3 * 255);
        panelsTelemetry.addData("Analog 3 Distance Converted", analog3.getVoltage() / 3.3 * 100);

        double act;
        if (analog3.getVoltage() < 2.0) {
            act = 0;
        } else {
            act = analog2.getVoltage() / 3.3 * 255;
        }
        BallColor ball;
        if (act < 10) {
            ball = BallColor.None;
        } else if (act < 92) {
            ball = BallColor.Green;
        } else {
            ball = BallColor.Purple;
        }
        panelsTelemetry.addData("Tune me", act);
        telemetry.addData("Ball", ball);
        panelsTelemetry.update();

    }
}
