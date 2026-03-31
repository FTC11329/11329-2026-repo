package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.BallColor;

import java.util.List;

@TeleOp(name = "Color Sensor Reader")
public class ReadColorSensor extends OpMode {
    RevColorSensorV3 colorI2C;
    DigitalChannel color6;
    DigitalChannel color7;
    AnalogInput analog2;
    AnalogInput analog3;
    TelemetryManager panelsTelemetry;
    Indexer indexer;


    @Override
    public void init() {
        indexer = new Indexer(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        colorI2C = hardwareMap.get(RevColorSensorV3.class, "ColorI2C");
        analog2 = hardwareMap.get(AnalogInput.class, "spindexerAnalog2");
        analog3 = hardwareMap.get(AnalogInput.class, "spindexerAnalog3");
        color6 = hardwareMap.get(DigitalChannel.class, "spindexerColor6");
        color7 = hardwareMap.get(DigitalChannel.class, "spindexerColor7");
        color6.setMode(DigitalChannel.Mode.OUTPUT);
        color7.setMode(DigitalChannel.Mode.OUTPUT);
    }

    @Override
    public void loop() {

//        boolean hue = color6.getState();
//        boolean distance = color7.getState();
//        panelsTelemetry.addData("Digital Hue", hue);
//        panelsTelemetry.addData("Digital Distance", distance);
//        boolean green  = hue && distance;
//        boolean purple = !hue && distance;
//        panelsTelemetry.addData("Green", green);
//        panelsTelemetry.addData("Purple", purple);
//        panelsTelemetry.addLine("");


        panelsTelemetry.addData("vol", analog2.getVoltage());
        panelsTelemetry.addData("Dis", analog2.getVoltage() / 3.3 * 100);
        panelsTelemetry.addData("Green T/F", analog2.getVoltage() / 3.3 / 100 > 15 ? 1 : 0);
        panelsTelemetry.addData("Purple T/F", analog3.getVoltage() > 2.0 ? 1 : 0);

//        panelsTelemetry.addData("Analog 2", analog2.getVoltage());
//        panelsTelemetry.addData("Analog 2 Hue Converted", analog2.getVoltage() / 3.3 * 360);
//        panelsTelemetry.addData("Analog 2 Color Converted", analog2.getVoltage() / 3.3 * 255);
//        panelsTelemetry.addData("Analog 2 Distance Converted", analog2.getVoltage() / 3.3 * 100);
//        panelsTelemetry.addData("Analog 3", analog3.getVoltage());
//        panelsTelemetry.addData("Analog 3 Hue Converted", analog3.getVoltage() / 3.3 * 360);
//        panelsTelemetry.addData("Analog 3 Color Converted", analog3.getVoltage() / 3.3 * 255);
//        panelsTelemetry.addData("Analog 3 Distance Converted", analog3.getVoltage() / 3.3 * 100);

        double R = 0;
        double G = 0;
        double B = 0;
        double A = 0;
        BallColor color = BallColor.None;
        if (analog2.getVoltage() / 3.3 * 100 < 20 && colorI2C.getDistance(DistanceUnit.INCH) < 1.9) {
            double distanceI2C = colorI2C.getDistance(DistanceUnit.INCH);

            NormalizedRGBA rgba = colorI2C.getNormalizedColors();
            R = rgba.red;
            G = rgba.green;
            B = rgba.blue;

            R *= distanceI2C;
            G *= distanceI2C;
            B *= distanceI2C;
            A *= distanceI2C;

            double max = Math.max(R, Math.max(G, B));

            R /= max;
            G /= max;
            B /= max;

//            if (R > 0.45) {
//                color = BallColor.Purple;
//            } else {
//                color = BallColor.Green;
//            }
        }
        double distanceI2C = colorI2C.getDistance(DistanceUnit.INCH);
        panelsTelemetry.addData("distI2C", distanceI2C);
        panelsTelemetry.addData("Red", R);
        panelsTelemetry.addData("Green", G);
        panelsTelemetry.addData("Blue", B);
        panelsTelemetry.addData("Alpha", A);

        telemetry.addData("color", color);
        panelsTelemetry.update();

    }
}
