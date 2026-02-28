package org.firstinspires.ftc.teamcode.teleops;

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

import java.util.List;

@TeleOp(name = "TEST", group = "       group")
public class TemporaryClassThatWeAreTotallyGoingToDelete extends OpMode {
    Shooter shooter;
    Robot robot;
    List<LynxModule> hubs;
    Indexer indexer;
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
    double hoodPos = 5;
    TouchSensor touchSensor;
    double lastTime = 0;
    @Override
    public void init() {
        analog2 = hardwareMap.get(AnalogInput.class, "spindexerAnalog2");
        analog3 = hardwareMap.get(AnalogInput.class, "spindexerAnalog3");
        color6 = hardwareMap.get(DigitalChannel.class, "spindexerColor6");
        color7 = hardwareMap.get(DigitalChannel.class, "spindexerColor7");
        color6.setMode(DigitalChannel.Mode.OUTPUT);
        color7.setMode(DigitalChannel.Mode.OUTPUT);
        lastTime = System.nanoTime();
    }

    @Override
    public void loop() {

        double startTime = System.nanoTime();

        telemetry.addData("Digital Hue", color6.getState());
        telemetry.addData("Digital Distance ", color7.getState());

        telemetry.addData("Analog Hue", analog2.getVoltage());
        telemetry.addData("Analog Distance", analog3.getVoltage());

        double loop = (System.nanoTime() - startTime) * 1e-9;

    }
}
