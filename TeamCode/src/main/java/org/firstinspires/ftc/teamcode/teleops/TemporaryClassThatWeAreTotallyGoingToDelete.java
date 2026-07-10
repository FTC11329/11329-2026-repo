package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.ColorFunctions;
import org.firstinspires.ftc.teamcode.util.FancyButton;
import org.firstinspires.ftc.teamcode.util.RobotSide;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "TEST", group = "       group")
public class TemporaryClassThatWeAreTotallyGoingToDelete extends OpMode {
    Shooter shooter;
    Turret turret;
    TelemetryManager panelsTelemetry;
    Robot robot;
    Climber climber;
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
    DcMotorEx intakeMotor;

    CRServo turretServo1;
    CRServo turretServo2;
    DcMotorEx rightBack;
    TouchSensor touchSensor;
    double hoodPos = 0;
    double lastTime = 0;
    public DcMotorEx flywheel1;
    public DcMotorEx flywheel2;
    ArrayList<double[]> readList;
    @Override
    public void init() {
        readList = new ArrayList<>();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        indexer = new Indexer(hardwareMap);
        intake = new Intake(hardwareMap);
    }
    double pos = 0;
    double pow = 0;
    @Override
    public void loop() {
        panelsTelemetry.addData("Volt", indexer.distanceAnalog.getVoltage());

        intake.intakeServo(gamepad1.touchpad);
        if (gamepad1.b) {
            addReadEntry(indexer.colorSensorI2C.getNormalizedColors());
            double[] finalRgba = normalizeList();
            panelsTelemetry.addData("R", finalRgba[0]);
            panelsTelemetry.addData("G", finalRgba[1]);
            panelsTelemetry.addData("B", finalRgba[2]);
            panelsTelemetry.addData("A", finalRgba[3]);
            telemetry.addData("color", ColorFunctions.toColor(finalRgba, indexer.colorSensorI2C.getDistance(DistanceUnit.INCH)));
            panelsTelemetry.update();
        }
        if (gamepad1.y) {
            readList.clear();
        }
        intake.intakeServo(gamepad1.touchpad);
    }

    public void addReadEntry(NormalizedRGBA rgba) {
        readList.add(new double[]{rgba.red / rgba.blue, rgba.green / rgba.blue, rgba.blue / rgba.blue, rgba.alpha});
    }

    public double[] normalizeList() {
        double rTot = 0;
        double gTot = 0;
        double bTot = 0;
        double aTot = 0;
        for (double[] rgba : readList) {
            rTot += rgba[0];
            gTot += rgba[1];
            bTot += rgba[2];
            aTot += rgba[3];
        }
        double red   = rTot / readList.size();
        double green = gTot / readList.size();
        double blue  = bTot / readList.size();
        double alpha = aTot / readList.size();

        return new double[]{red, green, blue, alpha};
    }


}
