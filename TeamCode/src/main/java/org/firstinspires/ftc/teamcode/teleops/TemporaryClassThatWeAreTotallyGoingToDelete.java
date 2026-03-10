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
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.FancyButton;
import org.firstinspires.ftc.teamcode.util.RobotSide;

import java.util.List;

@TeleOp(name = "TEST", group = "       group")
public class TemporaryClassThatWeAreTotallyGoingToDelete extends OpMode {
    Shooter shooter;
    Turret turret;
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
    public DcMotorEx flywheel1;
    public DcMotorEx flywheel2;
    @Override
    public void init() {
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");

        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel1.setCurrentAlert(4, CurrentUnit.AMPS);

        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel2.setCurrentAlert(4, CurrentUnit.AMPS);

    }

    @Override
    public void loop() {
        flywheel1.setPower(.12);
        flywheel2.setPower(.12);
//        telemetry.addData("ticks", turret.getTicks());

    }
}
