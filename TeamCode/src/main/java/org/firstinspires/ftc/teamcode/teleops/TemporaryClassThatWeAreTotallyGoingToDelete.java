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
import org.firstinspires.ftc.teamcode.subsystems.Climber;
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
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        turret = new Turret(hardwareMap, 0, RobotSide.Blue);
    }
    double pos = 0;
    @Override
    public void loop() {
        pos = gamepad1.left_stick_y * 180;
        turret.setTargetDeg(pos);
        turret.update(0, 0);
        panelsTelemetry.addData("turret pow", turret.turretPID.run());
        panelsTelemetry.addData("turret pos", turret.getAngle());
        panelsTelemetry.addData("turret tar", turret.turretPID.getTargetPosition());
        panelsTelemetry.update();
    }

}
