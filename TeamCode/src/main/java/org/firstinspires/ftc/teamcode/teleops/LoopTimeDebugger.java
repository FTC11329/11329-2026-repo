package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.RobotSide;

import java.util.List;

@TeleOp(name = "Loop Time Debugger", group = "     group")
public class LoopTimeDebugger extends OpMode {
    TelemetryManager panelsTelemetry;

    DcMotorEx intakeMotor;
    DcMotorEx feeder;
    DcMotorEx flywheel;

    DcMotorEx leftFront;
    DcMotorEx leftRear;
    DcMotorEx rightFront;
    DcMotorEx rightRear;

    CRServo turretServo1;
    CRServo turretServo2;
    Servo hoodServo1;
    Servo hoodServo2;

    double lastTime = 0;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        feeder = hardwareMap.get(DcMotorEx.class, "transfer");
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        hoodServo1 = hardwareMap.get(Servo.class, "hood1");
        hoodServo1.setDirection(Servo.Direction.FORWARD);

        hoodServo2 = hardwareMap.get(Servo.class, "hood2");
        hoodServo2.setDirection(Servo.Direction.REVERSE);
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretServo1 = hardwareMap.get(CRServo.class, "turret1");
        turretServo1.setDirection(CRServo.Direction.FORWARD);

        turretServo2 = hardwareMap.get(CRServo.class, "turret2");
        turretServo2.setDirection(CRServo.Direction.FORWARD);
        lastTime = System.nanoTime();
    }

    @Override
    public void loop() {

    }
}
