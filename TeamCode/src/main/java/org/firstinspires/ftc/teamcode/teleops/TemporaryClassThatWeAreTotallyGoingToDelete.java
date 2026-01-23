package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.panels.PanelsConfig;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.RobotSide;

@TeleOp(name = "TEST", group = "       group")
public class TemporaryClassThatWeAreTotallyGoingToDelete extends OpMode {
    ServoImplEx servo1;

    ServoImplEx servo2;
    DcMotorEx flywheel;
    DcMotorEx feeder;
    DcMotorEx intakeMotor;


    Robot robot;
//    Constants.Indexer indexer;
    TelemetryManager panelsTelemetry;
    double pos = 0;
    boolean started = false;


    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;
    DcMotorEx rightBack;
    @Override
    public void init() {
//        robot = new Robot(telemetry, hardwareMap, RobotSide.Blue, 0, 0);
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        feeder = hardwareMap.get(DcMotorEx.class, "transfer");
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        servo1 = hardwareMap.get(ServoImplEx.class, "spindexer1");
        servo2 = hardwareMap.get(ServoImplEx.class, "spindexer2");
        servo1.setDirection(Servo.Direction.REVERSE);
        servo2.setDirection(Servo.Direction.REVERSE);
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        servo1.setPosition(0);
        servo2.setPosition(0);
        servo1.setPwmRange(new PwmControl.PwmRange(542, 2450)); // probably the wrong way to l do it on but it works
        servo2.setPwmRange(new PwmControl.PwmRange(542, 2450));
        flywheel.setPower(.3);

        // telling each drive motor that it is in fact, a motor
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        // setting the motor direction to go correctly
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBack.setPower(1);
        leftFront.setPower(1);
        rightBack.setPower(1);
        rightFront.setPower(1);
        intakeMotor.setPower(1);
        feeder.setPower(1);


    }

    @Override
    public void loop() {
        telemetry.addData("rpm", flywheel.getVelocity() * 60 / Constants.Shooter.ticksPerRevolution);
    }
}
