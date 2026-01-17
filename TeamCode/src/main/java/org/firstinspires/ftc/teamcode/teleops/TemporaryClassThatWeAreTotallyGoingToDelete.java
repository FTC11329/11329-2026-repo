package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.panels.PanelsConfig;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.RobotSide;

@TeleOp(name = "TEST", group = "       group")
public class TemporaryClassThatWeAreTotallyGoingToDelete extends OpMode {
    ServoImplEx servo1;
    ServoImplEx servo2;
    Robot robot;
//    Constants.Indexer indexer;
    TelemetryManager panelsTelemetry;
    double pos = 0;
    boolean started = false;

    @Override
    public void init() {
//        robot = new Robot(telemetry, hardwareMap, RobotSide.Blue, 0, 0);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        servo1 = hardwareMap.get(ServoImplEx.class, "spindexer1");
        servo2 = hardwareMap.get(ServoImplEx.class, "spindexer2");
        servo1.setDirection(Servo.Direction.REVERSE);
        servo2.setDirection(Servo.Direction.REVERSE);
        servo1.setPosition(0);
        servo2.setPosition(0);
        servo1.setPwmRange(new PwmControl.PwmRange(542, 2450)); // probably the wrong way to l do it on but it works
        servo2.setPwmRange(new PwmControl.PwmRange(542, 2450));
    }

    @Override
    public void loop() {

        if (gamepad1.dpadLeftWasPressed()) {
            pos -= 0.1666666666666;
        }
        if (gamepad1.dpadRightWasPressed()) {
            pos += 0.1666666666666;
        }
        telemetry.addData("pos", pos);
        servo1.setPosition(pos);
        servo2.setPosition(pos);
    }
}
