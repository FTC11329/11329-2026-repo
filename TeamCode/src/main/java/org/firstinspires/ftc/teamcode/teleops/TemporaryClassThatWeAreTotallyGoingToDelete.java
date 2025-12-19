package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.SmartIndexer;
import org.firstinspires.ftc.teamcode.subsystems.SmartIndexerWNoTRev;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.FancyButton;
import org.firstinspires.ftc.teamcode.util.RobotSide;

@TeleOp(name = "TEST", group = "       group")
public class TemporaryClassThatWeAreTotallyGoingToDelete extends OpMode {
    Servo servo1;
    Servo servo2;
    double postion = 0;

    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, "spindexer1");
        servo2 = hardwareMap.get(Servo.class, "spindexer2");
        servo1.setDirection(Servo.Direction.REVERSE);
        servo2.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            postion += 0.0004;
        } else if (gamepad1.dpad_down){
            postion -= 0.0004;
        }
        telemetry.addData("Pos", postion);
        servo1.setPosition(postion);
        servo2.setPosition(postion);

    }
}
