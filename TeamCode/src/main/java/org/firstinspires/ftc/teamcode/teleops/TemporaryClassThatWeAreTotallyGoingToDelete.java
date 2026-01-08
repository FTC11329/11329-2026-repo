package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.panels.PanelsConfig;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.RobotSide;

@TeleOp(name = "TEST", group = "       group")
public class TemporaryClassThatWeAreTotallyGoingToDelete extends OpMode {
    CRServo servo1;
    CRServo servo2;
    DcMotorSimple motor1;
    DcMotorSimple motor2;
    Robot robot;
//    Constants.Indexer indexer;
    TelemetryManager panelsTelemetry;
    double power = 1;
    boolean started = false;

    @Override
    public void init() {
        robot = new Robot(telemetry, hardwareMap, RobotSide.Blue, 0, 0);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        motor1 = hardwareMap.get(DcMotorSimple.class, "flywheel");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        motor2 = hardwareMap.get(DcMotorSimple.class, "transfer");
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        servo1 = hardwareMap.get(CRServo.class, "spindexer1");
        servo2 = hardwareMap.get(CRServo.class, "spindexer2");
        servo1.setDirection(CRServo.Direction.REVERSE   );
        servo2.setDirection(CRServo.Direction.FORWARD);
//        motor1.setPower(1);
        motor2.setPower(1);
        servo1.setPower(0);
        servo2.setPower(0);
    }

    @Override
    public void loop() {
        robot.shooter.setHoodDeg(40);
        robot.shooter.setTargetRPM(30000);
        robot.shooter.update();
        panelsTelemetry.addData("rpm", robot.shooter.shooterPID.getPreviousPosition());
        panelsTelemetry.addData("rpm target", robot.shooter.shooterPID.getTargetPosition());
        panelsTelemetry.addData("error", robot.shooter.shooterPID.getError());
        panelsTelemetry.addData("timer", time);
        panelsTelemetry.addData("Power", robot.shooter.shooterPID.run());
        panelsTelemetry.update();
        if (( time > 10) || started){
            servo1.setPower(1);
            servo2.setPower(1);
            started = true;
        }

    }
}
