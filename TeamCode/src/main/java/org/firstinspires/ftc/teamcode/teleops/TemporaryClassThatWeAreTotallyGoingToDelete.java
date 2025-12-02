package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.RobotSide;

@TeleOp(name = "TEST", group = "zgroup")
public class TemporaryClassThatWeAreTotallyGoingToDelete extends OpMode {
    Robot robot;
    TelemetryManager panelsTelemetry;
    @Override
    public void init() {
        robot = new Robot(telemetry, hardwareMap, RobotSide.Blue);
        panelsTelemetry = Panels.getTelemetry();
    }

    @Override
    public void loop() {
        double power1 = -gamepad1.right_stick_y;
        double power2 = -gamepad1.left_stick_y;
        panelsTelemetry.debug("wave: $wave");
        panelsTelemetry.debug("wave2: $wave2");
        panelsTelemetry.graph("wave", power1);
        panelsTelemetry.graph("wave2", power2);
        panelsTelemetry.update(telemetry);
        robot.turret.setPower(power1);
    }
}
