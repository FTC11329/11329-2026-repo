package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.RobotSide;

@TeleOp(name = "Get Shooter Vals Blue", group = "                                 group")
public class TestingShooterValuesBlue extends OpMode {
    TestingShooterValues testingShooterValues;

    @Override
    public void init() {
        testingShooterValues = new TestingShooterValues(gamepad1, gamepad2, telemetry, hardwareMap, RobotSide.Blue);
        testingShooterValues.init();
    }

    @Override
    public void init_loop() {
        testingShooterValues.init_loop();
    }

    @Override
    public void loop() {
        testingShooterValues.loop();
    }

    @Override
    public void stop() {
        testingShooterValues.stop();
    }
}
