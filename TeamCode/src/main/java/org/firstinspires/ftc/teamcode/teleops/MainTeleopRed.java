package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.RobotSide;

@TeleOp(name = "Main Teleop Red", group = "                                                      group")
public class MainTeleopRed extends OpMode {
    MainTeleop mainTeleop;

    @Override
    public void init() {
        mainTeleop = new MainTeleop(gamepad1, gamepad2, telemetry, hardwareMap, RobotSide.Red);
        mainTeleop.init();
    }

    @Override
    public void loop() {
        mainTeleop.loop();
    }

    @Override
    public void stop() {
        mainTeleop.stop();
    }
}
