package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "Auto Replay", group = "        group")
public class MainAutoReplayTeleop extends OpMode {
    AutoReplayTeleop teleop;

    @Override
    public void init() {
        teleop = new AutoReplayTeleop(hardwareMap, telemetry, gamepad1, gamepad2);
        teleop.init();
    }

    @Override
    public void loop() {
        teleop.loop();
    }

}
