package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Auto Replay Time", group = "      group")
public class AutoReplayMode extends OpMode {

    AutoReplayTeleop autoReplay;

    @Override
    public void init() {
        autoReplay = new AutoReplayTeleop(hardwareMap, telemetry, gamepad1, gamepad2);
        autoReplay.init();
    }

    @Override
    public void loop() {
        autoReplay.loop();
    }

}
