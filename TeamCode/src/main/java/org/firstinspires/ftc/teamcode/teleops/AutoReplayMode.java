package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.util.RobotSide;

@TeleOp(name = "Auto Replay Time", group = "zgroup")
public class AutoReplayMode extends OpMode {

    AutoReplayTeleop autoReplay;

    @Override
    public void init() {
        autoReplay = new AutoReplayTeleop(hardwareMap, telemetry, gamepad1, gamepad2, RobotSide.Blue);
        autoReplay.init();
    }


    @Override
    public void init_loop(){
        autoReplay.init_loop();
    }
    @Override
    public void start(){
        autoReplay.start();
    }

    @Override
    public void loop() {
        autoReplay.loop();
    }


    @Override
    public void stop() { autoReplay.stop(); }
}
