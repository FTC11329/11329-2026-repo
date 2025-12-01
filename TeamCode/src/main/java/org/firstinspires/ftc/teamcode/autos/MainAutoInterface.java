package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.RobotSide;

@Autonomous(name = "Main Auto", group = "    group")
public class MainAutoInterface extends OpMode {
    //This is where we introduce the tele-operated controls
    MainAuto auto;

    @Override
    public void init() {
        auto = new MainAuto(hardwareMap, telemetry, RobotSide.Blue);
        auto.init();
    }

    @Override
    public void start(){
        auto.start();
    }

    @Override
    public void loop() {
        auto.loop();
    }

}
