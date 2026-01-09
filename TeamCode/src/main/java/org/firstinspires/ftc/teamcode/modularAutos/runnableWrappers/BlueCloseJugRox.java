package org.firstinspires.ftc.teamcode.modularAutos.runnableWrappers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.RobotSide;

@Autonomous
public class BlueCloseJugRox extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(telemetry, hardwareMap, RobotSide.Blue, 0,0,
                new BallColor[]{
                        BallColor.Green,
                        BallColor.Purple,
                        BallColor.Purple
                });

    }

    @Override
    public void loop() {

    }
}
