package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Clear Auto Values", group = "                                              group")
public class ClearAutoValues extends OpMode {
    EndValuesStorer endValuesStorer = new EndValuesStorer();


    @Override
    public void init() {
        telemetry.addLine("Hit start to clear auto values");
    }

    @Override
    public void start() {
        endValuesStorer.saveEndValues(0,0,0,0,0);
    }

    @Override
    public void loop() {
        requestOpModeStop();
    }
}
