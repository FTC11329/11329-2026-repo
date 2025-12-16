package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SmartIndexer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.RobotSide;

@TeleOp(name = "TEST", group = "       group")
public class TemporaryClassThatWeAreTotallyGoingToDelete extends OpMode {
    TelemetryManager panelsTelemetry;
//    SmartIndexer smartIndexer;
    Turret turret;
    double indexerPos = 0;

    @Override
    public void init() {
        turret = new Turret(hardwareMap, 0, RobotSide.Blue);
//        smartIndexer = new SmartIndexer(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            indexerPos += 0.002;
        }
        if (gamepad1.dpad_down) {
            indexerPos -= 0.002;
        }
//        smartIndexer.setIndexerPos(indexerPos);
        telemetry.addData("POS", indexerPos);
//        NormalizedRGBA rgba = smartIndexer.getColorRGBA();
//        telemetry.addData("R", rgba.red);
//        telemetry.addData("G", rgba.green);
//        telemetry.addData("B", rgba.blue);
//        telemetry.addData("A", rgba.alpha);
        telemetry.addData("TICKS", turret.getTicks());
    }
}
