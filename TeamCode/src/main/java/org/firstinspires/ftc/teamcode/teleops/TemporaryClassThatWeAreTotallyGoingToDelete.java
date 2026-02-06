package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.List;

@TeleOp(name = "TEST", group = "       group")
public class TemporaryClassThatWeAreTotallyGoingToDelete extends OpMode {
    Shooter shooter;
    List<LynxModule> hubs;
    TelemetryManager panelsTelemetry;

    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;

    CRServo turretServo1;
    CRServo turretServo2;
    DcMotorEx rightBack;
    double hoodPos = 5;
    TouchSensor touchSensor;
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        shooter = new Shooter(hardwareMap);
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        // setting the motor direction to go correctly
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        turretServo1 = hardwareMap.get(CRServo.class, "turret1");
        turretServo1.setDirection(CRServo.Direction.FORWARD);

        turretServo2 = hardwareMap.get(CRServo.class, "turret2");
        turretServo2.setDirection(CRServo.Direction.FORWARD);

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
//
        shooter.setPower(0.666667);
        shooter.update();

        long time1 = System.nanoTime();



//
//        if (System.currentTimeMillis() % 17 == 0) {
//            shooter.setHoodDeg(hoodPos);
//        }
//        leftBack.setPower(0.1);
//        leftFront.setPower(0.1);
//        rightBack.setPower(0.1);
//        rightFront.setPower(0.1);
//
//        turretServo1.setPower(hoodPos - 5);
//        turretServo2.setPower(hoodPos - 5);

        long time2 = System.nanoTime();
        panelsTelemetry.addData("volt", getBatteryVoltage());
        panelsTelemetry.addData("prep shooter", (time2 - time1) * 1e-6);
        panelsTelemetry.update();
    }
    double lastTime = System.currentTimeMillis();
    double lastVolt = 0;
    public double getBatteryVoltage() {
        if (System.currentTimeMillis() - lastTime < 1000) {
            return lastVolt;
        }
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        lastTime = System.currentTimeMillis();
        lastVolt = result;
        return result;
    }

}
