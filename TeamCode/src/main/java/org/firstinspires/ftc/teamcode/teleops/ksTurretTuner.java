package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.RobotSide;
import org.firstinspires.ftc.teamcode.util.ShapeDetection;
import org.firstinspires.ftc.teamcode.util.Tuple;
import org.openftc.easyopencv.OpenCvCamera;

import java.sql.Time;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "ks Turret Tuner", group = "    group")
public class ksTurretTuner extends OpMode {
    ArrayList<Tuple> posVsPowList = new ArrayList<>();
    Turret turret;
    Shooter shooter;
    @Override
    public void init() {
        turret = new Turret(hardwareMap, 0, RobotSide.Blue);
        shooter = new Shooter(hardwareMap);
    }

    double DELTA_POSITION_DEG = 10;
    double lastTime = System.nanoTime();
    boolean stopped = false;
    boolean usePid = false;
    double power = 0.02;
    int state = 2;
    Timer stateTimer = new Timer();
    @Override
    public void loop() {
        if (stopped) {
            turret.setPower(0);
            for (Tuple i : posVsPowList) {
                telemetry.addLine("pos:" + i.get1() + ", pow:" + i.get2());
            }
            telemetry.update();
            return;
        }
        if (gamepad1.a) {
            stopped = true;
            return;
        }
        switch (state) {
            case 0:
                if (stateTimer.getElapsedTime() > 90) {
                    power += 0.001;
                    stateTimer.resetTimer();
                }
                if ((Math.abs(turret.getVelocity()) > 1)) {
                    state = 1;
                }
                break;
            case 1:
                posVsPowList.add(new Tuple(Math.round(turret.getAngle() * 10) / 10.0, Math.round(power * 1000) / 1000.0));
                power += 0.2;
                state = 2;
                stateTimer.resetTimer();
                break;
            case 2:
                if (stateTimer.getElapsedTimeSeconds() > 0.05) {
                    stateTimer.resetTimer();
                    power = 0;
                    state = 3;
                }
                break;
            case 3:
                if (stateTimer.getElapsedTimeSeconds() > 0.6) {
                    stateTimer.resetTimer();
                    power = 0.02;
                    state = 0;
                }
                break;
        }

//        shooter.setPower(0.7);
        turret.update(0,0,0,false);
        turret.setPower(power);
        telemetry.addData("pow", power);
        telemetry.addData("vel", turret.getVelocity());

    }
}
