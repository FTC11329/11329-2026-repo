package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "Shooter", group = "group")
public class ShooterTrial extends OpMode {
    //This is where we introduce the tele-operated controls
    Shooter shooter;

    ElapsedTime time;
    double deltaTime;
    double lastTime;

    @Override
    public void init() {
        //do stuff init
        shooter = new Shooter(hardwareMap);
    }

    @Override
    public void start(){
        time = new ElapsedTime();
        time.reset();
        lastTime = time.milliseconds();
    }

    @Override
    public void loop() {
        double shooterSpeed = gamepad1.right_stick_y;
        shooter.setPower(shooterSpeed);
        if (gamepad1.a) {
            shooter.setHoodDeg(80);

        } else if (gamepad1.b) {
            shooter.setHoodDeg(60);

        } else if (gamepad1.x) {
            shooter.setHoodDeg(40);

        } else if (gamepad1.y) {
            shooter.setHoodDeg(20);

        } else {
            shooter.setHoodDeg(0);
        }
        telemetry.addData("deg", shooter.getHoodPos());
        deltaTime = time.milliseconds() - lastTime
        telemetry.addData("delta", deltaTime);
        lastTime = time.milliseconds();
    }
}
