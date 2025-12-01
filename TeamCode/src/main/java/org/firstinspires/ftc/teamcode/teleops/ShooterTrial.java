package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "Shooter Test", group = "group")
public class ShooterTrial extends OpMode {
    //This is where we introduce the tele-operated controls
    Shooter shooter;

    ElapsedTime time;
    double deltaTime;
    double lastTime;
    public CRServo shooterServo;

    @Override
    public void init() {
        //do stuff init
        shooter = new Shooter(hardwareMap);
        shooterServo = hardwareMap.get(CRServo.class, "shooterServo");
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
        if (gamepad1.left_bumper){
            shooter.setPower(1);
        }
        if (gamepad1.right_bumper){
            shooterServo.setPower(1);
        }

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
        telemetry.addData("deg", shooter.getHoodPosDeg());
        telemetry.addData("current", shooter.flywheel.getCurrent(CurrentUnit.AMPS));
        deltaTime = time.milliseconds() - lastTime;
        telemetry.addData("delta Time", deltaTime);
        telemetry.addData("Encoder RPM", shooter.getRPM());
        lastTime = time.milliseconds();
    }
}
