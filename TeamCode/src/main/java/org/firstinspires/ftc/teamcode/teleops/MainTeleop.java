package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.ColorFunctions;
import org.firstinspires.ftc.teamcode.util.PressHold;
import org.firstinspires.ftc.teamcode.util.RobotSide;

@TeleOp(name = "Main Teleop", group = "    group")
public class MainTeleop extends OpMode {
    //This is where we introduce the tele-operated controls
    Robot robot;

    PressHold intake;
    PressHold shoot;

    double angle = 5;

    @Override
    public void init() {
        robot = new Robot(telemetry, hardwareMap, RobotSide.Blue);
        intake = new PressHold(PressHold.PressType.DoublePress);
        shoot = new PressHold(PressHold.PressType.LongPress);
    }

    @Override
    public void loop() {
        robot.update();
        shoot.checkStatus(gamepad1.x);
        if (shoot.isOn && intake.isOn){
            intake.checkStatus(false);
            intake.checkStatus(true);
            intake.checkStatus(false);
        }else {
            intake.checkStatus(gamepad1.a);
        }
        robot.drivetrain.teleopMovement(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.left_bumper);

        //TURRET : 2 x
        robot.turret.setPower(gamepad2.left_stick_x);
        telemetry.addData("turret velocity encoder", robot.turret.encoder.getVelocity());
        telemetry.addData("turret position encoder", robot.turret.encoder.getCurrentPosition());

        //INTAKE : a
        robot.intake.setIntakePower(gamepad1.a ? 1 :0);

        //INDEXER : b
        robot.indexer.setIndexerPower(gamepad1.b ? 1 : 0);

        //SHOOTER : y
        if (gamepad1.y){
            robot.passiveShoot(6000, false);
        }

        //Shooter 2 : 2 a, and y
        robot.shooter.setPower(gamepad2.a ? 1 : 0);

        telemetry.addData("Spindexer Power", -gamepad1.left_stick_y);

        //TRANSFER
        robot.indexer.setIndexerToShooterPower(gamepad1.x ? 1 : -gamepad2.right_stick_x);

        telemetry.addData("indexer to shooter power", -gamepad2.right_stick_x);

        telemetry.addData("indexer r", robot.indexer.getColor().red);
        telemetry.addData("indexer g", robot.indexer.getColor().green);
        telemetry.addData("indexer b", robot.indexer.getColor().blue);
        telemetry.addData("indexer a", robot.indexer.getColor().alpha);

        telemetry.addData("intake Power", gamepad1.right_trigger - gamepad1.left_trigger);

        telemetry.addData("shooter power", -gamepad2.left_stick_y);

        angle += -gamepad2.right_stick_y * 0.5;
        robot.shooter.setHoodDeg(angle);
        telemetry.addData("shooter angle", angle);


        telemetry.addData("current", robot.shooter.flywheel.getCurrent(CurrentUnit.AMPS));

        telemetry.addData("Encoder RPM", robot.shooter.getRPM());

        telemetry.addData("turret power", gamepad2.right_trigger - gamepad2.left_trigger);
    }

}
