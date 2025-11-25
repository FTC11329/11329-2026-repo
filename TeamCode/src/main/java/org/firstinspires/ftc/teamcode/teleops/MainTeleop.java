package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.RobotSide;

@TeleOp(name = "Main Teleop", group = "    group")
public class MainTeleop extends OpMode {
    //This is where we introduce the tele-operated controls
    Robot robot;

    double angle = 5;

    @Override
    public void init() {
        robot = new Robot(telemetry, hardwareMap, RobotSide.Blue);
    }

    @Override
    public void loop() {
        robot.update();

        robot.drivetrain.teleopMovement(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, gamepad1.left_bumper);
        if (gamepad1.x){
            robot.indexer.setIndexerPower(1);
        }else{
            robot.indexer.setIndexerPower(-gamepad1.left_stick_y);
        }
        telemetry.addData("Spindexer Power", -gamepad1.left_stick_y);

        if (gamepad1.y){
            robot.indexer.setIndexerToShooterPower(1);
        }else{
            robot.indexer.setIndexerToShooterPower(-gamepad2.right_stick_x);
        }
        telemetry.addData("indexer to shooter power", -gamepad2.right_stick_x);

        telemetry.addData("indexer r", robot.indexer.getColor().red);
        telemetry.addData("indexer g", robot.indexer.getColor().green);
        telemetry.addData("indexer b", robot.indexer.getColor().blue);
        telemetry.addData("indexer a", robot.indexer.getColor().alpha);

        if (gamepad1.b){
            robot.intake.setIntakePower(1);
        }else{
            robot.intake.setIntakePower(gamepad1.right_trigger - gamepad1.left_trigger);
        }
        telemetry.addData("intake Power", gamepad1.right_trigger - gamepad1.left_trigger);

        if (gamepad1.a){
            robot.shooter.setPower(1);
        }
        else {
            robot.shooter.setPower(-gamepad2.left_stick_y);
        }

        telemetry.addData("shooter power", -gamepad2.left_stick_y);

        angle += -gamepad2.right_stick_y * 0.5;
        robot.shooter.setHoodDeg(angle);
        telemetry.addData("shooter angle", angle);

        robot.turret.setPower(gamepad2.left_stick_x);
        telemetry.addData("turret velocity encoder", robot.turret.encoder.getVelocity());
        telemetry.addData("turret position encoder", robot.turret.encoder.getCurrentPosition());


        telemetry.addData("turret power", gamepad2.right_trigger - gamepad2.left_trigger);
    }

}
