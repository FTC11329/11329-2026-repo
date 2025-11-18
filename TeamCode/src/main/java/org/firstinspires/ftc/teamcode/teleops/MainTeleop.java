package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
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
        robot.indexer.setSpindexerPower(-gamepad1.left_stick_y);
        telemetry.addData("Spindexer Power", -gamepad1.left_stick_y);

        robot.indexer.setIndexerToShooterPower(-gamepad1.right_stick_y);
        telemetry.addData("indexer to shooter power", -gamepad1.right_stick_y);

        telemetry.addData("indexer r", robot.indexer.getColor().red);
        telemetry.addData("indexer g", robot.indexer.getColor().green);
        telemetry.addData("indexer b", robot.indexer.getColor().blue);
        telemetry.addData("indexer a", robot.indexer.getColor().alpha);

        robot.intake.setintakePower(gamepad1.right_trigger - gamepad1.left_trigger);
        telemetry.addData("intake Power", gamepad1.right_trigger - gamepad1.left_trigger);

        robot.shooter.setPower(-gamepad2.left_stick_y);
        telemetry.addData("shooter power", -gamepad2.left_stick_y);

        angle += -gamepad2.right_stick_y * 0.5;
        robot.shooter.setHoodDeg(angle);
        telemetry.addData("shooter angle", angle);

        robot.turret.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
        telemetry.addData("turret power", gamepad2.right_trigger - gamepad2.left_trigger);
    }

}
