package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.AutoReplayTime;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.PressHold;
import org.firstinspires.ftc.teamcode.util.RobotSide;

public class AutoReplayTeleop {
    Robot robot;


    PressHold intake;
    PressHold shoot;

    double angle = 5;

    Gamepad gamepad1;
    Gamepad gamepad2;

    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepadInfo1;
    Gamepad gamepadInfo2;
    AutoReplayTime autoReplay;

    public AutoReplayTeleop(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepadInfo1 = gamepad1;
        this.gamepadInfo2 = gamepad2;
    }

    public void init() {
        robot = new Robot(telemetry, hardwareMap, RobotSide.Blue);
        intake = new PressHold(PressHold.PressType.DoublePress);
        shoot = new PressHold(PressHold.PressType.LongPress);
    }

    public void loop() {
        robot.update();

        robot.drivetrain.teleopMovement(gamepadInfo1.right_stick_y, gamepadInfo1.right_stick_x, gamepadInfo1.left_stick_x, gamepadInfo1.left_bumper);

        if (autoReplay.IsReplayOn()){
            gamepad1 = autoReplay.getGamepad1();
            gamepad2 = autoReplay.getGamepad2();
        }
        else{
            gamepad1 = gamepadInfo1;
            gamepad2 = gamepadInfo2;
        }

        shoot.checkStatus(gamepad1.b);
        if (shoot.isOn && intake.isOn){
            intake.checkStatus(false);
            intake.checkStatus(true);
            intake.checkStatus(false);
        }else {
            intake.checkStatus(gamepad1.a);
        }

        //TURRET
        robot.turret.setPower(gamepad2.left_stick_x);
        telemetry.addData("turret velocity encoder", robot.turret.encoder.getVelocity());
        telemetry.addData("turret position encoder", robot.turret.encoder.getCurrentPosition());

        //INTAKE
        robot.intake.setIntakePower(intake.isOn ? 1 : gamepad1.right_trigger - gamepad1.left_trigger);

        //INDEXER
        robot.indexer.setIndexerPower(intake.isOn ? 0.6 : -gamepad1.left_stick_y);

        //SHOOTER
        if (shoot.isOn){
            robot.passiveShoot(3500, false);
        }
        else {
            robot.shooter.setPower(-gamepad2.left_stick_y);
        }
        //TRANSFER
        if (shoot.isOn || gamepad1.y){
            robot.indexer.startTransfer();
        }


        telemetry.addData("Spindexer Power", -gamepad1.left_stick_y);

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


        telemetry.addData("turret power", gamepad2.right_trigger - gamepad2.left_trigger);
    }

}
