package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.AutoReplayTime;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.FancyButton;
import org.firstinspires.ftc.teamcode.util.RobotSide;

public class AutoReplayTeleop {
    Robot robot;


    FancyButton intake;
    FancyButton shoot;

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
        intake = new FancyButton(FancyButton.PressType.Toggle);
        shoot = new FancyButton(FancyButton.PressType.LongPress);
        autoReplay = new AutoReplayTime(robot.follower, telemetry, gamepadInfo1, gamepad2);
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

        //TURRET
        robot.turret.setPower(gamepad2.left_stick_x);
        telemetry.addData("turret velocity encoder", robot.turret.encoder.getVelocity());
        telemetry.addData("turret position encoder", robot.turret.encoder.getCurrentPosition());

        //INTAKE
        robot.intake.setIntakePower(intake.isOn ? 1 : 0);

        //INDEXER
        robot.indexer.setIndexerPower(intake.isOn ? 0.6 : 0);

        //SHOOTER
        if (shoot.isOn){
            robot.passiveShoot(6000, false);
        }

        robot.drivetrain.teleopMovement(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, gamepad1.left_bumper);
        if (gamepad1.x){
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




        telemetry.addData("turret power", gamepad2.right_trigger - gamepad2.left_trigger);
    }

}
