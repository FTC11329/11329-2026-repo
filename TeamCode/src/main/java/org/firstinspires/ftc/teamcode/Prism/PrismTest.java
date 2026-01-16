package org.firstinspires.ftc.teamcode.Prism;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Prism Artboard Example", group="Linear OpMode")

public class PrismTest extends LinearOpMode {

    PrismAnimations.Solid solid = new PrismAnimations.Solid(Color.BLUE);
    GoBildaPrismDriver prism;

    @Override
    public void runOpMode() {

        /*
         * Initialize the hardware variables. Note that the strings used here must correspond
         * to the names assigned during the robot configuration step on the driver's station.
         */
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        int brightness = 50;
        int start = 0;
        int end = 18;


        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        solid.setBrightness(brightness);
        solid.setStartIndex(start);
        solid.setStopIndex(end);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solid);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.bWasPressed()) {
                solid.setBrightness(brightness);
                solid.setStartIndex(start);
                solid.setStopIndex(end);
                prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solid);
            }
            if (gamepad1.yWasPressed()) {
                brightness += 5;
            }
            if (gamepad1.aWasPressed()) {
                brightness -= 5;
            }
            if (gamepad1.dpadUpWasPressed()) {
                start += 1;
            }
            if (gamepad1.dpadDownWasPressed()) {
                start -= 1;
            }
            if (gamepad1.dpadLeftWasPressed()) {
                end -= 1;
            }
            if (gamepad1.dpadRightWasPressed()) {
                end += 1;
            }
            telemetry.addData("gm", gamepad1.back);
            telemetry.addData("gm", gamepad1.dpadRightWasPressed());
            telemetry.addData("br", brightness);
            telemetry.addData("start", start);
            telemetry.addData("end", end);
            telemetry.update();
            sleep(20);
        }
    }
}