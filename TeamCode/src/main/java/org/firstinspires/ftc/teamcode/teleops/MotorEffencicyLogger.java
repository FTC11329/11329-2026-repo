package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


import java.util.List;

// TODO: add data logging (allocate ~8mb for in ram storage then save to csv at end).
//       create object (data object or sum) to aggregate all data points into one object
//       NOTE: All data stored MUST be in primitives and basic arrays
// TODO: Add Kalman Filter for noisy data? (uh since they have to be tuned)

@TeleOp(name = "Motor Effencincy Logger", group = "Logging")
public final class MotorEffencicyLogger extends LinearOpMode {

    List<LynxModule> allHubs;

    public DcMotorEx flywheel1; // todo: ensure that this has the encoder plugged into it
    public DcMotorEx flywheel2;


    long startingTimeStamp;
    long previousTimeStamp;

    double previousAngularVelocity;

    // 1/2 * mass * radius^2
    public static double MOMENT_OF_INERTIA = 1.7; //todo
    TelemetryManager panelsTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Set up Bulk Reads
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Set up hardware
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        flywheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        clearHubCache();
        startingTimeStamp = timeStamp();
        previousTimeStamp = startingTimeStamp;

        previousAngularVelocity = getAngularVelocity();


        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.aWasPressed()) {
                flywheel1.setPower(1);
                flywheel2.setPower(1);
            }
            panelsTelemetry.addData("Motor Efficiency", motorEfficiency());
            panelsTelemetry.addData("Velocity", getAngularVelocity());
            panelsTelemetry.addData("Acceleration", getAngularAcceleration());
            panelsTelemetry.addData("Mechanical Power Out", powerOut());
            panelsTelemetry.addData("Electrical Power In", powerIn());
            panelsTelemetry.addData("Set Power", getSetPower());
            panelsTelemetry.addData("Current (AMPS)", getMotorCurrent(CurrentUnit.AMPS));
            panelsTelemetry.addData("Estimated Motor Voltage", getMotorVoltage());
            panelsTelemetry.addData("Loop Time", getDeltaTime());
            panelsTelemetry.addData("Hz", 1.0 / getDeltaTime());
            panelsTelemetry.update();

            panelsTelemetry.update();
            clearHubCache();
            previousTimeStamp = timeStamp();
        }
    }

    public void clearHubCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public long timeStamp() {
        return System.nanoTime();
    }

    public long getDeltaTime() {
        return previousTimeStamp - timeStamp();
    }

    public long timeSinceStart() {
        return startingTimeStamp - timeStamp();
    }

    public double getAngularAcceleration() {
        return (previousAngularVelocity - getAngularVelocity()) / getDeltaTime();
    }

    public double getTorque() {
        return MOMENT_OF_INERTIA * getAngularAcceleration();
    }

    public DcMotorEx getEncoder() {
        return flywheel1;
    }

    public VoltageSensor getVoltageSensor() {
        return hardwareMap.voltageSensor.iterator().next();
    }

    public void setShooterPower(double power) {
        flywheel2.setPower(power);
        flywheel1.setPower(power);
    }

    public int getEncoderReading() {
        return getEncoder().getCurrentPosition();
    }

    public double getAngularVelocity() {
        return getEncoder().getVelocity(AngleUnit.RADIANS);
    }

    public double getMotorCurrent(CurrentUnit currentUnit) {
        return getEncoder().getCurrent(currentUnit);
    }

    public double getTotalSystemVoltage() {
        return getVoltageSensor().getVoltage();
    }

    public double getSetPower() {
        return flywheel1.getPower();
    }

    // Note: This is a gross estimation due to many reasons, however I believe it should be sufficient
    // 1: FTC motors are powered by PWM thus "power" isn't constant
    // 2: Not actual motor voltage only a product of system voltage and the "set" power.
    public double getMotorVoltage() {
        return getTotalSystemVoltage() * getSetPower();
    }

    public double powerOut() {
        return getTorque() * getAngularVelocity();
    }

    public double powerIn() {
        return getMotorCurrent(CurrentUnit.AMPS) * getMotorVoltage();
    }

    public double motorEfficiency() {
        return powerOut() / powerIn();
    }

    public static class DataObject {


        public DataObject() {

        }

    }

}

