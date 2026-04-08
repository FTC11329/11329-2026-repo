package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class Intake {
    // declaring motor variables

    DcMotorEx intakeMotor;
    CRServo intakeServo;
    DigitalChannel beamBreak;
    double lastPower = 0;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setCurrentAlert(3, CurrentUnit.AMPS);

        beamBreak = hardwareMap.get(DigitalChannel.class, "intakeSensor");
        beamBreak.setMode(DigitalChannel.Mode.INPUT);

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
    }

    public void intake(boolean set) {
        setIntakePower(set ? Constants.Intake.intakePower : Constants.Intake.intakeOffPower);
    }
    public void spit(boolean set) {
        setIntakePower(set ? Constants.Intake.spitPower : Constants.Intake.intakeOffPower);
    }
    public boolean isBeamBroken() {
        return !beamBreak.getState();
    }

    boolean superSlowOnce = true;
    public void setIntakePower(double set) {
        if (0.15 > set && set > Constants.Intake.intakeOffPower/2.0) {
            if (!superSlowOnce) {
                superSlowOnce = true;
                set = 0;
            }
        } else {
            superSlowOnce = false;
        }
        if (lastPower != set) {
            lastPower = set;
            intakeMotor.setPower(set);
            intakeServo.setPower(set);
        }
    }

    public void update(boolean spitIntake, boolean isIntaking, boolean isShooting, boolean forceSpit, boolean allowIntaking, boolean isPlugged, boolean intakeOverride) {
        if (isPlugged && isBeamBroken()) {
            setIntakePower(Constants.Intake.intakeOffPower/2.0);
        } else if (spitIntake || forceSpit) {
            spit(true);
        } else if ((isIntaking && allowIntaking) || intakeOverride) {
            intake(true);
        } else if (isShooting){
            setIntakePower(Constants.Intake.shootPower);
        } else {
            intake(false);
        }
    }

    public void stop() {
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setPower(0);
    }
}

