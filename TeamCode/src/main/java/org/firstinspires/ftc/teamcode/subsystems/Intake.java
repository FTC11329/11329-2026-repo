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
    double lastMotorPower = 0;
    double lastServoPower = 0;

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

    public void intakeMotor(boolean set) {
        setIntakeMotorPower(set ? Constants.Intake.intakeMotorPower : Constants.Intake.intakeMotorOffPower);
    }
    public void spitMotor() {
        setIntakeMotorPower(Constants.Intake.spitMotorPower);
    }
    public void intakeServo(boolean set) {
        setIntakeServoPower(set ? Constants.Intake.intakeServoPower : Constants.Intake.intakeServoOffPower);
    }
    public void spitServo() {
        setIntakeServoPower(Constants.Intake.spitServoPower);
    }
    public boolean isBeamBroken() {
        return !beamBreak.getState();
    }

    boolean superSlowOnce = true;
    public void setIntakeMotorPower(double set) {
        if (0.15 > set && set > Constants.Intake.intakeMotorOffPower /2.0) {
            if (!superSlowOnce) {
                superSlowOnce = true;
                set = 0;
            }
        } else {
            superSlowOnce = false;
        }
        if (lastMotorPower != set) {
            lastMotorPower = set;
            intakeMotor.setPower(set);
        }
    }

    public void setIntakeServoPower(double set) {
        if (lastServoPower != set) {
            lastServoPower = set;
            intakeServo.setPower(set);
        }
    }


        public void update(boolean spitIntake, boolean isIntaking, boolean isShooting, boolean forceSpit, boolean allowIntaking, boolean isPlugged, boolean intakeOverride) {
        if (isPlugged && isBeamBroken()) {
            setIntakeMotorPower(Constants.Intake.intakeMotorPluggedPower);
            setIntakeServoPower(0);
        } else if (spitIntake || forceSpit) {
            spitMotor();
            spitServo();
        } else if ((isIntaking && allowIntaking) || intakeOverride) {
            intakeMotor(true);
            intakeServo(true);
        } else if (isShooting){
            setIntakeMotorPower(Constants.Intake.intakeMotorOffPower);
            intakeServo(true);
        } else {
            intakeMotor(false);
            intakeServo(false);
        }
    }

    public void stop() {
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setPower(0);
    }
}

