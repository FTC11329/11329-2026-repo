package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Constants;

public class Climber {
    Servo leftClimbServo;
    Servo rightClimbServo;
    double currentPosition = 0;
    public Climber(HardwareMap hardwareMap) {
        leftClimbServo = hardwareMap.get(Servo.class, "leftClimbServo");
        leftClimbServo.setDirection(Servo.Direction.FORWARD);
        rightClimbServo = hardwareMap.get(Servo.class, "rightClimbServo");
        rightClimbServo.setDirection(Servo.Direction.REVERSE);
        leftClimbServo.setPosition(0);
        rightClimbServo.setPosition(0);
    }
    public void enableClimb() {
        setPosition(Constants.Climber.climbedPosition);
    }
    public void disableClimb() {
        setPosition(Constants.Climber.storedPosition);
    }
    public void setPosition(double set) {
        if (currentPosition != set) {
            rightClimbServo.setPosition(set);
            leftClimbServo.setPosition(set);
            currentPosition = set;
        }
    }
}
