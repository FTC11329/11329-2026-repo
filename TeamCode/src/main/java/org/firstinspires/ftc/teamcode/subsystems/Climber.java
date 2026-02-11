package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Constants;

public class Climber {
    Servo leftClimbServo;
    Servo rightClimbServo;
    public Climber(HardwareMap hardwareMap) {
        leftClimbServo = hardwareMap.get(Servo.class, "leftClimbServo");
        leftClimbServo.setDirection(Servo.Direction.REVERSE);
        rightClimbServo = hardwareMap.get(Servo.class, "rightClimbServo");
        rightClimbServo.setDirection(Servo.Direction.FORWARD);
        leftClimbServo.setPosition(1);
        rightClimbServo.setPosition(1);
    }
    public boolean climb;
    public void enableClimb() {
        setPosition(Constants.Climber.climbedPosition);
    }
    public void disableClimb() {
        setPosition(Constants.Climber.storedPosition);
    }
    double currentPosition = 1;
    public void setPosition(double set) {
        if (currentPosition != set) {
            rightClimbServo.setPosition(set);
            leftClimbServo.setPosition(set);
            currentPosition = set;
        }
    }
}
