package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class FancyButton {

    public PressType type;

    public boolean isOn;
    public boolean isPressed;

    public boolean startPress;
    public boolean endPress;

    public ElapsedTime time;

    public FancyButton(PressType type){
        this.type = type;
        isOn = false;
        isPressed = false;
        startPress = false;
        endPress = false;
        time = new ElapsedTime();
    }

    public void checkStatus(boolean pressed){
        if (startPress) startPress = false;
        if (endPress) endPress = false;

        // A toggle button and you can see time since it changed from false to true
        if (type == PressType.Toggle){
            if (pressed && !isPressed){
                if (!isOn){
                    startPress = true;
                    isOn = true;
                    time.reset();
                }else {
                    endPress = true;
                    isOn = false;
                }
            }
        }
        // Just a regular button but you can see time since start press
        else if (type == PressType.LongPress) {
            if (pressed && !isPressed){
                startPress = true;
                isOn = true;
                time.reset();
            }
            else if (!pressed && isPressed) {
                endPress = true;
                isOn = false;
            }
        }

        isPressed = pressed;
    }

    public void resetTimer() {
        time.reset();
    }

    public enum PressType{
        Toggle,
        LongPress
    }
}
