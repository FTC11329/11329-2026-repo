package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PressHold {

    public PressType type;

    public boolean isOn;
    public boolean isPressed;

    public boolean startPress;
    public boolean endPress;

    public ElapsedTime time;

    public PressHold(PressType type){
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

        if (type == PressType.DoublePress){
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
        else if (type == PressType.LongPress){
            if (pressed && !isPressed){
                startPress = true;
                isOn = true;
                time.reset();
            }
            else if (!pressed && isPressed){
                endPress = true;
                isOn = false;
                time.reset();
            }
        }

        isPressed = pressed;
    }

    public void resetTimer() {
        time.reset();
    }

    public enum PressType{
        DoublePress,
        LongPress
    }
}
