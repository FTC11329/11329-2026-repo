package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.util.BallColor;

import java.util.Arrays;

public class Lights {
    PrismAnimations.Pulse teamColorAnimation = new PrismAnimations.Pulse();
    PrismAnimations.Pulse cell0 = new PrismAnimations.Pulse();
    PrismAnimations.Pulse cell1 = new PrismAnimations.Pulse();
    PrismAnimations.Pulse cell2 = new PrismAnimations.Pulse();
    PrismAnimations.Solid bigRed = new PrismAnimations.Solid();

    Color teamColor = new Color(52, 153, 255);

    Color purple = new Color(148, 8, 251);
    Color green = new Color(0, 182, 0);

    PrismAnimations.Pulse[] ballCellsAnimation = new PrismAnimations.Pulse[]{
            cell0, cell1, cell2
    };
    GoBildaPrismDriver prism;

    boolean debounce = false;
    int bigRedLoop = 0;
    BallColor[] lastBallColors = new BallColor[] {BallColor.None, BallColor.None, BallColor.None};

    public Lights(HardwareMap hardwareMap) {
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        int brightness = 75;
        int start = 0;
        int end = 18;

        cell0.setIndexes(0, 5);
        cell1.setIndexes(6, 11);
        cell2.setIndexes(12, 17);
        int i = 0;
        prism.clearAllAnimations();
        for (PrismAnimations.Pulse cell : ballCellsAnimation) {
            cell.setBrightness(100);
            cell.setPrimaryColor(teamColor);
            cell.setSecondaryColor(Color.dimColor(teamColor));
            prism.insertAndUpdateAnimation(i, cell);
            i++;
        }
        bigRed.setBrightness(100);
        bigRed.setIndexes(0, 17);
        bigRed.setPrimaryColor(Color.RED);
        teamColorAnimation.setPrimaryColor(teamColor);
        teamColorAnimation.setSecondaryColor(Color.dimColor(teamColor));
        teamColorAnimation.setBrightness(75);
        teamColorAnimation.setIndexes(0, 17);
    }

    public void setBallColors(BallColor[] ballColor) {
        if (Arrays.equals(lastBallColors, ballColor)) {
            return;
        }

        // if empty
        if (lastBallColors[0] != BallColor.None && ballColor[0] == BallColor.None) {
            int i = 0;
            for (PrismAnimations.Pulse cell : ballCellsAnimation) {
                cell.setPrimaryColor(teamColor);
                cell.setSecondaryColor(Color.dimColor(teamColor));
                prism.insertAndUpdateAnimation(i, cell);
                i++;
            }
            lastBallColors =  ballColor;
            return;
        }

        int lastHighestNoneIndex = 0;
        for (int i = 0; i < 3; i++) {
            if (lastBallColors[i] == BallColor.None) {
                lastHighestNoneIndex = i;
                break;
            }
        }

        switch (ballColor[lastHighestNoneIndex]) {
            case Purple:
                ballCellsAnimation[lastHighestNoneIndex].setPrimaryColor(purple);
                ballCellsAnimation[lastHighestNoneIndex].setSecondaryColor(Color.dimColor(purple));
                break;
            case Green:
                ballCellsAnimation[lastHighestNoneIndex].setPrimaryColor(green);
                ballCellsAnimation[lastHighestNoneIndex].setSecondaryColor(Color.dimColor(green));
                break;
            case None:
                ballCellsAnimation[lastHighestNoneIndex].setPrimaryColor(teamColor);
                ballCellsAnimation[lastHighestNoneIndex].setSecondaryColor(Color.dimColor(teamColor));
        }

        if (bigRedLoop == 0 && ballColor[2] != BallColor.None) {
            prism.clearAllAnimations();
            prism.insertAndUpdateAnimation(0, bigRed);
            bigRedLoop++;
        }
        if (bigRedLoop > 0) {
            bigRedLoop++;
        }
        if (bigRedLoop == 0) {
            prism.insertAndUpdateAnimation(lastHighestNoneIndex, ballCellsAnimation[lastHighestNoneIndex]);
            lastBallColors =  ballColor.clone();
        }
        if (bigRedLoop == 4) {
            bigRedLoop = 0;
            prism.clearAllAnimations();
            for (int i = 0; i < 3; i++) {
                prism.insertAndUpdateAnimation(i, ballCellsAnimation[i]);
            }
            lastBallColors =  ballColor.clone();
        }

    }
    public void setColororsmthidk() {
        if (!debounce) {
            prism.clearAllAnimations();
            prism.insertAndUpdateAnimation(0, teamColorAnimation);
        }
        debounce = true;
    }

    public void update() {
        debounce = false;
    }

}
