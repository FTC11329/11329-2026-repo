package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.util.BallColor;

import java.util.Arrays;

public class Lights {
    PrismAnimations.DroidScan random = new PrismAnimations.DroidScan();
    PrismAnimations.Solid cell0 = new PrismAnimations.Solid();
    PrismAnimations.Solid cell1 = new PrismAnimations.Solid();
    PrismAnimations.Solid cell2 = new PrismAnimations.Solid();
    PrismAnimations.Solid bigRed = new PrismAnimations.Solid();

    Color teamColor = new Color(52, 153, 204);

    PrismAnimations.Solid[] ballCellsAnimation = new PrismAnimations.Solid[]{
            cell0, cell1, cell2
    };
    GoBildaPrismDriver prism;

    boolean debounce = false;
    long hasTurnedRedTime = -1000;
    int loopNumber = 0;
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
        for (PrismAnimations.Solid cell : ballCellsAnimation) {
            cell.setPrimaryColor(Color.TRANSPARENT);
            prism.insertAndUpdateAnimation(i, cell);
            i++;
        }
        bigRed.setBrightness(100);
        bigRed.setIndexes(0, 18);
        bigRed.setPrimaryColor(Color.RED);
        cell1.setPrimaryColor(Color.ORANGE);
        cell2.setPrimaryColor(Color.BLUE);
    }

    public void setBallColors(BallColor[] ballColor) {
        if (Arrays.equals(lastBallColors, ballColor)) {
            return;
        }

        // if empty
        if (lastBallColors[0] != BallColor.None && ballColor[0] == BallColor.None) {
            int i = 0;
            for (PrismAnimations.Solid cell : ballCellsAnimation) {
                cell.setPrimaryColor(Color.TRANSPARENT);
                prism.insertAndUpdateAnimation(i, cell);
                i++;
            }
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
                ballCellsAnimation[lastHighestNoneIndex].setPrimaryColor(new Color(148, 8, 251));
                break;
            case Green:
                ballCellsAnimation[lastHighestNoneIndex].setPrimaryColor(new Color(0, 182, 0));
                break;
            default:
                throw new RuntimeException("WHy you empty bro??");
        }

        prism.insertAndUpdateAnimation(lastHighestNoneIndex, ballCellsAnimation[lastHighestNoneIndex]);

        loopNumber ++;
        lastBallColors = ballColor;
    }
    public void setColororsmthidk() {
        if (!debounce) {
            prism.insertAndUpdateAnimation(0, random);
        }
        debounce = true;
    }

    public void update() {
        debounce = false;
    }

}
