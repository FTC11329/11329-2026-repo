package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.util.BallColor;

import java.util.Arrays;
import java.util.Random;

public class Lights {
    PrismAnimations.Pulse romaniaFlag0 = new PrismAnimations.Pulse();
    PrismAnimations.Pulse romaniaFlag1 = new PrismAnimations.Pulse();
    PrismAnimations.Pulse romaniaFlag2 = new PrismAnimations.Pulse();
    PrismAnimations.Pulse americaFlag0 = new PrismAnimations.Pulse();
    PrismAnimations.Pulse americaFlag1 = new PrismAnimations.Pulse();
    PrismAnimations.Pulse americaFlag2 = new PrismAnimations.Pulse();
    PrismAnimations.Sparkle teamColorAnimation = new PrismAnimations.Sparkle();
    PrismAnimations.Pulse cell0 = new PrismAnimations.Pulse();
    PrismAnimations.Pulse cell1 = new PrismAnimations.Pulse();
    PrismAnimations.Pulse cell2 = new PrismAnimations.Pulse();
    PrismAnimations.Pulse queueCell0 = new PrismAnimations.Pulse();
    PrismAnimations.Pulse queueCell1 = new PrismAnimations.Pulse();
    PrismAnimations.Pulse queueCell2 = new PrismAnimations.Pulse();

    PrismAnimations.Rainbow climbCell = new PrismAnimations.Rainbow();

    PrismAnimations.Solid bigRed = new PrismAnimations.Solid();

    Color teamColor = new Color(52, 153, 255);

    //documentation: colors. todo: orange and yellow should not be so blue. cheer them up.
    Color purple = new Color(148, 8, 251);
    Color green = new Color(0, 182, 0);
    Color red = new Color(182, 0, 0 );
    Color blue = new Color(0, 0, 182);
    Color orange = new Color(0, 0, 182);
    Color yellow = new Color(0, 0, 182);
    PrismAnimations.Pulse[] ballCellsAnimation = new PrismAnimations.Pulse[]{
            cell0, cell1, cell2
    };
    PrismAnimations.Pulse[] romaniaFlagFull = new PrismAnimations.Pulse[]{
            romaniaFlag0, romaniaFlag1, romaniaFlag2
    };
    PrismAnimations.Pulse[] americaFlagFull = new PrismAnimations.Pulse[]{
            americaFlag0, americaFlag1, americaFlag2
    };
    PrismAnimations.Pulse[] queueCellsAnimation = new PrismAnimations.Pulse[]{
            queueCell0, queueCell1, queueCell2
    };
    GoBildaPrismDriver prism;
    boolean debounce = false;
    boolean climbLights = false;
    boolean lastSmartShoot = false;
    boolean lastBasicallyHas3 = false;
    boolean bigRedBoolean = false;
    Timer bigRedTimer = new Timer();
    BallColor[] lastBallColors = new BallColor[] {BallColor.None, BallColor.None, BallColor.None};
    BallColor[] lastQueuedBallColors = new BallColor[] {BallColor.None, BallColor.None, BallColor.None};

    public Lights(HardwareMap hardwareMap) {
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        int brightness = 100;
        int start = 0;
        int end = 18;

        cell0.setIndexes(0, 5);
        cell1.setIndexes(6, 11);
        cell2.setIndexes(12, 17);
        //documentation: Romanian flag
        romaniaFlag0.setIndexes(0, 5);
        romaniaFlag1.setIndexes(6, 11);
        romaniaFlag2.setIndexes(12, 17);
        romaniaFlag0.setPrimaryColor(Color.RED);
        romaniaFlag0.setSecondaryColor(Color.dimColor(Color.RED));
        romaniaFlag1.setSecondaryColor(new Color(255, 230, 0));
        romaniaFlag1.setPrimaryColor(Color.dimColor(new Color(255, 230, 0)));
        romaniaFlag2.setPrimaryColor(Color.BLUE);
        romaniaFlag2.setSecondaryColor(Color.dimColor(Color.BLUE));

        //documentation: american flag
        americaFlag0.setIndexes(0, 5);
        americaFlag1.setIndexes(6, 11);
        americaFlag2.setIndexes(12, 17);
        americaFlag0.setPrimaryColor(Color.RED);
        americaFlag0.setSecondaryColor(Color.dimColor(Color.RED));
        americaFlag1.setPrimaryColor(Color.WHITE);
        americaFlag1.setSecondaryColor(Color.dimColor(Color.WHITE));
        americaFlag2.setPrimaryColor(Color.BLUE);
        americaFlag2.setSecondaryColor(Color.dimColor(Color.BLUE));
        for (PrismAnimations.Pulse cell : romaniaFlagFull) {
            cell.setBrightness(brightness);
        }
        int i = 0;
        prism.clearAllAnimations();
        for (PrismAnimations.Pulse cell : ballCellsAnimation) {
            cell.setBrightness(brightness);
            cell.setPrimaryColor(teamColor);
            cell.setSecondaryColor(Color.dimColor(teamColor));
            i++; //todo: add documentation
        }
        queueCell0.setIndexes(10, 11);
        queueCell1.setIndexes(12, 13);
        queueCell2.setIndexes(16, 17);
        for (PrismAnimations.Pulse queueCell : queueCellsAnimation) {
            queueCell.setBrightness(brightness);
            queueCell.setPrimaryColor(teamColor);
            queueCell.setSecondaryColor(Color.dimColor(teamColor));
        }
        climbCell.setIndexes(0, 23);
        climbCell.setSpeed(0.18f);
//        printColors(false);
        bigRed.setBrightness(brightness);
        bigRed.setIndexes(0, 17);
        bigRed.setPrimaryColor(Color.RED);
        teamColorAnimation.setPrimaryColor(teamColor);
        teamColorAnimation.setSecondaryColor(Color.dimColor(teamColor, 8));
        teamColorAnimation.setSparkleProbability(3);
        teamColorAnimation.setPeriod(100);
        teamColorAnimation.setBrightness(brightness);
        teamColorAnimation.setIndexes(0, 23);
        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(1, teamColorAnimation);
    }

    public void setBallColors(BallColor[] ballColor, BallColor[] queuedBalls, boolean basicallyHas3, boolean isInSmartShoot) {
        if (bigRedBoolean && bigRedTimer.getElapsedTimeSeconds() > 0.2) {
            bigRedBoolean = false;
            ballCellsAnimation = updateBallCellsAnimation(ballColor, ballCellsAnimation);
            queueCellsAnimation = updateBallCellsAnimation(queuedBalls, queueCellsAnimation);
            printColors(isInSmartShoot);
        }
        if (Arrays.equals(lastBallColors, ballColor) && Arrays.equals(lastQueuedBallColors, queuedBalls) && isInSmartShoot == lastSmartShoot && basicallyHas3 == lastBasicallyHas3) {
            return;
        }
        if ((!isListFull(lastBallColors) && isListFull(ballColor) || (!lastBasicallyHas3 && basicallyHas3))) {
            prism.clearAllAnimations();
            prism.insertAndUpdateAnimation(0, bigRed);
            bigRedBoolean = true;
            bigRedTimer.resetTimer();
            lastBallColors = ballColor.clone();
            return;
        }

        if (isInSmartShoot) {
            cell0.setIndexes(0, 1);
            cell1.setIndexes(4, 5);
            cell2.setIndexes(6, 7); //6767676767676767676767676767676767676767676767676767676767
            queueCellsAnimation = updateBallCellsAnimation(queuedBalls, queueCellsAnimation);

        } else {
            cell0.setIndexes(0, 5);
            cell1.setIndexes(6, 11);
            cell2.setIndexes(12, 17);
        }

        ballCellsAnimation = updateBallCellsAnimation(ballColor, ballCellsAnimation);

        printColors(isInSmartShoot);
        lastBallColors =  ballColor.clone();
        lastQueuedBallColors = queuedBalls.clone();
        lastSmartShoot = isInSmartShoot;
    }

    // Required setBallColors() to be called on loop to work
    public void printBigRed() {
        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(0, bigRed);
        bigRedBoolean = true;
        bigRedTimer.resetTimer();
    }

    public boolean isListFull(BallColor[] ballColors) {
        for (BallColor color : ballColors) {
            if (color == BallColor.None) {
                return false;
            }
        }
        return true;
    }

    public void printColors() {
        printColors(lastSmartShoot);
    }
    public void printColors(boolean isInSmartShoot) {
        prism.clearAllAnimations();
        if (!climbLights) {
            for (int i = 0; i < 3; i++) {
                prism.insertAndUpdateAnimation(i, ballCellsAnimation[i]);
            }
            if (isInSmartShoot) {
                for (int i = 0; i < 3; i++) {
                    prism.insertAndUpdateAnimation(i + 3, queueCellsAnimation[i]);
                }
            }
        } else {
            prism.insertAndUpdateAnimation(4, climbCell);
            Random random = new Random();
//            if (random.nextBoolean()) {
//                for (int i = 0; i < 3; i++) {
//                    prism.insertAndUpdateAnimation(i + 5, romaniaFlagFull[i]);
//                }
//            } else {
//                for (int i = 0; i < 3; i++) {
//                    prism.insertAndUpdateAnimation(i + 5, americaFlagFull[i]);
//                }
//            }
        }

    }

    public PrismAnimations.Pulse[] updateBallCellsAnimation(BallColor[] ballColorsToUse, PrismAnimations.Pulse[] listToChange) {
        for (int i = 0; i < 3; i++) {
            switch (ballColorsToUse[i]) {
                case Purple:
                    listToChange[i].setPrimaryColor(purple);
                    listToChange[i].setSecondaryColor(Color.dimColor(purple));
                    break;
                case Green:
                    listToChange[i].setPrimaryColor(green);
                    listToChange[i].setSecondaryColor(Color.dimColor(green));
                    break;
                case Red:
                    listToChange[i].setPrimaryColor(red);
                    listToChange[i].setSecondaryColor(Color.dimColor(red));
                    break;
                case Blue:
                    listToChange[i].setPrimaryColor(blue);
                    listToChange[i].setSecondaryColor(Color.dimColor(blue));
                    break;
                case Orange:
                    listToChange[i].setPrimaryColor(orange);
                    listToChange[i].setSecondaryColor(Color.dimColor(orange));
                    break;
                case Yellow:
                    listToChange[i].setPrimaryColor(yellow);
                    listToChange[i].setSecondaryColor(Color.dimColor(yellow));
                    break;
                case Any:
                    listToChange[i].setPrimaryColor(teamColor);
                    listToChange[i].setSecondaryColor(Color.dimColor(teamColor));
                    break;
                case None:
                    listToChange[i].setPrimaryColor(Color.TRANSPARENT);
                    listToChange[i].setSecondaryColor(Color.TRANSPARENT);
            }
        }
        return listToChange;
    }
    public void setClimbLights(boolean set) {
        climbLights = set;
        printColors();
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

    public void stop() {
        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(0, teamColorAnimation);
    }
}
