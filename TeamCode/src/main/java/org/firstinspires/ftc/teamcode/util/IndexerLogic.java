package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IndexerState;

import java.util.ArrayList;
import java.util.List;

public class IndexerLogic {
    DcMotorEx feeder;
    double lastTransferPower = 0;

    public IndexerState indexerState;
    List<BallColor> queue = new ArrayList<>();
    boolean lastHasShot = false;

    public IndexerLogic(HardwareMap hardwareMap, int startIndexerTicks) {
        this(hardwareMap, new BallColor[]{BallColor.None, BallColor.None, BallColor.None}, startIndexerTicks);
    }

    public IndexerLogic(HardwareMap hardwareMap, BallColor[] ballCells, int startIndexerTicks) {
        indexerState = new IndexerState(hardwareMap, ballCells, startIndexerTicks);

        feeder = hardwareMap.get(DcMotorEx.class, "transfer");
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public boolean allowIntakeing() {
        return indexerState.atPosition;
    }

    public void addToBackQueue(BallColor color) {
        queue.add(color);
    }

    public BallColor getFrontQueue() {
        return queue.get(0);
    }

    public void removeFrontQueue() {
        queue.remove(0);
    }

    public BallColor queuePop() {
        BallColor color = getFrontQueue();
        removeFrontQueue();
        return color;
    }

    public List<BallColor> getQueue() {
        return queue;
    }


    public void update(boolean hasShot, boolean isIntaking, boolean readyToShoot) {
        //
        if (hasShot && !lastHasShot) {
            lastHasShot = true;
        } else if (!hasShot) {
            lastHasShot = false;
        } else {
            hasShot = false;
        }

        // Removing after shoot
        if (hasShot && IndexerEnumsNew.isAShootEnum(indexerState.getIndexerPosition()) && indexerState.atPosition) {
            int indexToRemove = IndexerEnumsNew.getIndex(indexerState.getIndexerPosition());
            indexerState.removeBallAtIndex(indexToRemove);
            removeFrontQueue();
        }

        // move to nearest ball of right color
        if (isIntaking) {
            if (!indexerState.isBallCellsFull()) {
                if (indexerState.getColor() != BallColor.None) {
                    indexerState.setBallCellAtIntakeToRightColor();
                }
                indexerState.moveToNearest(BallColor.None, true);
            } else {
                isIntaking = false;
            }
        }
        if (!isIntaking) {
            if (indexerState.isBallCellsEmpty()) {
                update(hasShot, true, readyToShoot);
                return;
            } else {
                if (queue.isEmpty()) {
                    // auto fire case
                    indexerState.moveToNearest(BallColor.Any, false);
                } else {
                    indexerState.moveToNearest(getFrontQueue(), false);
                }
            }
        }

        // spin transfer wheel
        if (!isIntaking && indexerState.atPosition && readyToShoot) {
            setIndexerToShooterPower(Constants.Indexer.transferPower);
        } else {
            setIndexerToShooterPower(0);
        }

        indexerState.update();
    }

    public void setIndexerToShooterPower(double set) {
        if (lastTransferPower != set) {
            lastTransferPower = set;
            feeder.setPower(set);
        }
    }
}
