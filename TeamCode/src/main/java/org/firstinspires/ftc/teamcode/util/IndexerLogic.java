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
    public DcMotorEx feeder;
    double lastTransferPower = 0;

    public IndexerState indexerState;
    List<BallColor> queue = new ArrayList<>();
    boolean lastHasShot = false;
    boolean smartShooterBool = true;
    boolean manualShootToggle = false;
    boolean readyToManualShootToggle = false;

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
        if (!queue.isEmpty()) {
            queue.remove(0);
        }
    }

    public BallColor queuePop() {
        BallColor color = getFrontQueue();
        removeFrontQueue();
        return color;
    }

    public List<BallColor> getQueue() {
        return queue;
    }

    public void setSmartShootBool(boolean set) {
        smartShooterBool = false;
    }


    public void update(boolean hasShot, boolean isIntaking, boolean readyToShootMotors, boolean readyToShootPos, boolean manualShootButton) {
        boolean readyToShoot = readyToShootMotors && readyToShootPos;

        if (manualShootToggle) {
            manualShootButton = true;
        }

        if (hasShot && !lastHasShot) {
            lastHasShot = true;
        } else if (!hasShot) {
            lastHasShot = false;
        } else {
            hasShot = false;
        }

        if (smartShooterBool) {
            // Removing after shot
            if (hasShot && IndexerEnums.isAShootEnum(indexerState.getIndexerPosition()) && indexerState.atPosition) {
                int indexToRemove = IndexerEnums.getIndex(indexerState.getIndexerPosition());
                indexerState.removeBallAtIndex(indexToRemove);
                removeFrontQueue();
            }
        }

        // move to nearest ball of right color
        if (indexerState.atPosition) {
            if (isIntaking) {
                if (!indexerState.isBallCellsFull()) {
                    if (indexerState.getColor() != BallColor.None) {
                        indexerState.setBallCellAtIntakeToRightColor();
                    }
                    if (!indexerState.isBallCellsFull()) {
                        indexerState.moveToNearest(BallColor.None, true);
                    }
                } else {
                    isIntaking = false;
                }
            }
            if (smartShooterBool) {
                if (!isIntaking) {
                    if (indexerState.isBallCellsEmpty()) {
                        update(hasShot, true, readyToShootMotors, readyToShootPos, manualShootButton);
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
                if (!isIntaking && indexerState.atPosition && readyToShoot) {
                    setIndexerToShooterPower(Constants.Indexer.transferPower);
                } else {
                    setIndexerToShooterPower(0);
                }
            }
            if (smartShooterBool) {
                if (manualShootButton) {
                    readyToManualShootToggle = true;
                }
                if (readyToManualShootToggle && readyToShootMotors) {
                    readyToManualShootToggle = false;
                    manualShootToggle = true;
                    indexerState.spinIndexerOnce();
                }
                if (manualShootToggle) {
                    if (!readyToShootMotors) {
                        indexerState.stopIndexer(true);
                        setIndexerToShooterPower(0);
                    } else {
                        indexerState.stopIndexer(false);
                        setIndexerToShooterPower(Constants.Indexer.transferPower);
                    }
                    if (indexerState.atPosition) {
                        setIndexerToShooterPower(0);
                        indexerState.stopIndexer(false);
                        manualShootToggle = false;
                        indexerState.setBallCells(new BallColor[]{BallColor.None, BallColor.None, BallColor.None});
                    }
                }
            }
        }

        // spin transfer wheel
        indexerState.update();
    }

    public void setIndexerToShooterPower(double set) {
        if (lastTransferPower != set) {
            lastTransferPower = set;
            feeder.setPower(set);
        }
    }
}
