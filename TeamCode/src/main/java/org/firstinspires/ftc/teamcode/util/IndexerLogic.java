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
    public boolean started = false;
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
        if (manualShootButton && readyToShootMotors && !started) {
            started = true;
        }
        if (started) {
            feeder.setPower(1);
            if (indexerState.unload()) {
                started = false;
                feeder.setPower(0);
            }
        }
        if (indexerState.atPosition && isIntaking && !started) {
            if (!indexerState.isBallCellsFull()) {
                if (indexerState.getColor() != BallColor.None) {
                    indexerState.setBallCellAtIntakeToRightColor();
                }
                if (!indexerState.isBallCellsFull()) {
                    indexerState.moveToNearest(BallColor.None, true);
                }
            }

        }

        if  (!started) {indexerState.update();}
    }

    public void setIndexerToShooterPower(double set) {
        if (lastTransferPower != set) {
            lastTransferPower = set;
            feeder.setPower(set);
        }
    }
}
