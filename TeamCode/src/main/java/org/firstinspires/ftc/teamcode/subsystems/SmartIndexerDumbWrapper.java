package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.IndexerEnums;

public class SmartIndexerDumbWrapper {
    SmartIndexerWNoTRev smartIndexer;

    IndexerEnums[] indexerList = new IndexerEnums[]{
                IndexerEnums.StoreTlBTr012Revrese,
                IndexerEnums.TransferTBlBr012Reverse,
                IndexerEnums.StoreTlBTr120,
                IndexerEnums.TransferTBlBr120,
                IndexerEnums.StoreTlBTr201,
                IndexerEnums.TransferTBlBr201,
                IndexerEnums.StoreTlBTr012,
    };
    int index = 0;
    boolean up = false;

    public SmartIndexerDumbWrapper(HardwareMap hardwareMap) {
        smartIndexer = new SmartIndexerWNoTRev(hardwareMap);
    }

    public void upOneIndex() {
        index++;
    }
    public void downOneIndex() {
        index--;
    }

    public void update() {
        if (index > 6) {
            index -= 7;
        }
        if (index < 0) {
            index += 7;
        }
        smartIndexer.setIndexerPos(indexerList[index]);
    }

}
