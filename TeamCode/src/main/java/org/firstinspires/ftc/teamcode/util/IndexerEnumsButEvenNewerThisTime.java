package org.firstinspires.ftc.teamcode.util;

public enum IndexerEnumsButEvenNewerThisTime {
    intake0, intake1, intake2, intake3;

    // 2   1 | 0   2 | 1   0 | 2   1 |
    //   0   |   1   |   2   |   0   |
    public static IndexerEnumsButEvenNewerThisTime getEnum(int index) {
        switch (index) {
            case 0:
                return intake0;
            case 1:
                return intake1;
            case 2:
                return intake2;
            case 3:
                return intake3;
        }
        throw new RuntimeException("getEnum failed, idk why, it shouldnt");

    }

    public static int getIndex(IndexerEnumsButEvenNewerThisTime indexerEnums) {
        switch (indexerEnums) {
            case intake0:
                return 0;
            case intake1:
                return 1;
            case intake2:
                return 2;
            case intake3:
                return 3;
        }
        throw new RuntimeException("getIndex failed, idk why, it shouldnt");

    }

    public static double convertEnumToPercentOfRot(IndexerEnumsButEvenNewerThisTime indexerPos) {
        switch (indexerPos) {
            case intake0:
                return 0;
            case intake1:
                return 1.0/3.0;
            case intake2:
                return 2.0/3.0;
            case intake3:
                return 1;
        }
        throw new RuntimeException("Shouldn't error ever, in the indexer");
    }
}