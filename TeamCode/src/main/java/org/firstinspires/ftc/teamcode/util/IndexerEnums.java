package org.firstinspires.ftc.teamcode.util;

public enum IndexerEnums {
    intake0, intake1, intake2, intake3,
    shoot0, shoot1, shoot2;

    // 2   1 | 0   2 | 1   0 | 2   1 |
    //   0   |   1   |   2   |   0   |
    public static IndexerEnums getEnum(int index, boolean shootIndex) {
        if (!shootIndex) {
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
        } else {
            switch (index) {
                case 0:
                    return shoot0;
                case 1:
                    return shoot1;
                case 2:
                    return shoot2;
            }
        }
        throw new RuntimeException("getEnum failed, idk why, it shouldnt");
    }

    public static int getIndex(IndexerEnums indexerEnums) {
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

    public static double convertEnumToPercentOfRot(IndexerEnums indexerPos) {
        switch (indexerPos) {
            case intake0:
                return 0;
            case intake1:
                return 1.0 / 3.0;
            case intake2:
                return 2.0 / 3.0;
            case intake3:
                return 1;

            case shoot0:
                return 1.0 / 2.0;
            case shoot1:
                return 5.0 / 6.0;
            case shoot2:
                return 1.0 / 6.0;
        }
        throw new RuntimeException("Shouldn't error ever, in the indexer");
    }

    public static boolean isAShootEnum(IndexerEnums indexerPos) {
        switch (indexerPos) {
            case shoot0:
            case shoot1:
            case shoot2:
                return true;
            case intake0:
            case intake1:
            case intake2:
            case intake3:
            default:
                return false;
        }
    }
}