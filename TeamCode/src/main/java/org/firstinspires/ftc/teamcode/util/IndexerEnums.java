package org.firstinspires.ftc.teamcode.util;

public enum IndexerEnums {
    // Orders for Storing positions from topLeft to bottom To topRight
    intake0, intake1, intake2,
    shoot0, shoot1, shoot2;

    // 0   2 | 1   0 | 2   1
    //   1   |   2   |   0

    //       0   |   1   |   2
    //     1   2 | 2   0 | 0   1
    public static IndexerEnums getEnum(int index, boolean isAnIntakePosition) {
        if (isAnIntakePosition) {
            switch (index) {
                case 0:
                    return intake0;
                case 1:
                    return intake1;
                case 2:
                    return intake2;
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

            case shoot0:
                return 0;
            case shoot1:
                return 1;
            case shoot2:
                return 2;
        }
        throw new RuntimeException("getIndex failed, idk why, it shouldnt");

    }

    public static boolean isAShootEnum(IndexerEnums indexerEnums) {
        switch (indexerEnums) { 
            case intake0:
            case intake1:
            case intake2:
                return false;
            default:
                return true;
        }
    }
}