package org.firstinspires.ftc.teamcode.util;

public enum IndexerEnumsNew {
    // Orders for Storing positions from topLeft to bottom To topRight
    intake0, intake1, intake2,
    shoot0, shoot1, shoot2, initial;

    // 0   2 | 1   0 | 2   1
    //   1   |   2   |   0

    //       0   |   1   |   2
    //     1   2 | 2   0 | 0   1
    public static IndexerEnumsNew getEnum(int index, boolean isAnIntakePosition) {
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
        System.exit(0);

        return getEnum(index, isAnIntakePosition);
    }

    public static int getIndex(IndexerEnumsNew indexerEnums) {
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
        System.exit(0);

        return getIndex(indexerEnums);
    }

    public static boolean isAShootEnum(IndexerEnumsNew indexerEnums) {
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