package org.firstinspires.ftc.teamcode.util;

public enum IndexerEnums {
    // Orders for Storing positions from topLeft to bottom To topRight
    StoreTlBTr012Revrese, 
    StoreTlBTr120,
    StoreTlBTr201,
    StoreTlBTr012,
    // Order for Shooting positions from top to bottomLeft To bottomRight
    TransferTBlBr012Reverse,
    TransferTBlBr120,
    TransferTBlBr201,
    TransferTBlBr012 // todo remove if we dont want it

    // Store States
    //Reverse
    // 0   2 | 1   0 | 2   1 | 0   2 
    //   1   |   2   |   0   |   1   
    // Transfer States 
    //    Reverse                    V Doesn't currently exist
    //       0   |   1   |   2   |   0   
    //     1   2 | 2   0 | 0   1 | 1   2 
}