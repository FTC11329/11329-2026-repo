package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Bulk {

    public static void auto(HardwareMap hardwareMap) {
        setCache(hardwareMap, LynxModule.BulkCachingMode.AUTO);
    }

    public static void setCache(
            HardwareMap hardwareMap,
            LynxModule.BulkCachingMode mode
    ) {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(mode);
        }
    }
}