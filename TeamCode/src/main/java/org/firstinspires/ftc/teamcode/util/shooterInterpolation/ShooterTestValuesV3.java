package org.firstinspires.ftc.teamcode.util.shooterInterpolation;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterTestValuesV3 implements ShooterValuesParent {
    private final ShooterTreeInterpolation shooterMap = new ShooterTreeInterpolation();

    private final double ADDITION = 0; // + shooter farther
    private final double FAR_ADDITION = 0; // + shooter farther
    public ShooterTestValuesV3() {
        // Distance units must have a consistent unit
        //todo: distance is calculated by edge of goal to center of robot drivetrain with the intake facing the goal
        shooterMap.put(13, new ShooterState(1920, 9, .8125));
        shooterMap.put(18, new ShooterState(1860, 12, .786));
        shooterMap.put(23, new ShooterState(1770, 19.1, .672));
        shooterMap.put(28, new ShooterState(1750, 24.1, .598));
        shooterMap.put(33, new ShooterState(1800, 25.1, .664));
        shooterMap.put(38, new ShooterState(1783,30,.602));
        shooterMap.put(43, new ShooterState(1872,30,.684));
        shooterMap.put(48, new ShooterState(1900,30.6,.692));
        shooterMap.put(53, new ShooterState(1940,31,.694));
        shooterMap.put(58, new ShooterState(1990,31.6,.68));
        shooterMap.put(63, new ShooterState(2076,35,.67 /*SIX SEVEN INTERROBANG*/));
        shooterMap.put(68, new ShooterState(2100,35,.718));
        shooterMap.put(73, new ShooterState(2140,35.3,.748));
        shooterMap.put(78, new ShooterState(2200,35.7,.756));
        shooterMap.put(83, new ShooterState(2250,36.2,.754));
    }

    public ShooterState get(double distance) {
        return shooterMap.get(distance);
    }
}

