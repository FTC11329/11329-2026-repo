package org.firstinspires.ftc.teamcode.util.shooterInterpolation;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterTestValues implements ShooterValuesParent {
    private final ShooterTreeInterpolation shooterMap = new ShooterTreeInterpolation();

    private final double ADDITION = 0; // + shooter farther
    private final double FAR_ADDITION = 0; // + shooter farther
    private final double FAR_RPM_ADDITION = -50; // + shooter farther
    public ShooterTestValues() {
        // Distance units must have a consistent unit
        shooterMap.put(20.2 + ADDITION, new ShooterState(1920, 9, .8125));
        shooterMap.put(25.2 + ADDITION, new ShooterState(1860, 12, .786));
        shooterMap.put(30.2 + ADDITION, new ShooterState(1770, 19.1, .672));
        shooterMap.put(35.2 + ADDITION, new ShooterState(1750, 24.1, .598));
        shooterMap.put(40.2 + ADDITION, new ShooterState(1800, 25.1, .664));
        shooterMap.put(45.2 + ADDITION, new ShooterState(1783,30,.602));
        shooterMap.put(50.2 + ADDITION, new ShooterState(1872,30,.684));
        shooterMap.put(55.2 + ADDITION, new ShooterState(1900,30.6,.692));
        shooterMap.put(60.2 + ADDITION, new ShooterState(1940,31,.694));
        shooterMap.put(65.2 + ADDITION, new ShooterState(1990,31.6,.68));
        shooterMap.put(70.2 + ADDITION, new ShooterState(2076,35,.67 /*SIX SEVEN INTERROBANG*/));
        shooterMap.put(75.2 + ADDITION, new ShooterState(2100,35,.718));
        shooterMap.put(80.2 + ADDITION, new ShooterState(2140,35.3,.748));
        shooterMap.put(85.2 + ADDITION, new ShooterState(2200,35.7,.756));
        shooterMap.put(90.2 + ADDITION, new ShooterState(2250,36.2,.754));
        shooterMap.put(95.2 + ADDITION, new ShooterState(2300,37.3,.766));
        shooterMap.put(100.2 + ADDITION, new ShooterState(2350,36.3,.718));
        shooterMap.put(105.2 + ADDITION, new ShooterState(2450,35.3,.7575));
        shooterMap.put(110.2 + ADDITION, new ShooterState(2450 + FAR_RPM_ADDITION,34.3,.72));
        shooterMap.put(115.2 + ADDITION, new ShooterState(2600 + FAR_RPM_ADDITION,36.3,.718));
        shooterMap.put(120.2 + ADDITION, new ShooterState(2660 + FAR_RPM_ADDITION,30,.892));
        shooterMap.put(125.2 + ADDITION, new ShooterState(2700 + FAR_RPM_ADDITION,29.5,.836));
        shooterMap.put(130.2 + ADDITION, new ShooterState(2822 + FAR_RPM_ADDITION,29,.914));
        shooterMap.put(135.2 + ADDITION, new ShooterState(2914 + FAR_RPM_ADDITION,29,.876));
        shooterMap.put(140.2 + ADDITION, new ShooterState(2911 + FAR_RPM_ADDITION,29,.914));
        shooterMap.put(145.2 + ADDITION, new ShooterState(2940 + FAR_RPM_ADDITION,26,1.002));
        shooterMap.put(150.2 + ADDITION, new ShooterState(3058 + FAR_RPM_ADDITION,26,1.004));
    }

    public ShooterState get(double distance) {
        return shooterMap.get(distance);
    }
}

