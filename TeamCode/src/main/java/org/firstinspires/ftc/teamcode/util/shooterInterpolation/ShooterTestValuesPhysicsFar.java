package org.firstinspires.ftc.teamcode.util.shooterInterpolation;

import com.bylazar.configurables.annotations.Configurable;

import java.util.List;

@Configurable
public class ShooterTestValuesPhysicsFar implements ShooterValuesParent {
    private final ShooterTreeInterpolation shooterMap = new ShooterTreeInterpolation();

    private final double CLOSE_ADDITION = -4; // - shooter farther
    private final double ADDITION = -8; // - shooter farther
    private final double FAR_ADDITION = 0; // - shooter farther
    private final double FAR_RPM_ADDITION = 0; // + shooter farther
    private final double FAR_ANGLE_ADDITION = 0; // + shooter farther
    public ShooterTestValuesPhysicsFar() {
        // Distance units must have a consistent unit
        shooterMap.put(20.2 + ADDITION + CLOSE_ADDITION, new ShooterState(1920, 9, .8125));
        shooterMap.put(25.2 + ADDITION + CLOSE_ADDITION, new ShooterState(1860, 12, .786));
        shooterMap.put(30.2 + ADDITION + CLOSE_ADDITION, new ShooterState(1770, 19.1, .672));
        shooterMap.put(35.2 + ADDITION + CLOSE_ADDITION, new ShooterState(1750, 24.1, .598));
        shooterMap.put(40.2 + ADDITION + CLOSE_ADDITION, new ShooterState(1800, 25.1, .664));
        shooterMap.put(45.2 + ADDITION + CLOSE_ADDITION, new ShooterState(1783,30,.602));
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
        shooterMap.put(110.2 + FAR_ADDITION, new ShooterState(2810 + FAR_RPM_ADDITION,42.6+ FAR_ANGLE_ADDITION,0.665));
        shooterMap.put(115.2 + FAR_ADDITION, new ShooterState(2855 + FAR_RPM_ADDITION,43.3+ FAR_ANGLE_ADDITION,0.674));
        shooterMap.put(120.2 + FAR_ADDITION, new ShooterState(2900 + FAR_RPM_ADDITION,44.1+ FAR_ANGLE_ADDITION,0.683));
        shooterMap.put(125.2 + FAR_ADDITION, new ShooterState(2945 + FAR_RPM_ADDITION,44.7+ FAR_ANGLE_ADDITION,0.692));
        shooterMap.put(130.2 + FAR_ADDITION, new ShooterState(2990 + FAR_RPM_ADDITION,45.3+ FAR_ANGLE_ADDITION,0.700));
        shooterMap.put(135.2 + FAR_ADDITION, new ShooterState(3035 + FAR_RPM_ADDITION,45.9+ FAR_ANGLE_ADDITION,0.709));
        shooterMap.put(140.2 + FAR_ADDITION, new ShooterState(3080 + FAR_RPM_ADDITION,46.4+ FAR_ANGLE_ADDITION,0.717));
        shooterMap.put(145.2 + FAR_ADDITION, new ShooterState(3128 + FAR_RPM_ADDITION,46.9+ FAR_ANGLE_ADDITION,0.726));
        shooterMap.put(150.2 + FAR_ADDITION, new ShooterState(3175 + FAR_RPM_ADDITION,47.3+ FAR_ANGLE_ADDITION,0.734));
        shooterMap.put(155 + FAR_ADDITION, new ShooterState(3220 + FAR_RPM_ADDITION,47.8+ FAR_ANGLE_ADDITION,.75));
        shooterMap.put(160 + FAR_ADDITION, new ShooterState(3265 + FAR_RPM_ADDITION,48.2+ FAR_ANGLE_ADDITION,.75));
    }

    public ShooterState get(double distance) {
        return shooterMap.get(distance);
    }

    public List<Double> getDistances(){
        return shooterMap.getKeys();
    }
}

