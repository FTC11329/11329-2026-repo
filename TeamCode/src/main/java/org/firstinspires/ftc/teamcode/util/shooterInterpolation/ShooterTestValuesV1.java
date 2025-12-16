package org.firstinspires.ftc.teamcode.util.shooterInterpolation;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterTestValuesV1 implements ShooterValuesParent {
    private final ShooterTreeInterpolation shooterMap = new ShooterTreeInterpolation();

    private final double ADDITION = 2; // + shooter farther
    private final double FAR_ADDITION = 3; // + shooter farther
    public ShooterTestValuesV1() {
        // Distance units must have a consistent unit
        shooterMap.put(36 + ADDITION, new ShooterState(2200, 19.1, 0.43));
        shooterMap.put(43.76 + ADDITION, new ShooterState(2332, 20.3, 0.67));
        shooterMap.put(52.5 + ADDITION, new ShooterState(2308, 24.2, 0.71));
        shooterMap.put(65 + ADDITION, new ShooterState(2336, 34.4, 0.40));
        shooterMap.put(71.6 + ADDITION, new ShooterState(2336, 35, 0.44));
        shooterMap.put(75.8 + ADDITION, new ShooterState(2420, 35, 0.47));
        shooterMap.put(82.7 + ADDITION, new ShooterState(2500, 32.3, 0.58));
        shooterMap.put(90.5 + ADDITION + FAR_ADDITION * 0.2, new ShooterState(2620, 35.3, 0.62));
        shooterMap.put(117.23 + ADDITION + FAR_ADDITION* 0.5, new ShooterState(2948, 35.3, 0.7));
        shooterMap.put(131.5 + ADDITION + FAR_ADDITION * 0.8, new ShooterState(3344, 29.9, 0.78));
        shooterMap.put(143 + ADDITION + FAR_ADDITION * 1, new ShooterState(3400, 29.9, 0.78));
        shooterMap.put(153.69 + ADDITION + FAR_ADDITION * 1, new ShooterState(3460, 30.2, 1));
    }

    public ShooterState get(double distance) {
        return shooterMap.get(distance);
    }
}

