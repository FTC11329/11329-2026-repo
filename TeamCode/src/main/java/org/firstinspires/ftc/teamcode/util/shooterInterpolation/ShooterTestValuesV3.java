package org.firstinspires.ftc.teamcode.util.shooterInterpolation;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterTestValuesV3 implements ShooterValuesParent {
    private final ShooterTreeInterpolation shooterMap = new ShooterTreeInterpolation();

    private final double ADDITION = 0; // + shooter farther
    private final double FAR_ADDITION = 0; // + shooter farther
    public ShooterTestValuesV3() {
        // Distance units must have a consistent unit
        shooterMap.put(58, new ShooterState(2537, 49.8, 0.55));
    }

    public ShooterState get(double distance) {
        return shooterMap.get(distance);
    }
}

