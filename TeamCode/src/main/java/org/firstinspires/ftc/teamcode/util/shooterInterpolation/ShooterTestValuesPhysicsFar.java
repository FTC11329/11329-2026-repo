package org.firstinspires.ftc.teamcode.util.shooterInterpolation;

import com.bylazar.configurables.annotations.Configurable;

import java.util.List;

@Configurable
public class ShooterTestValuesPhysicsFar implements ShooterValuesParent {
    private final ShooterTreeInterpolation shooterMap = new ShooterTreeInterpolation();

    private final double CLOSE_ADDITION = 0; // - shooter farther
    private final double ADDITION = 0; // - shooter farther
    private final double FAR_ADDITION = 0; // - shooter farther
    private final double FAR_RPM_ADDITION = 0; // + shooter farther
    private final double FAR_ANGLE_ADDITION = 0; // + shooter farther
    public ShooterTestValuesPhysicsFar() {
        // Distance units must have a consistent unit
        shooterMap.put(21.8 + ADDITION + CLOSE_ADDITION, new ShooterState(1770, 10, .854));
        shooterMap.put(31.5 + ADDITION + CLOSE_ADDITION, new ShooterState(1587, 20, .652));
        shooterMap.put(41.6 + ADDITION + CLOSE_ADDITION, new ShooterState(1643, 28, .586));
        shooterMap.put(51.8 + ADDITION + CLOSE_ADDITION, new ShooterState(1664, 35, .602));
        shooterMap.put(62.7 + ADDITION + CLOSE_ADDITION, new ShooterState(1794, 40, .516));
        shooterMap.put(71.9 + ADDITION + CLOSE_ADDITION, new ShooterState(1920,42.8,.510));
        shooterMap.put(82.4 + ADDITION, new ShooterState(2040,43.3,.636));
        shooterMap.put(88.5 + ADDITION, new ShooterState(2060,40.8,.604));
        shooterMap.put(101 + ADDITION, new ShooterState(2280,40.9,.642));
        shooterMap.put(112 + FAR_ADDITION, new ShooterState(2360 + FAR_RPM_ADDITION,42.5+ FAR_ANGLE_ADDITION,0.676));
        shooterMap.put(117.2 + FAR_ADDITION, new ShooterState(2500 + FAR_RPM_ADDITION,41.2+ FAR_ANGLE_ADDITION,.66));
        shooterMap.put(130.8 + FAR_ADDITION, new ShooterState(2600 + FAR_RPM_ADDITION,42+ FAR_ANGLE_ADDITION,.686));
        shooterMap.put(143.3 + FAR_ADDITION, new ShooterState(2820 + FAR_RPM_ADDITION,42.6+ FAR_ANGLE_ADDITION,.71));
        shooterMap.put(152.8 + FAR_ADDITION, new ShooterState(3100 + FAR_RPM_ADDITION,42.8+ FAR_ANGLE_ADDITION,.75));
        shooterMap.put(162.0 + FAR_ADDITION, new ShooterState(3000 + FAR_RPM_ADDITION,43.9+ FAR_ANGLE_ADDITION,.772));
    }

    public ShooterState get(double distance) {
        return shooterMap.get(distance);
    }

    public List<Double> getDistances(){
        return shooterMap.getKeys();
    }
}

