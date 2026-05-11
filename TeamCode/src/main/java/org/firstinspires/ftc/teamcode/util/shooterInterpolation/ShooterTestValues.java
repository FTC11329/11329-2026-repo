package org.firstinspires.ftc.teamcode.util.shooterInterpolation;

import com.bylazar.configurables.annotations.Configurable;

import java.util.List;

@Configurable
public class ShooterTestValues implements ShooterValuesParent {
    private final ShooterTreeInterpolation shooterMap = new ShooterTreeInterpolation();

    private final double CLOSE_ADDITION = -4.5; // - shooter farther
    private final double ADDITION = 5.5; // - shooter farther
    private final double FAR_ADDITION = 1; // - shooter farther
    private final double RPM_MULTIPLYER = 1; // + shooter farther
    private final double FAR_ANGLE_ADDITION = 0; // + shooter farther
    public ShooterTestValues() {
        // Distance units must have a consistent unit
        shooterMap.put(25 + ADDITION + CLOSE_ADDITION, new ShooterState(2100 * RPM_MULTIPLYER, 16, 0.76));
        shooterMap.put(35 + ADDITION + CLOSE_ADDITION, new ShooterState(2050 * RPM_MULTIPLYER, 21, 0.594));
        shooterMap.put(45 + ADDITION + CLOSE_ADDITION, new ShooterState(2120 * RPM_MULTIPLYER, 24, 0.646));
        shooterMap.put(55. + ADDITION + CLOSE_ADDITION, new ShooterState(2200 * RPM_MULTIPLYER, 30.5, 0.635));
        shooterMap.put(65. + ADDITION + CLOSE_ADDITION, new ShooterState(2270 * RPM_MULTIPLYER, 34, 0.656));
        shooterMap.put(75. + ADDITION, new ShooterState(2400 * RPM_MULTIPLYER,36,0.626));
        shooterMap.put(85. + ADDITION, new ShooterState(2550 * RPM_MULTIPLYER,37,0.76));
        shooterMap.put(95. + ADDITION, new ShooterState(2600 * RPM_MULTIPLYER,37,0.706));
        shooterMap.put(105 + ADDITION, new ShooterState(2700 * RPM_MULTIPLYER,38,0.732));
        shooterMap.put(115 + ADDITION + FAR_ADDITION, new ShooterState(2870 * RPM_MULTIPLYER,38+ FAR_ANGLE_ADDITION,0.834));
        shooterMap.put(125. + ADDITION + FAR_ADDITION, new ShooterState(3030 * RPM_MULTIPLYER,39.3+ FAR_ANGLE_ADDITION,0.922));
        shooterMap.put(135. + ADDITION + FAR_ADDITION, new ShooterState(3100 * RPM_MULTIPLYER,36.5+ FAR_ANGLE_ADDITION,0.916));
        shooterMap.put(145. + ADDITION + FAR_ADDITION, new ShooterState(3240 * RPM_MULTIPLYER,38+ FAR_ANGLE_ADDITION,0.956));
        shooterMap.put(155. + ADDITION + FAR_ADDITION, new ShooterState(3400 * RPM_MULTIPLYER,38+ FAR_ANGLE_ADDITION,0.940));
        shooterMap.put(165.+ ADDITION + FAR_ADDITION, new ShooterState(3530 * RPM_MULTIPLYER,40.5+ FAR_ANGLE_ADDITION, 0.972));
    }

    public ShooterState get(double distance) {
        return shooterMap.get(distance);
    }

    public List<Double> getDistances(){
        return shooterMap.getKeys();
    }
}

